#!/usr/bin/env python3
"""Collect and parse external BIOZ calibration scans from MAX30001G.ino."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
from typing import Any


DEFAULT_SCAN_COMMANDS = [
    "<",
    "m8",
    "Si0",
    "Sa8",
    "Sf1",
    "Sr0",
    "Sp1",
    "St48",
    "Sc64",
    "Bc8000",
    "a",
    ".",
]

DEFAULT_CORRECTION_SCALE_PPM = 1250650

DEFAULT_SCAN_META = {
    "scan_avg": 8,
    "scan_fast": 1,
    "scan_fullrange": 0,
    "scan_phase_range": "reduced",
    "settle_samples": 48,
    "current_change_settle_samples": 64,
    "initial_current_nA": 8000,
}

MEASUREMENT_FIELDS = [
    "run_id",
    "load_id",
    "load_description",
    "fit_hint",
    "topology",
    "component_values",
    "frequency_hz",
    "expected_real_ohm",
    "expected_imag_ohm",
    "expected_magnitude_ohm",
    "expected_phase_deg",
    "cosine_magnitude_ohm",
    "cosine_phase_deg",
    "cosine_real_ohm",
    "cosine_imag_ohm",
    "triangular_magnitude_ohm",
    "triangular_phase_deg",
    "maxabs_ohm",
    "normal_magnitude_ohm",
    "normal_phase_deg",
    "normal_reliability",
    "current_nA",
    "num_phase_points",
    "scale_expected_over_cosine",
    "cosine_magnitude_error_pct",
    "cosine_phase_error_deg",
    "scan_avg",
    "scan_fast",
    "scan_fullrange",
    "scan_phase_range",
    "settle_samples",
    "current_change_settle_samples",
    "initial_current_nA",
    "warnings",
    "raw_log_file",
    "notes",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run semi-automated external BIOZ fixture calibration scans."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "--loads-file",
        type=Path,
        default=Path(__file__).with_name("fixture_loads.json"),
        help="Fixture load manifest JSON.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("calibration_runs"),
        help="Output directory for timestamped runs.",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        help="Existing run directory to continue, for example calibration_runs/20260716_120000.",
    )
    parser.add_argument(
        "--force-rerun",
        action="store_true",
        help="Rerun requested loads even if raw logs already exist in a resumed run.",
    )
    parser.add_argument(
        "--load",
        action="append",
        dest="load_ids",
        help="Load ID to run. May be repeated. Defaults to every load in the manifest.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not open serial. Write theoretical rows only.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=240.0,
        help="Seconds to wait for one scan to complete.",
    )
    parser.add_argument(
        "--expected-spectrum-rows",
        type=int,
        default=8,
        help="Expected rows in the normal reduced-range spectrum.",
    )
    parser.add_argument(
        "--enable-correction",
        action="store_true",
        help=(
            "Enable firmware external-scan magnitude correction before each scan. "
            "Default is raw output, matching the original calibration data collection."
        ),
    )
    parser.add_argument(
        "--correction-scale-ppm",
        type=int,
        default=DEFAULT_CORRECTION_SCALE_PPM,
        help=(
            "Global correction scale sent with Kg<ppm> when --enable-correction is used. "
            f"Default: {DEFAULT_CORRECTION_SCALE_PPM}."
        ),
    )
    parser.add_argument("--operator", default="", help="Operator name for manifest.")
    parser.add_argument("--board", default="", help="Board identifier for manifest.")
    parser.add_argument(
        "--fixture-revision",
        default="initial",
        help="Fixture revision for manifest.",
    )
    parser.add_argument(
        "--physical-wiring",
        default="4-wire physical fixture",
        help="Physical wiring description for manifest.",
    )
    parser.add_argument(
        "--driver-wire-setting",
        default="existing sketch setting",
        help="Driver wire setting used during the run.",
    )
    parser.add_argument("--notes", default="", help="Run notes.")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Do not prompt before each load. Useful for parser/dry-run checks only.",
    )
    return parser.parse_args()


def build_scan_commands(args: argparse.Namespace) -> list[str]:
    commands = list(DEFAULT_SCAN_COMMANDS)
    if not args.enable_correction:
        return commands

    # Correction is runtime-only in the sketch, so send it after the runner opens
    # the serial port and before the scan setup commands.
    return ["Ke1", f"Kg{args.correction_scale_ppm}"] + commands


def load_fixture(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    loads = data.get("loads", [])
    if not isinstance(loads, list) or not loads:
        raise ValueError(f"No loads found in {path}")
    return loads


def selected_loads(loads: list[dict[str, Any]], requested: list[str] | None) -> list[dict[str, Any]]:
    if not requested:
        return loads

    by_id = {load["id"]: load for load in loads}
    missing = [load_id for load_id in requested if load_id not in by_id]
    if missing:
        known = ", ".join(sorted(by_id))
        raise ValueError(f"Unknown load ID(s): {', '.join(missing)}. Known loads: {known}")
    return [by_id[load_id] for load_id in requested]


def impedance(network: dict[str, Any], frequency_hz: float) -> complex:
    kind = network["type"]
    if kind == "resistor":
        return complex(float(network["ohm"]), 0.0)
    if kind == "capacitor":
        capacitance = float(network["farad"])
        if frequency_hz <= 0.0 or capacitance <= 0.0:
            return complex(math.inf, 0.0)
        return 1.0 / (1j * 2.0 * math.pi * frequency_hz * capacitance)
    if kind == "series":
        return sum((impedance(part, frequency_hz) for part in network["parts"]), complex(0.0, 0.0))
    if kind == "parallel":
        admittance = complex(0.0, 0.0)
        for branch in network["branches"]:
            z_branch = impedance(branch, frequency_hz)
            if z_branch == 0:
                return complex(0.0, 0.0)
            if math.isinf(abs(z_branch)):
                continue
            admittance += 1.0 / z_branch
        if admittance == 0:
            return complex(math.inf, 0.0)
        return 1.0 / admittance
    raise ValueError(f"Unsupported network type: {kind}")


def phase_deg(value: complex) -> float:
    return math.degrees(math.atan2(value.imag, value.real))


def magnitude(value: complex) -> float:
    return abs(value)


def polar_to_complex(mag: float | None, phase: float | None) -> complex | None:
    if mag is None or phase is None:
        return None
    angle = math.radians(phase)
    return complex(mag * math.cos(angle), mag * math.sin(angle))


def network_topology(network: dict[str, Any]) -> str:
    kind = network["type"]
    if kind in {"resistor", "capacitor"}:
        return kind
    child_key = "parts" if kind == "series" else "branches"
    return f"{kind}(" + ",".join(network_topology(child) for child in network[child_key]) + ")"


def compact_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def parse_float(text: str) -> float | None:
    try:
        return float(text.strip())
    except ValueError:
        return None


def parse_scan_output(text: str) -> dict[str, Any]:
    diagnostics: dict[float, dict[str, Any]] = {}
    spectrum: dict[float, dict[str, float]] = {}
    warnings: list[str] = []
    in_spectrum = False

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if "[WARN]" in line or line.startswith("WARN") or " WARN " in line:
            warnings.append(line)

        marker = "scan_external_summary,"
        marker_index = line.find(marker)
        if marker_index >= 0:
            payload = line[marker_index + len(marker) :]
            if payload.startswith("frequency_hz,"):
                continue
            parts = [part.strip() for part in payload.split(",")]
            if len(parts) < 8:
                continue
            values = [parse_float(part) for part in parts[:6]]
            current = parse_float(parts[6])
            point_count = parse_float(parts[7])
            if any(value is None for value in values) or current is None or point_count is None:
                continue
            frequency = values[0]
            assert frequency is not None
            diagnostics[frequency] = {
                "cosine_magnitude_ohm": values[1],
                "cosine_phase_deg": values[2],
                "triangular_magnitude_ohm": values[3],
                "triangular_phase_deg": values[4],
                "maxabs_ohm": values[5],
                "current_nA": int(current),
                "num_phase_points": int(point_count),
            }
            continue

        if line in {
            "frequency_hz,magnitude_ohm,phase_deg",
            "frequency_hz,magnitude_ohm,phase_deg,reliability",
        }:
            in_spectrum = True
            continue

        if in_spectrum:
            parts = [part.strip() for part in line.split(",")]
            if len(parts) not in {3, 4}:
                continue
            frequency = parse_float(parts[0])
            mag = parse_float(parts[1])
            phase = parse_float(parts[2])
            if frequency is None or mag is None or phase is None:
                continue
            spectrum[frequency] = {
                "normal_magnitude_ohm": mag,
                "normal_phase_deg": phase,
                "normal_reliability": parts[3] if len(parts) == 4 else "",
            }

    return {
        "diagnostics": diagnostics,
        "spectrum": spectrum,
        "warnings": warnings,
    }


def import_serial_module() -> Any:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for serial collection. Install it with: "
            "python3 -m pip install -r calibration/requirements.txt"
        ) from exc
    return serial


def open_serial(port: str, baud: int) -> Any:
    serial = import_serial_module()
    ser = serial.Serial(port, baudrate=baud, timeout=1.0)
    try:
        ser.dtr = True
        ser.rts = False
    except Exception:
        pass
    time.sleep(0.5)
    ser.write(b"\r\n")
    ser.flush()
    wait_for_ready(ser)
    ser.reset_input_buffer()
    return ser


def wait_for_ready(ser: Any, timeout_s: float = 15.0) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + timeout_s
    no_output_deadline = time.monotonic() + 2.5
    saw_output = False
    quiet_deadline: float | None = None

    print("Waiting for sketch startup/ready prompt...")
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            if not saw_output and time.monotonic() >= no_output_deadline:
                break
            if quiet_deadline is not None and time.monotonic() >= quiet_deadline:
                break
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        lines.append(line)
        saw_output = True
        quiet_deadline = time.monotonic() + 1.0
        if "Ready for commands" in line:
            print("Sketch is ready for commands.")
            return lines

    if saw_output:
        print("Startup output ended without an explicit ready prompt; continuing.")
    else:
        print("No startup output detected; continuing and expecting command responses.")
    return lines


def read_available_lines(ser: Any, quiet_s: float = 0.15, max_s: float = 2.0) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + max_s
    quiet_deadline = time.monotonic() + quiet_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            if time.monotonic() >= quiet_deadline:
                break
            continue
        lines.append(raw.decode("utf-8", errors="replace").rstrip("\r\n"))
        quiet_deadline = time.monotonic() + quiet_s
    return lines


def print_response_lines(lines: list[str], max_lines: int = 12) -> None:
    if not lines:
        print("     no response")
        return
    for line in lines[:max_lines]:
        if line.strip():
            print(f"     {line}")
    omitted = len(lines) - max_lines
    if omitted > 0:
        print(f"     ... {omitted} more response lines")


def run_serial_scan(
    ser: Any,
    commands: list[str],
    timeout_s: float,
    expected_spectrum_rows: int,
    progress: bool = True,
) -> str:
    ser.reset_input_buffer()
    command_lines: list[str] = []
    if progress:
        print("Sending scan commands...")
    for index, command in enumerate(commands):
        ser.write((command + "\r\n").encode("ascii"))
        ser.flush()
        if progress:
            print(f"  -> {command}")
        if index < len(commands) - 1:
            response = read_available_lines(ser, quiet_s=0.25, max_s=3.0)
            command_lines.extend(response)
            if progress:
                print_response_lines(response)
        else:
            time.sleep(0.10)

    lines: list[str] = command_lines
    deadline = time.monotonic() + timeout_s
    saw_spectrum = False
    spectrum_rows = 0
    diagnostic_rows = 0
    last_progress_second = -1
    stop_deadline: float | None = None
    start_time = time.monotonic()

    if progress:
        print(f"Waiting for scan output, timeout {timeout_s:.0f}s...")
    while time.monotonic() < deadline:
        if stop_deadline is not None and time.monotonic() >= stop_deadline:
            break

        raw = ser.readline()
        elapsed = time.monotonic() - start_time
        elapsed_second = int(elapsed)
        if progress and elapsed_second != last_progress_second and elapsed_second % 5 == 0:
            last_progress_second = elapsed_second
            print(
                f"  elapsed {elapsed_second:3d}s | diagnostics {diagnostic_rows:2d} | "
                f"spectrum {spectrum_rows:2d}/{expected_spectrum_rows}"
            )

        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        lines.append(line)

        stripped = line.strip()
        marker = "scan_external_summary,"
        marker_index = stripped.find(marker)
        if marker_index >= 0:
            payload = stripped[marker_index + len(marker) :]
            if not payload.startswith("frequency_hz,"):
                diagnostic_rows += 1
                if progress:
                    fields = [part.strip() for part in payload.split(",")]
                    frequency = fields[0] if fields else "?"
                    print(f"  diagnostic row {diagnostic_rows}: {frequency} Hz")

        if stripped in {
            "frequency_hz,magnitude_ohm,phase_deg",
            "frequency_hz,magnitude_ohm,phase_deg,reliability",
        }:
            saw_spectrum = True
            spectrum_rows = 0
            if progress:
                print("  normal spectrum table started")
            continue
        if saw_spectrum:
            parts = [part.strip() for part in stripped.split(",")]
            numeric_parts = parts[:3]
            if len(parts) in {3, 4} and all(parse_float(part) is not None for part in numeric_parts):
                spectrum_rows += 1
                if progress:
                    print(f"  spectrum row {spectrum_rows}/{expected_spectrum_rows}: {parts[0]} Hz")
                if spectrum_rows >= expected_spectrum_rows:
                    stop_deadline = time.monotonic() + 1.0
            elif stop_deadline is not None and time.monotonic() >= stop_deadline:
                break

        if stop_deadline is not None and time.monotonic() >= stop_deadline:
            break

    if progress:
        elapsed = time.monotonic() - start_time
        timed_out = time.monotonic() >= deadline and spectrum_rows < expected_spectrum_rows
        status = "timeout" if timed_out else "complete"
        print(
            f"Scan capture {status}: {elapsed:.1f}s, diagnostics {diagnostic_rows}, "
            f"spectrum {spectrum_rows}/{expected_spectrum_rows}"
        )

    return "\n".join(lines) + ("\n" if lines else "")


def git_state() -> dict[str, str]:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        status = subprocess.check_output(
            ["git", "status", "--short"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return {"commit": "", "dirty_note": "git state unavailable"}
    return {
        "commit": commit,
        "dirty_note": "dirty" if status else "clean",
    }


def timestamp_run_id() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def fsync_parent(path: Path) -> None:
    try:
        directory_fd = os.open(path.parent, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def write_text_synced(path: Path, text: str) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    fsync_parent(path)


def write_json_synced(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    fsync_parent(path)


def make_run_dirs(root: Path, run_id: str | None = None, resume: Path | None = None) -> dict[str, Path]:
    run_dir = resume if resume is not None else root / (run_id or timestamp_run_id())
    raw_dir = run_dir / "raw"
    parsed_dir = run_dir / "parsed"
    raw_dir.mkdir(parents=True, exist_ok=resume is not None)
    parsed_dir.mkdir(parents=True, exist_ok=resume is not None)
    return {"run": run_dir, "raw": raw_dir, "parsed": parsed_dir}


def expected_frequencies_from_fixture() -> list[float]:
    return [131072.0, 81920.0, 40960.0, 18204.0, 8192.0, 4096.0, 2048.0, 1024.0]


def build_measurement_rows(
    run_id: str,
    load: dict[str, Any],
    parsed: dict[str, Any],
    raw_log_file: str,
    notes: str,
) -> list[dict[str, Any]]:
    diagnostics = parsed["diagnostics"]
    spectrum = parsed["spectrum"]
    frequencies = sorted(set(diagnostics) | set(spectrum), reverse=True)
    if not frequencies:
        frequencies = expected_frequencies_from_fixture()

    rows: list[dict[str, Any]] = []
    warning_text = " | ".join(parsed["warnings"])
    network = load["network"]

    for frequency in frequencies:
        expected = impedance(network, frequency)
        expected_mag = magnitude(expected)
        expected_phase = phase_deg(expected)
        diag = diagnostics.get(frequency, {})
        normal = spectrum.get(frequency, {})
        cosine = polar_to_complex(
            diag.get("cosine_magnitude_ohm"),
            diag.get("cosine_phase_deg"),
        )
        cosine_mag = diag.get("cosine_magnitude_ohm")
        cosine_phase = diag.get("cosine_phase_deg")
        scale = ""
        mag_error_pct = ""
        phase_error = ""
        if cosine_mag not in (None, 0):
            scale = expected_mag / cosine_mag
            if expected_mag != 0:
                mag_error_pct = 100.0 * (cosine_mag - expected_mag) / expected_mag
        if cosine_phase is not None:
            phase_error = cosine_phase - expected_phase

        row = {
            "run_id": run_id,
            "load_id": load["id"],
            "load_description": load.get("description", ""),
            "fit_hint": load.get("fit_hint", ""),
            "topology": network_topology(network),
            "component_values": compact_json(network),
            "frequency_hz": frequency,
            "expected_real_ohm": expected.real,
            "expected_imag_ohm": expected.imag,
            "expected_magnitude_ohm": expected_mag,
            "expected_phase_deg": expected_phase,
            "cosine_magnitude_ohm": cosine_mag if cosine_mag is not None else "",
            "cosine_phase_deg": cosine_phase if cosine_phase is not None else "",
            "cosine_real_ohm": cosine.real if cosine is not None else "",
            "cosine_imag_ohm": cosine.imag if cosine is not None else "",
            "triangular_magnitude_ohm": diag.get("triangular_magnitude_ohm", ""),
            "triangular_phase_deg": diag.get("triangular_phase_deg", ""),
            "maxabs_ohm": diag.get("maxabs_ohm", ""),
            "normal_magnitude_ohm": normal.get("normal_magnitude_ohm", ""),
            "normal_phase_deg": normal.get("normal_phase_deg", ""),
            "normal_reliability": normal.get("normal_reliability", ""),
            "current_nA": diag.get("current_nA", ""),
            "num_phase_points": diag.get("num_phase_points", ""),
            "scale_expected_over_cosine": scale,
            "cosine_magnitude_error_pct": mag_error_pct,
            "cosine_phase_error_deg": phase_error,
            "warnings": warning_text,
            "raw_log_file": raw_log_file,
            "notes": notes,
        }
        row.update(DEFAULT_SCAN_META)
        rows.append(row)
    return rows


def write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
        handle.flush()
        os.fsync(handle.fileno())
    fsync_parent(path)


def build_fit_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[float, list[float]] = {}
    for row in rows:
        if row["fit_hint"] not in {"calibration", "tissue_model"}:
            continue
        scale = row.get("scale_expected_over_cosine")
        if scale == "" or scale is None:
            continue
        grouped.setdefault(float(row["frequency_hz"]), []).append(float(scale))

    summary: list[dict[str, Any]] = []
    all_scales: list[float] = []
    for frequency in sorted(grouped, reverse=True):
        scales = grouped[frequency]
        all_scales.extend(scales)
        summary.append(
            {
                "group": "frequency",
                "frequency_hz": frequency,
                "count": len(scales),
                "scale_mean": statistics.fmean(scales),
                "scale_median": statistics.median(scales),
                "scale_stdev": statistics.stdev(scales) if len(scales) > 1 else 0.0,
            }
        )

    if all_scales:
        summary.insert(
            0,
            {
                "group": "global",
                "frequency_hz": "",
                "count": len(all_scales),
                "scale_mean": statistics.fmean(all_scales),
                "scale_median": statistics.median(all_scales),
                "scale_stdev": statistics.stdev(all_scales) if len(all_scales) > 1 else 0.0,
            },
        )
    return summary


def write_manifest(
    path: Path,
    args: argparse.Namespace,
    run_id: str,
    loads: list[dict[str, Any]],
) -> None:
    state = git_state()
    existing: dict[str, Any] = {}
    if path.exists():
        with path.open("r", encoding="utf-8") as handle:
            existing = json.load(handle)

    manifest = {
        **existing,
        "run_id": run_id,
        "date_time": existing.get("date_time", dt.datetime.now().isoformat(timespec="seconds")),
        "last_updated": dt.datetime.now().isoformat(timespec="seconds"),
        "operator": args.operator,
        "board": args.board,
        "fixture_revision": args.fixture_revision,
        "physical_wiring": args.physical_wiring,
        "driver_wire_setting": args.driver_wire_setting,
        "sketch": "examples/MAX30001G/MAX30001G.ino",
        "git_commit": state["commit"],
        "git_dirty_note": state["dirty_note"],
        "scan_commands": build_scan_commands(args),
        "scan_meta": DEFAULT_SCAN_META,
        "correction_enabled": args.enable_correction,
        "correction_scale_ppm": args.correction_scale_ppm if args.enable_correction else "",
        "correction_scale": (
            args.correction_scale_ppm / 1_000_000.0 if args.enable_correction else ""
        ),
        "loads": [
            {
                "id": load["id"],
                "description": load.get("description", ""),
                "fit_hint": load.get("fit_hint", ""),
                "network": load["network"],
            }
            for load in loads
        ],
        "notes": args.notes,
    }
    write_json_synced(path, manifest)


def raw_log_is_complete(path: Path) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        return False
    parsed = parse_scan_output(path.read_text(encoding="utf-8", errors="replace"))
    return bool(parsed["diagnostics"] or parsed["spectrum"])


def write_parsed_outputs(dirs: dict[str, Path], all_rows: list[dict[str, Any]]) -> None:
    measurements_path = dirs["parsed"] / "measurements.csv"
    write_csv(measurements_path, MEASUREMENT_FIELDS, all_rows)
    summary_rows = build_fit_summary(all_rows)
    write_csv(
        dirs["parsed"] / "fit_summary.csv",
        ["group", "frequency_hz", "count", "scale_mean", "scale_median", "scale_stdev"],
        summary_rows,
    )


def main() -> int:
    args = parse_args()
    fixture_loads = load_fixture(args.loads_file)
    loads = selected_loads(fixture_loads, args.load_ids)
    scan_commands = build_scan_commands(args)
    requested_load_ids = {load["id"] for load in loads}
    run_id = args.resume.name if args.resume is not None else timestamp_run_id()
    dirs = make_run_dirs(args.out, run_id, args.resume)
    manifest_loads = fixture_loads if args.resume is not None else loads
    write_manifest(dirs["run"] / "manifest.json", args, run_id, manifest_loads)

    serial_handle = None
    if not args.dry_run:
        serial_handle = open_serial(args.port, args.baud)

    all_rows: list[dict[str, Any]] = []
    completed_loads: set[str] = set()
    loads_to_rebuild = fixture_loads if args.resume is not None else loads
    for load in loads_to_rebuild:
        raw_log_file = f"raw/{load['id']}.txt"
        raw_path = dirs["run"] / raw_log_file
        force_this_load = args.force_rerun and load["id"] in requested_load_ids
        if args.resume is None or force_this_load or not raw_log_is_complete(raw_path):
            continue
        parsed = parse_scan_output(raw_path.read_text(encoding="utf-8", errors="replace"))
        all_rows.extend(
            build_measurement_rows(
                run_id=run_id,
                load=load,
                parsed=parsed,
                raw_log_file=raw_log_file,
                notes=args.notes,
            )
        )
        completed_loads.add(load["id"])

    if completed_loads:
        print(f"Resuming {dirs['run']}; already complete: {', '.join(sorted(completed_loads))}")
        write_parsed_outputs(dirs, all_rows)

    try:
        for load in loads:
            if load["id"] in completed_loads:
                continue

            print(f"\nLoad {load['id']}: {load.get('description', '')}")
            if not args.yes:
                input("Set the fixture to this load, then press Enter to scan.")

            raw_log_file = f"raw/{load['id']}.txt"
            raw_path = dirs["run"] / raw_log_file

            if args.dry_run:
                raw_text = ""
                parsed = {"diagnostics": {}, "spectrum": {}, "warnings": []}
            else:
                assert serial_handle is not None
                raw_text = run_serial_scan(
                    serial_handle,
                    scan_commands,
                    args.timeout,
                    args.expected_spectrum_rows,
                )
                parsed = parse_scan_output(raw_text)

            write_text_synced(raw_path, raw_text)
            load_rows = build_measurement_rows(
                run_id=run_id,
                load=load,
                parsed=parsed,
                raw_log_file=raw_log_file,
                notes=args.notes,
            )
            all_rows.extend(load_rows)
            write_parsed_outputs(dirs, all_rows)
            parsed_count = sum(1 for row in load_rows if row["cosine_magnitude_ohm"] != "")
            print(f"Captured {parsed_count} measured frequency rows for {load['id']}.")
    except KeyboardInterrupt:
        print("\nInterrupted. Data completed before the interrupt has been flushed to disk.")
        return 130
    finally:
        if serial_handle is not None:
            serial_handle.close()

    measurements_path = dirs["parsed"] / "measurements.csv"
    write_parsed_outputs(dirs, all_rows)

    print(f"\nWrote {dirs['run']}")
    print(f"Measurements: {measurements_path}")
    print(f"Fit summary rows: {len(build_fit_summary(all_rows))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
