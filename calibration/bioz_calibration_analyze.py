#!/usr/bin/env python3
"""Analyze MAX30001G external BIOZ calibration measurements."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
from pathlib import Path
import statistics
from typing import Any, Callable


FIT_HINTS_DEFAULT = {"calibration", "tissue_model"}
MODEL_NAMES = [
    "uncalibrated",
    "global_ls",
    "global_median",
    "frequency_ls",
    "frequency_median",
    "current_ls",
    "current_median",
]

USABILITY_FIELDS = [
    "run_id",
    "load_id",
    "load_description",
    "fit_hint",
    "frequency_hz",
    "expected_magnitude_ohm",
    "expected_phase_deg",
    "cosine_magnitude_ohm",
    "cosine_phase_deg",
    "current_nA",
    "fit_eligible",
    "fit_exclusion_reason",
    "global_median_error_pct",
    "phase_error_deg",
    "usability_class",
    "usability_score",
    "usability_reasons",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze BIOZ calibration runs and compare correction models."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        type=Path,
        help="Calibration run directories containing parsed/measurements.csv.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output analysis directory. Defaults to calibration_analysis/YYYYMMDD_HHMMSS.",
    )
    parser.add_argument(
        "--fit-hint",
        action="append",
        dest="fit_hints",
        help="fit_hint values allowed for fitting. May be repeated.",
    )
    parser.add_argument(
        "--min-fit-ohm",
        type=float,
        default=20.0,
        help="Minimum expected magnitude for default fit eligibility.",
    )
    parser.add_argument(
        "--max-fit-ohm",
        type=float,
        default=50000.0,
        help="Maximum expected magnitude for default fit eligibility.",
    )
    parser.add_argument(
        "--max-fit-phase-error-deg",
        type=float,
        default=10.0,
        help="Maximum absolute phase error for default fit eligibility.",
    )
    parser.add_argument(
        "--exclude-warnings",
        action="store_true",
        help="Exclude rows with warnings from fitting.",
    )
    parser.add_argument(
        "--residual-limit-pct",
        type=float,
        default=10.0,
        help="Residual threshold for exclusion candidate reporting.",
    )
    parser.add_argument(
        "--phase-limit-deg",
        type=float,
        default=10.0,
        help="Phase-error threshold for exclusion candidate reporting.",
    )
    parser.add_argument(
        "--model-fit-source",
        choices=["fit_eligible", "validated_candidate", "validated_or_boundary"],
        default="fit_eligible",
        help=(
            "Rows used to fit the reported correction models. "
            "fit_eligible is the first-pass default. validated_candidate and "
            "validated_or_boundary use provisional usability classes computed "
            "from the first-pass global median model."
        ),
    )
    return parser.parse_args()


def output_dir(path: Path | None) -> Path:
    if path is not None:
        return path
    return Path("calibration_analysis") / dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def read_measurements(run_dirs: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for run_dir in run_dirs:
        path = run_dir / "parsed" / "measurements.csv"
        if not path.exists():
            raise FileNotFoundError(f"Missing {path}")
        with path.open("r", encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                row = dict(row)
                row["source_run_dir"] = str(run_dir)
                rows.append(row)
    return rows


def f(row: dict[str, Any], name: str, default: float = math.nan) -> float:
    value = row.get(name, "")
    if value == "" or value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def s(row: dict[str, Any], name: str) -> str:
    return str(row.get(name, ""))


def row_key(row: dict[str, Any]) -> tuple[str, str, float]:
    return (s(row, "run_id"), s(row, "load_id"), f(row, "frequency_hz"))


def ratio(row: dict[str, Any]) -> float:
    measured = f(row, "cosine_magnitude_ohm")
    expected = f(row, "expected_magnitude_ohm")
    if measured == 0.0 or math.isnan(measured) or math.isnan(expected):
        return math.nan
    return expected / measured


def pct_error(measured: float, expected: float) -> float:
    if expected == 0.0 or math.isnan(measured) or math.isnan(expected):
        return math.nan
    return 100.0 * (measured - expected) / expected


def fit_eligible_reason(
    row: dict[str, Any],
    fit_hints: set[str],
    min_fit_ohm: float,
    max_fit_ohm: float,
    max_fit_phase_error_deg: float,
    exclude_warnings: bool,
) -> str:
    if s(row, "fit_hint") not in fit_hints:
        return "fit_hint"
    expected = f(row, "expected_magnitude_ohm")
    measured = f(row, "cosine_magnitude_ohm")
    phase_error = abs(f(row, "cosine_phase_error_deg"))
    if math.isnan(expected) or math.isnan(measured) or measured <= 0.0:
        return "missing_measurement"
    if expected < min_fit_ohm:
        return "below_min_fit_ohm"
    if expected > max_fit_ohm:
        return "above_max_fit_ohm"
    if phase_error > max_fit_phase_error_deg:
        return "phase_error"
    if exclude_warnings and s(row, "warnings"):
        return "warning"
    return ""


def scale_ls(rows: list[dict[str, Any]]) -> float:
    numerator = 0.0
    denominator = 0.0
    for row in rows:
        measured = f(row, "cosine_magnitude_ohm")
        expected = f(row, "expected_magnitude_ohm")
        if measured > 0.0 and not math.isnan(expected):
            numerator += measured * expected
            denominator += measured * measured
    return numerator / denominator if denominator else math.nan


def scale_median(rows: list[dict[str, Any]]) -> float:
    ratios = [ratio(row) for row in rows]
    ratios = [value for value in ratios if not math.isnan(value)]
    return statistics.median(ratios) if ratios else math.nan


def grouped_scale(
    rows: list[dict[str, Any]],
    group_fn: Callable[[dict[str, Any]], Any],
    scale_fn: Callable[[list[dict[str, Any]]], float],
) -> dict[Any, float]:
    groups: dict[Any, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(group_fn(row), []).append(row)
    return {key: scale_fn(group_rows) for key, group_rows in groups.items()}


def build_models(fit_rows: list[dict[str, Any]]) -> dict[str, Any]:
    global_ls = scale_ls(fit_rows)
    global_median = scale_median(fit_rows)
    return {
        "uncalibrated": 1.0,
        "global_ls": global_ls,
        "global_median": global_median,
        "frequency_ls": grouped_scale(fit_rows, lambda row: f(row, "frequency_hz"), scale_ls),
        "frequency_median": grouped_scale(fit_rows, lambda row: f(row, "frequency_hz"), scale_median),
        "current_ls": grouped_scale(fit_rows, lambda row: s(row, "current_nA"), scale_ls),
        "current_median": grouped_scale(fit_rows, lambda row: s(row, "current_nA"), scale_median),
    }


def model_scale(model_name: str, models: dict[str, Any], row: dict[str, Any]) -> float:
    model = models[model_name]
    if isinstance(model, float):
        return model
    if model_name.startswith("frequency_"):
        key = f(row, "frequency_hz")
    elif model_name.startswith("current_"):
        key = s(row, "current_nA")
    else:
        key = None
    value = model.get(key, math.nan)
    if math.isnan(value):
        return models["global_median"]
    return value


def residual_for(model_name: str, models: dict[str, Any], row: dict[str, Any]) -> tuple[float, float, float]:
    scale = model_scale(model_name, models, row)
    measured = f(row, "cosine_magnitude_ohm")
    expected = f(row, "expected_magnitude_ohm")
    calibrated = measured * scale
    return scale, calibrated, pct_error(calibrated, expected)


def metric_summary(rows: list[dict[str, Any]], model_name: str, models: dict[str, Any]) -> dict[str, float]:
    errors = []
    phase_errors = []
    for row in rows:
        _, _, err = residual_for(model_name, models, row)
        if not math.isnan(err):
            errors.append(err)
        phase = f(row, "cosine_phase_error_deg")
        if not math.isnan(phase):
            phase_errors.append(phase)
    abs_errors = [abs(value) for value in errors]
    abs_phase = [abs(value) for value in phase_errors]
    return {
        "count": float(len(errors)),
        "mean_error_pct": statistics.fmean(errors) if errors else math.nan,
        "mean_abs_error_pct": statistics.fmean(abs_errors) if abs_errors else math.nan,
        "median_abs_error_pct": statistics.median(abs_errors) if abs_errors else math.nan,
        "max_abs_error_pct": max(abs_errors) if abs_errors else math.nan,
        "mean_abs_phase_error_deg": statistics.fmean(abs_phase) if abs_phase else math.nan,
        "max_abs_phase_error_deg": max(abs_phase) if abs_phase else math.nan,
    }


def leave_one_load_out(rows: list[dict[str, Any]], model_name: str) -> dict[str, float]:
    loads = sorted({s(row, "load_id") for row in rows})
    errors = []
    for load in loads:
        train = [row for row in rows if s(row, "load_id") != load]
        test = [row for row in rows if s(row, "load_id") == load]
        if not train or not test:
            continue
        models = build_models(train)
        for row in test:
            _, _, err = residual_for(model_name, models, row)
            if not math.isnan(err):
                errors.append(abs(err))
    return {
        "cv_count": float(len(errors)),
        "cv_mean_abs_error_pct": statistics.fmean(errors) if errors else math.nan,
        "cv_median_abs_error_pct": statistics.median(errors) if errors else math.nan,
        "cv_max_abs_error_pct": max(errors) if errors else math.nan,
    }


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def format_float(value: Any, digits: int = 6) -> Any:
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.{digits}g}"
    return value


def write_summary_text(
    path: Path,
    rows: list[dict[str, Any]],
    model_rows: list[dict[str, Any]],
    model_fit_source: str,
    model_fit_count: int,
    fallback_note: str,
) -> None:
    best = min(
        (row for row in model_rows if row["eligible_mean_abs_error_pct"] != ""),
        key=lambda row: float(row["eligible_mean_abs_error_pct"]),
    )
    lines = [
        "BIOZ Calibration Analysis Summary",
        "",
        f"Input rows: {len(rows)}",
        f"Loads: {len({s(row, 'load_id') for row in rows})}",
        f"Model fit source: {model_fit_source}",
        f"Model fit rows: {model_fit_count}",
        f"Best eligible model by mean absolute magnitude residual: {best['model']}",
        f"Eligible mean abs residual: {best['eligible_mean_abs_error_pct']} %",
        f"Eligible median abs residual: {best['eligible_median_abs_error_pct']} %",
        "",
        "Interpretation guidance:",
        "- Prefer global_median if its residuals are close to frequency/current models.",
        "- Prefer frequency_median if high and low frequencies show repeatable scale differences.",
        "- Prefer current_median only if residuals group by current after frequency effects are removed.",
        "- Do not use complex correction unless phase residuals remain systematic after exclusions.",
        "",
    ]
    if fallback_note:
        lines.extend(["Fallback:", fallback_note, ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def usability_class(
    row: dict[str, Any],
    global_median_error_pct: float,
    phase_error_deg: float,
    min_fit_ohm: float,
    max_fit_ohm: float,
) -> tuple[str, int, list[str]]:
    expected = f(row, "expected_magnitude_ohm")
    fit_hint = s(row, "fit_hint")
    abs_mag = abs(global_median_error_pct)
    abs_phase = abs(phase_error_deg)
    reasons: list[str] = []

    if expected < min_fit_ohm:
        reasons.append("below_validated_magnitude_floor")
    if expected > max_fit_ohm:
        reasons.append("above_validated_magnitude_ceiling")
    if fit_hint == "parasitic_check":
        reasons.append("parasitic_boundary_load")
    if abs_phase > 10.0:
        reasons.append("phase_error_gt_10deg")
    if abs_mag > 10.0:
        reasons.append("global_magnitude_residual_gt_10pct")

    if abs_mag <= 5.0 and abs_phase <= 5.0 and min_fit_ohm <= expected <= max_fit_ohm and fit_hint != "parasitic_check":
        return "validated_candidate", 3, reasons
    if abs_mag <= 10.0 and abs_phase <= 10.0:
        return "boundary_candidate", 2, reasons
    if abs_mag <= 20.0 or abs_phase <= 20.0:
        return "questionable", 1, reasons
    return "unreliable", 0, reasons


def build_load_summary(usability_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in usability_rows:
        grouped.setdefault(str(row["load_id"]), []).append(row)

    output: list[dict[str, Any]] = []
    for load_id in sorted(grouped):
        rows = grouped[load_id]
        counts = {"validated_candidate": 0, "boundary_candidate": 0, "questionable": 0, "unreliable": 0}
        fit_count = 0
        for row in rows:
            counts[str(row["usability_class"])] += 1
            if str(row["fit_eligible"]) == "1":
                fit_count += 1
        validated_freqs = [
            str(row["frequency_hz"])
            for row in rows
            if str(row["usability_class"]) == "validated_candidate"
        ]
        output.append(
            {
                "load_id": load_id,
                "load_description": rows[0]["load_description"],
                "fit_hint": rows[0]["fit_hint"],
                "rows": len(rows),
                "fit_eligible_rows": fit_count,
                "validated_candidate_rows": counts["validated_candidate"],
                "boundary_candidate_rows": counts["boundary_candidate"],
                "questionable_rows": counts["questionable"],
                "unreliable_rows": counts["unreliable"],
                "validated_candidate_frequencies_hz": "|".join(validated_freqs),
            }
        )
    return output


def build_analysis_tables(
    rows: list[dict[str, Any]],
    models: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    residual_rows: list[dict[str, Any]] = []
    exclusion_rows: list[dict[str, Any]] = []
    usability_rows: list[dict[str, Any]] = []

    for row in rows:
        out = {
            "source_run_dir": s(row, "source_run_dir"),
            "run_id": s(row, "run_id"),
            "load_id": s(row, "load_id"),
            "load_description": s(row, "load_description"),
            "fit_hint": s(row, "fit_hint"),
            "frequency_hz": f(row, "frequency_hz"),
            "current_nA": s(row, "current_nA"),
            "expected_magnitude_ohm": f(row, "expected_magnitude_ohm"),
            "expected_phase_deg": f(row, "expected_phase_deg"),
            "cosine_magnitude_ohm": f(row, "cosine_magnitude_ohm"),
            "cosine_phase_deg": f(row, "cosine_phase_deg"),
            "cosine_phase_error_deg": f(row, "cosine_phase_error_deg"),
            "fit_eligible": row["fit_eligible"],
            "fit_exclusion_reason": row["fit_exclusion_reason"],
        }
        for model_name in MODEL_NAMES:
            scale, calibrated, err = residual_for(model_name, models, row)
            out[f"{model_name}_scale"] = scale
            out[f"{model_name}_magnitude_ohm"] = calibrated
            out[f"{model_name}_error_pct"] = err
        residual_rows.append({key: format_float(value) for key, value in out.items()})

        raw_global_error = out["global_median_error_pct"]
        global_error_value = raw_global_error if isinstance(raw_global_error, float) else float(raw_global_error)
        phase_error_value = f(row, "cosine_phase_error_deg")
        phase_err_abs = abs(phase_error_value)

        review_reasons = []
        if row["fit_exclusion_reason"]:
            review_reasons.append(row["fit_exclusion_reason"])
        if not math.isnan(global_error_value) and abs(global_error_value) > args.residual_limit_pct:
            review_reasons.append("global_residual")
        if not math.isnan(phase_err_abs) and phase_err_abs > args.phase_limit_deg:
            review_reasons.append("phase_residual")
        if review_reasons:
            exclusion_rows.append(
                {
                    "run_id": s(row, "run_id"),
                    "load_id": s(row, "load_id"),
                    "frequency_hz": f"{f(row, 'frequency_hz'):.1f}",
                    "expected_magnitude_ohm": f"{f(row, 'expected_magnitude_ohm'):.6g}",
                    "cosine_magnitude_ohm": f"{f(row, 'cosine_magnitude_ohm'):.6g}",
                    "global_median_error_pct": f"{global_error_value:.6g}",
                    "cosine_phase_error_deg": f"{phase_error_value:.6g}",
                    "reasons": "|".join(dict.fromkeys(review_reasons)),
                }
            )

        usability, score, usability_reasons = usability_class(
            row,
            global_median_error_pct=global_error_value,
            phase_error_deg=phase_error_value,
            min_fit_ohm=args.min_fit_ohm,
            max_fit_ohm=args.max_fit_ohm,
        )
        row["_usability_class"] = usability
        row["_usability_score"] = str(score)
        usability_rows.append(
            {
                "run_id": s(row, "run_id"),
                "load_id": s(row, "load_id"),
                "load_description": s(row, "load_description"),
                "fit_hint": s(row, "fit_hint"),
                "frequency_hz": format_float(f(row, "frequency_hz")),
                "expected_magnitude_ohm": format_float(f(row, "expected_magnitude_ohm")),
                "expected_phase_deg": format_float(f(row, "expected_phase_deg")),
                "cosine_magnitude_ohm": format_float(f(row, "cosine_magnitude_ohm")),
                "cosine_phase_deg": format_float(f(row, "cosine_phase_deg")),
                "current_nA": s(row, "current_nA"),
                "fit_eligible": row["fit_eligible"],
                "fit_exclusion_reason": row["fit_exclusion_reason"],
                "global_median_error_pct": format_float(global_error_value),
                "phase_error_deg": format_float(phase_error_value),
                "usability_class": usability,
                "usability_score": score,
                "usability_reasons": "|".join(usability_reasons),
            }
        )

    return residual_rows, exclusion_rows, usability_rows


def select_model_fit_rows(
    rows: list[dict[str, Any]],
    source: str,
) -> tuple[list[dict[str, Any]], str]:
    if source == "fit_eligible":
        return [row for row in rows if row["fit_eligible"] == "1"], ""
    if source == "validated_candidate":
        selected = [row for row in rows if row.get("_usability_class") == "validated_candidate"]
    elif source == "validated_or_boundary":
        selected = [
            row
            for row in rows
            if row.get("_usability_class") in {"validated_candidate", "boundary_candidate"}
        ]
    else:
        selected = []

    if selected:
        return selected, ""

    fallback = [row for row in rows if row["fit_eligible"] == "1"]
    return fallback, (
        f"No rows matched model fit source '{source}'. "
        "Fell back to first-pass fit_eligible rows."
    )


def main() -> int:
    args = parse_args()
    out_dir = output_dir(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    fit_hints = set(args.fit_hints or FIT_HINTS_DEFAULT)

    rows = read_measurements(args.runs)
    for row in rows:
        reason = fit_eligible_reason(
            row,
            fit_hints=fit_hints,
            min_fit_ohm=args.min_fit_ohm,
            max_fit_ohm=args.max_fit_ohm,
            max_fit_phase_error_deg=args.max_fit_phase_error_deg,
            exclude_warnings=args.exclude_warnings,
        )
        row["fit_eligible"] = "1" if reason == "" else "0"
        row["fit_exclusion_reason"] = reason

    first_pass_fit_rows = [row for row in rows if row["fit_eligible"] == "1"]
    first_pass_models = build_models(first_pass_fit_rows)
    build_analysis_tables(rows, first_pass_models, args)

    fit_rows, fallback_note = select_model_fit_rows(rows, args.model_fit_source)
    models = build_models(fit_rows)
    residual_rows, exclusion_rows, usability_rows = build_analysis_tables(rows, models, args)

    model_rows: list[dict[str, Any]] = []
    all_rows = rows
    eligible_rows = fit_rows
    for model_name in MODEL_NAMES:
        all_summary = metric_summary(all_rows, model_name, models)
        eligible_summary = metric_summary(eligible_rows, model_name, models)
        cv_summary = leave_one_load_out(eligible_rows, model_name)
        row = {
            "model": model_name,
            "fit_count": len(fit_rows),
            "model_fit_source": args.model_fit_source if not fallback_note else "fit_eligible_fallback",
            "all_count": int(all_summary["count"]),
            "all_mean_abs_error_pct": all_summary["mean_abs_error_pct"],
            "all_median_abs_error_pct": all_summary["median_abs_error_pct"],
            "all_max_abs_error_pct": all_summary["max_abs_error_pct"],
            "eligible_count": int(eligible_summary["count"]),
            "eligible_mean_abs_error_pct": eligible_summary["mean_abs_error_pct"],
            "eligible_median_abs_error_pct": eligible_summary["median_abs_error_pct"],
            "eligible_max_abs_error_pct": eligible_summary["max_abs_error_pct"],
            "eligible_mean_abs_phase_error_deg": eligible_summary["mean_abs_phase_error_deg"],
            "cv_mean_abs_error_pct": cv_summary["cv_mean_abs_error_pct"],
            "cv_median_abs_error_pct": cv_summary["cv_median_abs_error_pct"],
            "cv_max_abs_error_pct": cv_summary["cv_max_abs_error_pct"],
        }
        model_rows.append({key: format_float(value) for key, value in row.items()})

    scale_frequency_rows = []
    for frequency in sorted(models["frequency_median"], reverse=True):
        freq_rows = [row for row in fit_rows if f(row, "frequency_hz") == frequency]
        scale_frequency_rows.append(
            {
                "frequency_hz": f"{frequency:.1f}",
                "fit_count": len(freq_rows),
                "scale_ls": format_float(models["frequency_ls"][frequency]),
                "scale_median": format_float(models["frequency_median"][frequency]),
                "ratio_min": format_float(min(ratio(row) for row in freq_rows)),
                "ratio_max": format_float(max(ratio(row) for row in freq_rows)),
            }
        )

    scale_current_rows = []
    for current in sorted(models["current_median"], key=lambda value: float(value or 0.0)):
        current_rows = [row for row in fit_rows if s(row, "current_nA") == current]
        scale_current_rows.append(
            {
                "current_nA": current,
                "fit_count": len(current_rows),
                "scale_ls": format_float(models["current_ls"][current]),
                "scale_median": format_float(models["current_median"][current]),
                "ratio_min": format_float(min(ratio(row) for row in current_rows)),
                "ratio_max": format_float(max(ratio(row) for row in current_rows)),
            }
        )

    residual_fields = list(residual_rows[0].keys()) if residual_rows else []
    write_csv(out_dir / "residuals.csv", residual_rows, residual_fields)
    write_csv(out_dir / "model_summary.csv", model_rows, list(model_rows[0].keys()))
    write_csv(out_dir / "scale_by_frequency.csv", scale_frequency_rows, list(scale_frequency_rows[0].keys()))
    write_csv(out_dir / "scale_by_current.csv", scale_current_rows, list(scale_current_rows[0].keys()))
    write_csv(
        out_dir / "exclusion_candidates.csv",
        exclusion_rows,
        [
            "run_id",
            "load_id",
            "frequency_hz",
            "expected_magnitude_ohm",
            "cosine_magnitude_ohm",
            "global_median_error_pct",
            "cosine_phase_error_deg",
            "reasons",
        ],
    )
    write_csv(out_dir / "usable_region_review.csv", usability_rows, USABILITY_FIELDS)
    load_summary_rows = build_load_summary(usability_rows)
    write_csv(
        out_dir / "usable_region_by_load.csv",
        load_summary_rows,
        [
            "load_id",
            "load_description",
            "fit_hint",
            "rows",
            "fit_eligible_rows",
            "validated_candidate_rows",
            "boundary_candidate_rows",
            "questionable_rows",
            "unreliable_rows",
            "validated_candidate_frequencies_hz",
        ],
    )
    write_summary_text(
        out_dir / "summary.txt",
        rows,
        model_rows,
        model_fit_source=args.model_fit_source if not fallback_note else "fit_eligible_fallback",
        model_fit_count=len(fit_rows),
        fallback_note=fallback_note,
    )

    print(f"Wrote {out_dir}")
    print(f"Rows: {len(rows)} | model fit rows: {len(fit_rows)} | model fit source: {args.model_fit_source if not fallback_note else 'fit_eligible_fallback'}")
    if fallback_note:
        print(fallback_note)
    print(f"Outputs: model_summary.csv, residuals.csv, scale_by_frequency.csv, scale_by_current.csv, exclusion_candidates.csv, usable_region_review.csv, usable_region_by_load.csv, summary.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
