# External BIOZ Calibration

This folder contains the host-side tools for collecting calibration data from the
external BIOZ impedance fixture.

The first workflow is semi-automated: the fixture is still switched manually, but
the Python program sends the scan setup commands, captures the serial output,
parses the scan tables, computes theoretical impedance for the selected load,
and writes raw and parsed files into a timestamped run folder.

## Setup

Install the Python serial dependency:

```bash
python3 -m pip install -r calibration/requirements.txt
```

Upload `examples/MAX30001G/MAX30001G.ino` to the board and close Arduino IDE
Serial Monitor before running the calibration script.

On connection, the runner sends a blank line to release the sketch startup
prompt, waits for `Ready for commands` when the board resets, then sends the scan
commands. This avoids losing commands during the sketch startup input flush.

The script expects the sketch scan output cleanup currently used by
`MAX30001G.ino`:

- ordinary status lines may be logger-prefixed, for example `[INFO] ...`,
- diagnostic scan rows are logged as `scan_external_summary,...`,
- the normal spectrum is printed as raw CSV:

```text
frequency_hz,magnitude_ohm,phase_deg
...
```

## Dry Run

Use dry-run mode to verify the fixture manifest and theoretical impedance model
without opening a serial port:

```bash
python3 calibration/bioz_calibration_runner.py --dry-run --load R100 --load X1
```

## Calibration Run

Run all fixture loads:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0
```

Run selected loads:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0 --load R100 --load R1K --load R10K --load P2 --load S3 --load X1 --load X2
```

Raw calibration is the default. To verify corrected firmware output in the same
serial session used for scanning, enable correction explicitly:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0 --enable-correction --load R100 --load R1K --load R10K
```

Resume an interrupted run or add more loads to an existing run:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/YYYYMMDD_HHMMSS --load R330 --load P1 --load S1
```

The runner skips completed loads when `raw/<load_id>.txt` already contains a
parseable scan. Use `--force-rerun` to overwrite a completed load.

For each load, set the fixture to the requested setting and press Enter. The
script sends:

```text
Ke1          only when --enable-correction is used
Kg1250650    only when --enable-correction is used, or Kg<--correction-scale-ppm>
<
m8
Si0
Sa8
Sf1
Sr0
Sp1
St48
Sc64
Bc8000
a
.
```

During each scan the script prints progress every few seconds and reports each
diagnostic/spectrum row as it arrives. This is only operator feedback; the raw
serial log is still saved unchanged.

Outputs are written to:

```text
calibration_runs/
  YYYYMMDD_HHMMSS/
    manifest.json
    raw/
      R100.txt
      ...
    parsed/
      measurements.csv
      fit_summary.csv
```

`measurements.csv` is the main dataset for deciding whether a global scale,
per-frequency scale, or more complex calibration model is justified.

The runner flushes and fsyncs raw logs and parsed CSV files after each completed
load so an interrupted run can be continued with `--resume`.

## Calibration Analysis

Analyze the full fixture run:

```bash
python3 calibration/bioz_calibration_analyze.py --out calibration/calibration_analysis/YYYYMMDD_full calibration/calibration_runs/YYYYMMDD_HHMMSS
```

After reviewing the provisional usable region, rerun model fitting with only
validated candidate rows:

```bash
python3 calibration/bioz_calibration_analyze.py --model-fit-source validated_candidate --out calibration/calibration_analysis/YYYYMMDD_validated calibration/calibration_runs/YYYYMMDD_HHMMSS
```

Analyze the full fixture run together with a repeatability run:

```bash
python3 calibration/bioz_calibration_analyze.py --out calibration/calibration_analysis/YYYYMMDD_combined calibration/calibration_runs/FULL_RUN calibration/calibration_runs/REPEAT_RUN
```

The analyzer does not require pre-existing validation candidate columns in
`measurements.csv`. It computes provisional usability classes internally from a
first-pass fit. If no rows match the requested `--model-fit-source`, it falls
back to the first-pass `fit_eligible` rows and reports the fallback in
`summary.txt`.

Outputs:

```text
model_summary.csv
residuals.csv
scale_by_frequency.csv
scale_by_current.csv
exclusion_candidates.csv
usable_region_review.csv
usable_region_by_load.csv
summary.txt
```

Use the full fixture run for model selection. Use the repeatability run to
confirm that residuals are systematic rather than random drift.

Start human review with `usable_region_by_load.csv`, then inspect individual
points in `usable_region_review.csv`. `residuals.csv` is the detailed table for
model comparison and rerunning fits.

## Current Decision

Use global median magnitude scale as the first firmware correction model:

```text
Z_corrected_mag = K_global_median * Z_cosine_mag
phase_corrected_deg = Z_cosine_phase_deg
```

`current_median` gives slightly lower in-sample residuals, but the advantage
does not hold up meaningfully in cross-validation and current selection is
confounded with impedance range. `frequency_median` is not materially better
after validated-candidate filtering. The global median model is simpler and
gives equivalent practical results.

Initial value from `calibration/calibration_analysis/20260717_validated`:

```text
K_global_median = 1.25065
```

Runtime firmware commands in `MAX30001G.ino`:

```text
Kp          print BIOZ scan calibration settings
Ke0         disable external scan correction
Ke1         enable external scan correction
Kg1250650   set global K to 1.250650, encoded as ppm
Kr          reset runtime calibration defaults
```

Correction defaults to OFF so raw calibration collection is not changed unless
explicitly enabled. When enabled for external scans, the spectrum table adds a
reliability column:

```text
frequency_hz,magnitude_ohm,phase_deg,reliability
```

Use runner option `--enable-correction` for verification runs. This sends `Ke1`
and `Kg<ppm>` after opening the serial port, which avoids losing runtime
correction settings when the board resets on serial connection. The default
`--correction-scale-ppm` is `1250650`.
