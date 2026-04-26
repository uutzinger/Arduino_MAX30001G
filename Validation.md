# MAX30001G Validation Plan

## Goal

Validate SPI communication, register access, internal calibration paths, and measurement modes for the MAX30001G library on real hardware.

Do not attach a subject until all bench tests in Phase 1 pass.

## Required Test Order
1. SPI and register health check
2. ECG internal signal calibration
3. BIOZ internal signal calibration
4. BIOZ internal impedance calibration
5. BIOZ internal scan
6. Subject-attached ECG test
7. Subject-attached BIOZ impedance test
8. Subject-attached combined ECG + BIOZ test
9. External BIOZ scan if needed

## Example Sketches
- `examples/Hardware_HealthCheck/Hardware_HealthCheck.ino`
- `examples/ECG_SignalCalibration/ECG_SignalCalibration.ino`
- `examples/BIOZ_SignalCalibration/BIOZ_SignalCalibration.ino`
- `examples/BIOZ_Internal_ImpedanceCalibration/BIOZ_Internal_ImpedanceCalibration.ino`
- `examples/BIOZ_Internal_ImpedanceSweep/BIOZ_Internal_ImpedanceSweep.ino`
- `examples/BIOZScan_Internal/BIOZScan_Internal.ino`
- `examples/BIOZScan_Internal_Fast/BIOZScan_Internal_Fast.ino`
- `examples/BIOZ_SettlingCharacterization/BIOZ_SettlingCharacterization.ino`
- `examples/ECG_FIFOInterruptValidation/ECG_FIFOInterruptValidation.ino`
- `examples/BIOZ_FIFOInterruptValidation/BIOZ_FIFOInterruptValidation.ino`
- `examples/ECGandBIOZ_FIFOInterruptValidation/ECGandBIOZ_FIFOInterruptValidation.ino`
- `examples/ECG/ECG.ino`
- `examples/BIOZ/BIOZ.ino`
- `examples/ECGandBIOZ/ECGandBIOZ.ino`
- `examples/BIOZScan/BIOZScan.ino`
- `examples/MAX30001G/MAX30001G.ino`

## Hardware Checklist
- [ ] MAX30001G board powered correctly
- [ ] SPI wiring verified: `MOSI`, `MISO`, `SCK`, `CS`
- [ ] `INTB` connected if available
- [ ] `INT2B` connected if available or set to `-1`
- [ ] Serial monitor set to `115200`
- [ ] No subject connected for Phase 1

## Phase 1: Bench Tests With No Subject Attached

### Test 1.1: Hardware Health Check
Objective: verify SPI communication, INFO register access, idle STATUS register access, and readable register map before any analog testing.

Primary sketch: `Hardware_HealthCheck.ino`

Interactive alternative:
```text
h
i
t
r
```

Procedure:
1. Upload the health-check sketch.
2. Open Serial Monitor at `115200`.
3. Confirm the sketch reports a valid INFO register and prints the STATUS register.
4. Confirm the register dump is stable across repeated resets.

Pass criteria:
- [x] SPI self-check passes
- [x] INFO register is non-zero and stable
- [x] STATUS register is readable
- [x] Idle PLL status is recorded
- [x] No persistent non-PLL fault flags are reported at idle
- [x] Full register dump completes without bus errors

Result: [x] PASS  [ ] FAIL
Notes: Passed on ESP32-S3 with `CS=6`, `INTB=12`, `INT2B=13`.

Recorded result:
- [x] PASS on ESP32-S3 after correcting pin mapping to `CS=6`, `INTB=12`, `INT2B=13`
- Verified `INFO REG = 0x541003`
- Verified `STATUS REG = 0x000000`
- Verified `Idle PLL Status = NO UNLOCK FLAG`
- Verified `Non-PLL Fault = NO`

### Test 1.2: ECG Signal Calibration
Objective: verify the ECG signal chain using the internal ECG calibration source.

Primary sketch: `ECG_SignalCalibration.ino`

Interactive alternative:
```text
m4
Es1
Eg2
.
z
```

Pass criteria:
- [x] Calibration waveform is present
- [x] Active PLL status reports OK after startup delay
- [x] Output is stable and repeatable
- [x] Sample rate matches selected ECG speed within tolerance
- [x] No clipping or dropouts are observed

Result: [x] PASS  [ ] FAIL
Notes: Passed on ESP32-S3 with `CS=6`, `INTB=12`, `INT2B=-1`.

Recorded result:
- [x] PASS after setting ECG FIFO threshold to 8 samples and resetting FIFO after PLL warm-up
- Verified active PLL status reported `OK` with `STATUS=0x000200` after startup delay
- Verified `ECG_data` was filled through the normal `afe.update()` path before the 100 ms direct FIFO fallback
- Verified no startup FIFO overflow after discarding warm-up samples with `FIFOReset()`
- Observed internal ~1 Hz ECG calibration waveform at ~256 sps
- Waveform appeared as alternating step/exponential decay, expected with the configured ECG high-pass filter

### Test 1.3: BIOZ Internal Signal Calibration
Objective: verify the BIOZ signal path using the internal BIOZ signal calibration mode.

Primary sketch: `BIOZ_SignalCalibration.ino`

Interactive alternative:
```text
m5
Bs0
Bg1
.
z
```

Pass criteria:
- [x] Raw BIOZ calibration output is present
- [x] Raw output is stable and repeatable
- [x] Sample rate matches selected BIOZ speed within tolerance
- [x] No `inf` impedance values are reported in signal-calibration mode

Result: [x] PASS  [ ] FAIL
Notes: Passed as a BIOZ VCAL routing and FIFO data-path test. The raw waveform is periodic and repeatable, but it is not an ideal 50/50 square wave at the raw BIOZ FIFO output.

Recorded result:
- [x] PASS on ESP32-S3 with `CS=6`, `INTB=12`, `INT2B=-1`
- Verified active PLL status reported `OK` with `STATUS=0x000000` after startup delay
- Verified `CNFG_CAL=0x704800`, showing VCAL enabled, bipolar mode, 0.5 mV, ~1 Hz, 50% duty
- Verified `CNFG_BMUX=0x3B0040`, showing BIOZP routed to `VCALP`, BIOZN routed to `VCALN`, and external BIOZ inputs disconnected
- Verified `CNFG_BIOZ=0xE50800`, with BIOZ enabled, gain 20 V/V, low-noise mode, and AHPF/DLPF/DHPF bypassed for the calibration test
- Confirmed signal-calibration mode must report raw FIFO values as `BIOZ_Cal_[raw]`; Ohms conversion is not valid because the BIOZ current generator is disabled
- Confirmed the prior `inf` output was caused by converting VCAL voltage calibration samples to Ohms with zero BIOZ drive current
- Confirmed the periodic raw output remained after bypassing DLPF, so the non-square shape is not caused by the BIOZ digital low-pass filter
- Updated `MAX30001G.ino` BIOZ calibration mode to use raw BIOZ FIFO reads and `BIOZ_Cal_[raw]` labeling

### Test 1.4: BIOZ Internal Impedance Calibration
Objective: verify impedance measurement using the MAX30001 internal test impedance.

Primary sketch: `BIOZ_Internal_ImpedanceCalibration.ino`

Sweep/characterization sketch: `BIOZ_Internal_ImpedanceSweep.ino`

Interactive alternative:
```text
m6
Cr1000
Bs0
Bg1
Bf8000
Bc8000
.
z
```

Pass criteria:
- [x] Measured impedance is stable
- [x] Reported impedance is the correct order of magnitude for the selected internal resistor
- [x] Two-point calibration example reports stable internal impedance values through the continuous setup/start/update path
- [x] Phase/frequency sweep is stable for repeated runs with `BIOZ_Internal_ImpedanceSweep.ino`
- [ ] Changing the selected resistor changes the measured value in the correct direction

Result: [x] FUNCTIONAL PASS  [ ] FAIL
Notes: Functional pass for the 1 kOhm low-resistance internal BIST path. The measured magnitude is repeatable but about `0.86x..0.89x` of nominal, so this should be treated as a calibration/phase-model issue rather than a data-path failure. Testing the alternate internal resistor remains open.

Recorded result:
- [x] Verified 1 kOhm low-resistance internal BIST path is selected, not the 1 MOhm path
- [x] Verified `CNFG_BMUX=0x301C40`, with low-resistance BIST enabled and `RNOM=4` for nominal 1000 ohm
- [x] Verified active PLL status reported `OK` with `STATUS=0x000000`
- [x] Corrected BIOZ Ohm conversion by using `V_ref` in volts instead of mV; prior values were 1000x too large
- [x] Corrected `BIOZ_test_rnom` unit labeling from kOhm to Ohm
- [x] At actual 8192 Hz, 8 uA, 20 V/V, 1 kOhm internal resistor, phase sweep fit showed a linear/triangular response over 0..180 degrees
- [x] 8192 Hz phase sweep fit: `Z ~= 897.01 - 9.8948 * phase_deg`, `R^2=0.99995`, magnitude ~= 890.5 ohm, offset ~= 6.5 ohm
- [x] Run automatic phase sweep at multiple frequencies using the internal impedance sweep sketch
- [x] Record fitted magnitude, offset, zero crossing, and response shape for each frequency
- [x] Sweep showed stable triangular/linear phase response and scale of approximately `0.86x..0.89x` nominal across 1024 Hz to 18204 Hz
- [x] At 4 kHz and above, fitted magnitude stabilized around `888..893 ohm` for nominal 1 kOhm
- [x] Latest confirmed sweep on 2026-04-19 matched expected registers and repeatable measurements:
  - `CNFG_BMUX=0x301C40` for all frequencies, confirming low-resistance internal BIST routing
  - `CNFG_BIOZ_LC=0x800055` for all frequencies
  - `CNFG_BIOZ=0x951710`, `0x951610`, `0x951510`, `0x951410`, `0x951310` for actual `1024`, `2048`, `4096`, `8192`, `18204 Hz`
  - `STATUS=0x000000` and `pll=OK` throughout the sweep
- [x] Decided `BIOZScan_Internal.ino` should use a triangular phase-response model for internal-BIST validation
- [x] Ran `BIOZScan_Internal.ino` through acquisition-scale validation and matched the manual internal impedance sweep
- [x] Split the original internal impedance sweep use case into `BIOZ_Internal_ImpedanceSweep.ino`
- [x] Repurposed `BIOZ_Internal_ImpedanceCalibration.ino` as a two-point internal impedance example that uses the normal continuous-profile sequence: `setupBIOZImpedanceCalibration(...)`, `start()`, `update()`, FIFO discard, and averaging
- [x] Confirmed repurposed two-point `BIOZ_Internal_ImpedanceCalibration.ino` on 2026-04-25:
  - Startup idle PLL reported `OK` with `STATUS=0x0`
  - `baseline,8000,8192.0,0.00,891.078,890.899,891.161,8,0x200,OK`
  - `changed_frequency_phase,4000,4096.0,45.00,462.353,462.246,462.377,8,0x200,OK`
  - Register snapshots confirmed internal 1 kOhm low-resistance BIST routing with `CNFG_BMUX=0x301C40` and `CNFG_BIOZ_LC=0x800055`
  - 8 kHz / 0 deg point used `CNFG_BIOZ=0x951410`; 4 kHz / 45 deg point used `CNFG_BIOZ=0x951514`
  - The post-measurement `status_hex=0x200` rows still decoded as `pll=OK`; register snapshots before collection showed `STATUS=0x000000`

Latest confirmed 1 kOhm internal BIST sweep:

| Requested | Actual | 0 deg | 45 deg | 90 deg | 135 deg | 168.75 deg |
|---:|---:|---:|---:|---:|---:|---:|
| 1000 Hz | 1024 Hz | 876.279 ohm | 506.404 ohm | 93.347 ohm | -366.102 ohm | -745.307 ohm |
| 2000 Hz | 2048 Hz | 886.703 ohm | 478.733 ohm | 47.676 ohm | -407.933 ohm | -766.209 ohm |
| 4000 Hz | 4096 Hz | 889.841 ohm | 462.514 ohm | 23.490 ohm | -427.692 ohm | -774.538 ohm |
| 8000 Hz | 8192 Hz | 891.438 ohm | 454.660 ohm | 12.358 ohm | -436.418 ohm | -778.097 ohm |
| 16000 Hz | 18204 Hz | 893.696 ohm | 452.025 ohm | 7.457 ohm | -440.688 ohm | -781.345 ohm |

Sweep fit summary:

| Requested | Actual | Magnitude Fit | Scale vs 1k | Zero Crossing | Offset | Fit Quality |
|---:|---:|---:|---:|---:|---:|---:|
| 1000 Hz | 1024 Hz | 864.4 ohm | 0.864x | 95.34 deg | 51.3 ohm | R2=0.99665 |
| 2000 Hz | 2048 Hz | 881.7 ohm | 0.882x | 92.63 deg | 25.7 ohm | R2=0.99913 |
| 4000 Hz | 4096 Hz | 887.8 ohm | 0.888x | 91.27 deg | 12.5 ohm | R2=0.99978 |
| 8000 Hz | 8192 Hz | 890.4 ohm | 0.890x | 90.66 deg | 6.5 ohm | R2=0.99994 |
| 16000 Hz | 18204 Hz | 893.1 ohm | 0.893x | 90.39 deg | 3.9 ohm | R2=0.99998 |

### Test 1.5: BIOZ Internal Scan
Objective: verify the non-blocking BIOZ scan engine using the internal test resistor before scanning electrodes.

Primary sketch: `BIOZScan_Internal.ino`

Interactive alternative:
```text
m8
Si1
Cr1000
Sa2
Sf0
Sr0
.
```

Expected output:
- CSV-style lines with `frequency_hz,magnitude_ohm,phase_deg`

Pass criteria:
- [x] Scan completes without hanging
- [x] Frequency sweep is printed
- [x] Magnitude and phase-point diagnostics are reported for scanned frequencies
- [x] Repeated fixed-current internal-BIST phase points are reasonably consistent
- [x] Final triangular-fit summary output matches the manual internal impedance sweep

Result: [x] PASS  [ ] FAIL
Notes: Internal-BIST scan acquisition and triangular-fit summary output match `BIOZ_Internal_ImpedanceSweep.ino` for fixed 8 uA validation mode. Current pass is for the internal 1 kOhm BIST validation path; current auto-ranging, production timing, and external-load calibration remain follow-up tasks.

Current intended BIOZ scan behavior:
- Configure the BIOZ scan once, then run it as a non-blocking state machine from `update()`.
- Sweep selected BIOZ modulation frequencies from `freq_start_index` to `freq_end_index`.
- For each frequency, start with a configured drive current, measure multiple demodulation phase offsets, and average FIFO samples at each phase.
- Use raw ADC magnitude to decide whether the current is too low, in range, or too high; retry the same frequency with an adjusted current when allowed by `max_retries`.
- Convert each phase point to Ohms, then fit a phase-response model to produce one `frequency_hz,magnitude_ohm,phase_deg` result per frequency.
- Save the current profile for possible reuse in later scans.

Temporary internal-BIST validation spec:
- Internal scan validation must first match `BIOZ_Internal_ImpedanceSweep.ino` before current auto-ranging or execution-time optimization is trusted.
- Use fixed `8 uA`, `20 V/V`, low-rate BIOZ sampling, `AHPF=150 Hz`, `DLPF=4 Hz`, `DHPF=bypass`, internal 1 kOhm low-resistance BIST.
- Configure in the same order as the working internal impedance validation path: frequency, current, modulation mode, phase, BIST route, BIOZ enable, FIFO reset/synchronization, warm-up, clear status, FIFO reset.
- Discard three FIFO thresholds after each phase/frequency change, then average the next threshold.
- Expected validation scale is the Test 1.4 table: hundreds of Ohms at 0/45/135/168.75 degree phase points.
- Once triangular-fit summary output is verified, remove or isolate the slow validation behavior and restore the intended faster scan/current-ranging behavior.

Current BIOZ data-path note:
- `BIOZ_Internal_ImpedanceCalibration.ino` uses the normal configured-profile path for two internal BIST settings. It calls `setupBIOZImpedanceCalibration(...)`, `start()`, `update(false)`, and also periodically calls `readBIOZ_FIFO(false)` as a direct FIFO polling fallback before averaging values popped from `BIOZ_data`.
- `BIOZScan_Internal.ino` / `stepBIOZScan()` now owns scan acquisition directly in the scan state machine. It reads raw FIFO samples after interrupt/status polling, discards sample-count-based settling samples, averages settled samples, converts to ohms, and fits the phase response.
- Therefore the present scan and calibration validation programs are buffered, but they are not pure interrupt-driven acquisition tests. A separate later task should validate interrupt-only FIFO draining for continuous BIOZ streaming examples.

Recorded progress as of 2026-04-19:
- [x] `BIOZScan_Internal.ino` completes without hanging on ESP32-S3 with `CS=6`, `INTB=12`, `INT2B=-1`
- [x] Added PLL warm-up/status check and a 180 s timeout to the internal scan example
- [x] Added temporary phase-point diagnostics: `scan_phase,frequency_hz,phase_deg,raw_adc,ohm,current_nA`
- [x] Added temporary scan register diagnostics: `scan_registers,f=...,attempt=...,current_nA=...`
- [x] Added public driver helper `printBiozDiagnosticRegisters()` so sketches do not call private `readRegister24()`
- [x] Confirmed `CNFG_BMUX=0x301C40` in both scan and the working internal impedance validation path, meaning low-resistance internal BIST is enabled with `RNOM=4` for nominal 1000 ohm
- [x] Confirmed scan acquisition timing improved after adding a discard-before-measurement state and using `avg=8`
- [x] Confirmed scan setup sequence was corrected to `frequency -> current -> modulation mode`
- [x] Confirmed fixed-current validation mode uses `avg=8`, `max_retries=1`, `initial_current_nA=8000`
- [x] Compared failing scan registers against the then-working internal impedance sweep path
- [x] Root-cause candidate identified: scan `CNFG_BIOZ` differed from the working internal impedance validation path even though BIST routing matched
- [x] Temporarily moved internal point acquisition into a shared helper used by the internal impedance sweep path and the internal scan validation path
- [x] Restored `BIOZScan_Internal.ino` to the expected `~893/891/889/887/873 ohm` spectrum after switching the internal validation branch to the helper lifecycle
- [x] Removed temporary `scan_internal_*` debug output after validation passed
- [x] Failing scan at 8192 Hz had `CNFG_BIOZ=0xAD141F`, decoded as `AHPF=500Hz`, `EXT_RBIAS=1`, `FCGEN=4`, `CGMAG=8uA`, `gain=20V/V`, `DLPF=4Hz`
- [x] Working internal impedance validation path at 8192 Hz had `CNFG_BIOZ=0x951410`, decoded as `AHPF=150Hz`, `EXT_RBIAS=0`, `FCGEN=4`, `CGMAG=8uA`, `gain=20V/V`, `DLPF=4Hz`
- [x] Patched internal-BIST scan path to force `AHPF=150Hz` and call `setLeadsBias(false, 0U)` after setting frequency/current/modulation so `EXT_RBIAS` should clear to 0
- [x] Re-uploaded `BIOZScan_Internal.ino` after the `AHPF=150Hz` / `EXT_RBIAS=0` patch
- [x] Verified scan registers are now close to the working internal impedance validation path: at 8192 Hz `CNFG_BMUX=0x301C40`, `CNFG_BIOZ=0x95141F`, `CNFG_BIOZ_LC=0x800055`
- [x] Confirmed remaining `CNFG_BIOZ=0x...1F` vs `0x...10` difference is the phase-offset low nibble, not an AHPF/EXT_RBIAS/current/gain difference
- [x] Confirmed scan amplitude is still wrong after register parity: scan reports raw `~45` counts and fitted magnitude `~0.425 ohm`, while Test 1.4 reports hundreds of Ohms
- [x] Patched internal-BIST validation path to match the working internal impedance validation sequence more closely:
  - BIOZ FIFO threshold parity changed to `setFIFOInterruptThreshold(32U, avg)`
  - BIOZ is disabled before internal scan frequency and phase reconfiguration
  - BIOZ filter and internal BIST route are re-applied in the same late setup portion as the working internal impedance validation path
  - BIOZ is re-enabled per phase with lead settings applied
  - added `FIFOReset()`, `synch()`, warm-up delay, status clear, and final `FIFOReset()` before collecting samples
  - discard phase now clears the first FIFO threshold without an extra reset/synchronization for internal-BIST validation
- [x] Re-upload after conservative validation patch showed major improvement: scan phase values increased from `~0.4 ohm` scale to about `180..222 ohm` fitted/phase scale
- [x] Verified `MNGR_INT=0xFF0014` now matches the working internal impedance validation threshold register
- [x] Verified scan phase response shape is now triangular/linear and polarity matches Test 1.4
- [x] Found harmless validation warning source: `setLeadsBias(false, 0U)` was called while BIOZ was intentionally disabled during reconfiguration
- [x] Removed disabled-AFE `setLeadsBias(false, 0U)` call; per-phase lead-bias clearing remains after BIOZ is re-enabled
- [x] Resolved scale mismatch by increasing internal-BIST validation discard to three FIFO thresholds
- [x] Added one-point diagnostic at 8192 Hz / 0 deg comparing `readBIOZ_FIFO(true)` raw conversion against the next FIFO threshold read with `readBIOZ_FIFO(false)` under the same active phase/frequency setup
- [x] Reviewed `fifo_compare` at 8192 Hz / 0 deg: `raw_to_ohm=222.144`, `converted_ohm=903.568`, `ratio=4.067`
- [x] Interpreted `ratio=4.067` as a settling/discard issue rather than a raw-conversion formula bug, because the converted path was the next FIFO threshold at the same active setup and matched the expected Test 1.4 scale
- [x] Increased internal-BIST scan discard from one FIFO threshold to two FIFO thresholds before processing each phase point
- [x] Re-tested after two-threshold discard; scale now matches Test 1.4:
  - `scan_phase,8192.0,0.00` moved from `~222 ohm` to `903.63 ohm`
  - `fifo_compare` at 8192 Hz / 0 deg reported `raw_to_ohm=903.635`, `converted_ohm=891.248`, `ratio=0.986`
  - 8192 Hz phase sweep now matches expected triangular/linear internal-BIST response: 0 deg `903.63 ohm`, 45 deg `461.26 ohm`, 90 deg `12.51 ohm`, 135 deg `-442.02 ohm`, 168.75 deg `-788.45 ohm`
  - 4096 Hz phase sweep also matches expected scale: 0 deg `902.15 ohm`, 45 deg `469.05 ohm`, 90 deg `24.01 ohm`, 135 deg `-433.42 ohm`, 168.75 deg `-784.79 ohm`
- [x] Changed internal-BIST validation discard from two FIFO thresholds to three FIFO thresholds for one follow-up settling test
- [x] Re-tested after three-threshold discard; internal-BIST scan now matches the manual calibration sweep:
  - 8192 Hz / 0 deg moved from the two-threshold value `903.63 ohm` to `891.26 ohm`
  - `fifo_compare` reported `raw_to_ohm=891.256`, `converted_ohm=891.222`, `ratio=1.000`
  - 8192 Hz phase sweep: 0 deg `891.26 ohm`, 45 deg `454.96 ohm`, 90 deg `12.36 ohm`, 135 deg `-435.91 ohm`, 168.75 deg `-777.75 ohm`
  - Three FIFO thresholds are the current known-good internal-BIST settling discard for `avg=8`, low-rate BIOZ, `DLPF=4 Hz`
- [x] Internal-BIST scan acquisition scale issue is resolved for fixed 8 uA validation mode
- [x] Triangular-fit final summary output passed:
  - `18204.0 Hz`: `893.486 ohm`, phase `0.250 deg`
  - `8192.0 Hz`: `891.780 ohm`, phase `0.500 deg`
  - `4096.0 Hz`: `889.897 ohm`, phase `0.750 deg`
  - `2048.0 Hz`: `887.297 ohm`, phase `1.750 deg`
  - `1024.0 Hz`: `874.445 ohm`, phase `3.250 deg`
- [x] Measured whether faster BIOZ sampling and different DLPF settings reduce settling time using `BIOZ_SettlingCharacterization.ino`.
- [x] Added sample-based BIOZ scan settling defaults: 24 samples normally, 32 samples after current changes, converted to FIFO discard events with `ceil(samples / avg)`.
- [ ] Follow-up: measure final-window noise and repeatability for DLPF bypass, 4 Hz, 8 Hz, and 16 Hz before changing production DLPF from 4 Hz.
- [x] After acquisition amplitude matched, implemented triangular fit for internal-BIST validation while keeping cosine fit for external/tissue scans

## BIOZ Calibration And Scan Follow-Up Tasks

These tasks are tracked here instead of a separate TODO file.

### BIOZScan Model Task
- [x] Run `BIOZScan_Internal.ino` and compare scan output against the manual frequency/phase sweep in Test 1.4.
- [x] Determined that the existing scan fit assumed a sinusoidal/cosine phase response.
- [x] Renamed the cosine least-squares fit to `fitImpedanceCosine()`.
- [x] Added `impedanceTriangularModel()` and `fitImpedanceTriangular()` for internal-BIST validation.
- [x] Updated scan finalization so internal-resistor scans use the triangular fit and external/tissue scans keep using the cosine fit.
- [x] Ran `BIOZScan_Internal.ino` after triangular-fit change and compared final `frequency_hz,magnitude_ohm,phase_deg` output against phase-point data.
- [x] Determined triangular fit is sufficient for the internal 1 kOhm BIST validation path.
- [ ] Later decision: determine whether production internal-BIST validation should keep triangular-fit summary output or instead use:
  - max-absolute response across phase,
  - measured scale-factor correction,
  - a separate internal-BIST-only output table.
- [x] Recorded that scan magnitude follows the observed `0.86x..0.89x` internal-resistor scale.
- [ ] For tissue/sample measurement with reactance, keep enough phase information to estimate complex impedance, not only magnitude.
- [ ] Decide whether internal BIST should be used only as a data-path/scale check while external calibration loads are used for complex impedance calibration.

### BIOZScan Settling Characterization Task
Do this after the internal-BIST scan fit model is addressed. Prefer a dedicated diagnostic sketch or explicit diagnostic mode rather than adding more timing experiments to the production scan loop.

- [x] Created dedicated diagnostic sketch: `examples/BIOZ_SettlingCharacterization/BIOZ_SettlingCharacterization.ino`
  - Uses the internal 1 kOhm BIOZ BIST path.
  - Uses a 1-sample BIOZ FIFO threshold by default for finer time resolution.
  - Prints `sample` CSV rows for individual post-transition samples.
  - Also prints `group8` CSV rows that summarize each consecutive 8-sample block, so results can still be compared with the current production-style scan averaging.
  - Includes compile-time switches for phase, frequency, current, sampling-rate, DLPF, and BIOZ-start tests.
  - Reports `delta_from_final_percent` using the mean of the last captured 8 valid samples in each case as the temporary final reference.
  - Reports a per-case 0.1% settling summary after the individual sample rows using the first 8-consecutive-sample window within threshold.
  - Prints a final summary table with settled sample index, elapsed milliseconds, and equivalent 8-sample FIFO blocks for each experiment.
  - Runs the same phase/frequency/current/DLPF/BIOZ-start suite at both low and fast BIOZ sampling rates, plus explicit low-to-fast and fast-to-low sampling-rate transition tests.
- [x] Measure phase-change settling at 8192 Hz, fixed 8 uA, fixed 1 kOhm internal BIST:
  - step from 0 deg to 45 deg and record individual samples plus 8-sample group summaries,
  - step from 45 deg to 90 deg and record individual samples plus 8-sample group summaries,
  - step from 168.75 deg back to 0 deg and record individual samples plus 8-sample group summaries.
- [x] Measure frequency-change settling with phase and current fixed:
  - 4096 Hz to 8192 Hz,
  - 8192 Hz to 4096 Hz,
  - optionally 2048 Hz to 8192 Hz for a larger transition.
- [x] Measure current-change settling with frequency and phase fixed:
  - 8 uA to 16 uA,
  - 8 uA to 32 uA,
  - reverse transitions back to 8 uA.
- [x] Measure BIOZ enable/start settling:
  - BIOZ disabled to enabled at 8192 Hz / 0 deg,
  - record individual samples plus 8-sample group summaries after `FIFOReset()` and `synch()`.
- [x] Include sampling-rate tests:
  - low-rate BIOZ (`fast=false`, ~32 sps),
  - fast BIOZ (`fast=true`, ~64 sps),
  - compare settling in samples and in milliseconds.
- [x] Include digital-filter settling tests:
  - DLPF bypass,
  - DLPF 4 Hz,
  - DLPF 8 Hz and/or 16 Hz if useful,
  - record how many FIFO thresholds are needed to reach within 1%, 2%, and 5% of the final value.
- [x] Keep AHPF condition in the settling record because AHPF cutoff may affect baseline and response:
  - Settling characterization results are for AHPF 150 Hz, matching the known-good internal-BIST validation setup.
- [x] Use the settling results to choose production scan timing:
  - default discard threshold count,
  - whether discard count should depend on sampling rate, DLPF, frequency, or transition type,
  - whether current changes require a longer discard than phase-only changes.

#### Settling Characterization Results

Criterion: first point where 8 consecutive valid samples stay within 0.1% of the final reference. The characterization sketch was updated to mirror BIOZScan point acquisition: scan-like low-level setup, FIFO threshold `avg=8`, raw FIFO reads, scan-style raw-to-ohm conversion, `FIFOReset()`, `synch()`, and sample-count settling. The 45 deg to 90 deg phase case has a final demodulated value of only about `12 ohm`, so the relative threshold is comparable to quantization/noise; do not use that row alone to size scan settling.

| Suite | Test | Transition | Speed | DLPF | Settled sample | Settled ms | 8-sample blocks |
|---|---|---|---|---|---:|---:|---:|
| low | phase | 0deg -> 45deg | low | 4Hz | 24 | 751 | 3 |
| low | phase | 45deg -> 90deg | low | 4Hz | 72 | 2251 | 9 |
| low | phase | 168.75deg -> 0deg | low | 4Hz | 24 | 751 | 3 |
| low | frequency | 4096Hz -> 8192Hz | low | 4Hz | 16 | 752 | 2 |
| low | frequency | 8192Hz -> 4096Hz | low | 4Hz | 16 | 752 | 2 |
| low | frequency | 2048Hz -> 8192Hz | low | 4Hz | 16 | 752 | 2 |
| low | current | 8000nA -> 16000nA | low | 4Hz | 25 | 1001 | 4 |
| low | current | 16000nA -> 8000nA | low | 4Hz | 25 | 1001 | 4 |
| low | current | 8000nA -> 32000nA | low | 4Hz | 25 | 1001 | 4 |
| low | current | 32000nA -> 8000nA | low | 4Hz | 25 | 1001 | 4 |
| low | dlpf | 4Hz -> bypass | low | bypass | 15 | 501 | 2 |
| low | dlpf | bypass -> 4Hz | low | 4Hz | 24 | 751 | 3 |
| low | dlpf | 4Hz -> 8Hz | low | 8Hz | 15 | 501 | 2 |
| low | dlpf | 4Hz -> 16Hz | low | 16Hz | 24 | 751 | 3 |
| low | bioz_start | stopped -> enabled | low | 4Hz | 25 | 1001 | 4 |
| fast | phase | 0deg -> 45deg | fast | 4Hz | 15 | 379 | 2 |
| fast | phase | 45deg -> 90deg | fast | 4Hz | not settled | 0 | 0 |
| fast | phase | 168.75deg -> 0deg | fast | 4Hz | 15 | 379 | 2 |
| fast | frequency | 4096Hz -> 8192Hz | fast | 4Hz | 16 | 378 | 2 |
| fast | frequency | 8192Hz -> 4096Hz | fast | 4Hz | 16 | 378 | 2 |
| fast | frequency | 2048Hz -> 8192Hz | fast | 4Hz | 16 | 378 | 2 |
| fast | current | 8000nA -> 16000nA | fast | 4Hz | 17 | 505 | 3 |
| fast | current | 16000nA -> 8000nA | fast | 4Hz | 17 | 505 | 3 |
| fast | current | 8000nA -> 32000nA | fast | 4Hz | 17 | 505 | 3 |
| fast | current | 32000nA -> 8000nA | fast | 4Hz | 17 | 505 | 3 |
| fast | dlpf | 4Hz -> bypass | fast | bypass | 13 | 251 | 2 |
| fast | dlpf | bypass -> 4Hz | fast | 4Hz | 24 | 377 | 3 |
| fast | dlpf | 4Hz -> 8Hz | fast | 8Hz | 23 | 377 | 3 |
| fast | dlpf | 4Hz -> 16Hz | fast | 16Hz | 13 | 251 | 2 |
| fast | bioz_start | stopped -> enabled | fast | 4Hz | 17 | 505 | 3 |
| transition | sampling_rate | low -> fast | fast | 4Hz | 24 | 377 | 3 |
| transition | sampling_rate | fast -> low | low | 4Hz | 24 | 751 | 3 |


#### Settling Strategy Applied To BIOZ Scan

- Use sample-count settling defaults, not hard-coded FIFO-block counts, because FIFO block count changes with `avg`.
- Default phase/frequency/filter settling: 24 samples.
- Current-change settling: 32 samples.
- Convert samples to FIFO discard events as `ceil(settle_samples / avg)`.
  - `avg=8`: 24 samples = 3 FIFO events; 32 samples = 4 FIFO events.
  - `avg=4`: 24 samples = 6 FIFO events; 32 samples = 8 FIFO events.
  - `avg=2`: 24 samples = 12 FIFO events; 32 samples = 16 FIFO events.
- Implemented in `BIOZScanConfig` as `settle_samples = 24` and `current_change_settle_samples = 32`.
- Scan-like characterization shows frequency-only transitions settle by about 16 samples, but phase/filter/start/sampling-rate transitions still reach 24 to 25 samples in representative cases. Keep the single default at 24 samples until per-transition settle policies are implemented.
- Current-change transitions settled by 25 samples at low rate and 17 samples at fast rate. Keep `current_change_settle_samples = 32` because it maps to a conservative fourth FIFO block at `avg=8` and covers low-rate current changes.
- Fast BIOZ sampling reduces elapsed settling time but does not justify reducing the default sample count without per-transition policies.
- Keep DLPF at 4 Hz until noise/repeatability is compared against bypass, 8 Hz, and 16 Hz. Some DLPF transitions settle faster, but settling speed alone is not enough to choose the measurement filter.

#### Current Status

- Settling characterization is complete for the internal 1 kOhm BIST path with AHPF 150 Hz.
- Scan-like settling characterization was rerun after BIOZScan state-machine refactor.
- BIOZ scan settling defaults remain sample-based: 24 samples normally and 32 samples after current changes.
- Fast BIOZ sampling should reduce elapsed scan time without reducing the global discard sample count.
- DLPF remains at 4 Hz until noise/repeatability is validated across filter choices.
- Temporary `scan_phase`, `scan_registers`, and `fifo_compare` diagnostics have been removed from normal scan output.

### BIOZScan Cleanup After Validation
Remove or reduce temporary diagnostics once `BIOZScan_Internal.ino` reports the expected internal BIST amplitude and repeatable phase response.

- [x] Remove temporary `scan_phase,...` CSV printing from the scan driver, or guard it behind an explicit debug/config flag.
- [x] Remove temporary `scan_registers` register dumps from the scan driver, or guard them behind the same diagnostic flag.
- [x] Remove temporary `fifo_compare,...` one-point converted/raw FIFO diagnostic from the scan driver.
- [x] Remove the startup register dump from `BIOZScan_Internal.ino` after register parity with the internal impedance validation path is confirmed.
- [x] Revisit `BIOZScan_Internal.ino` validation-only settings: `avg = 8`, `max_retries = 1`, fixed `initial_current_nA = 8000`.
  - Keep these settings for internal BIST regression validation until the refactored scan state machine reproduces the known spectrum.
- [x] Keep the `frequency -> current -> modulation mode -> phase` setup order in the driver.
- [x] Keep configurable sample-count settling through `settle_samples` and `current_change_settle_samples`.
- [x] Update comments and serial output names so the final example distinguishes validation diagnostics from normal scan output.

Validated internal-BIST scan baselines:

- Initial expected fixed-current internal 1 kOhm BIST spectrum:
  - `18204.0,893.022,0.250`
  - `8192.0,891.203,0.500`
  - `4096.0,889.562,0.750`
  - `2048.0,886.522,1.750`
  - `1024.0,873.769,3.250`
- Helper-based restored internal scan spectrum:
  - `18204.0,892.817,0.250`
  - `8192.0,891.098,0.500`
  - `4096.0,889.340,0.750`
  - `2048.0,886.528,1.750`
  - `1024.0,873.391,3.250`
- Conclusion: the original internal scan regression came from scan-specific acquisition lifecycle differences, not from the triangular fit or the sample-count settling defaults.

Before/after settling comparison after attempting to remove the internal-only BIOZ stop/restart path:

- Settling results were materially unchanged for scan-sizing cases.
- Final reference values shifted only by small run-to-run noise, typically a few hundredths of an ohm.
- The known low-signal `45deg -> 90deg` phase case is not used to size settling because the final demodulated value is only about `12.5 ohm`.
- Conclusion: removing the internal-only stop/restart behavior did not justify changing production settling defaults. Keep `24` samples for normal phase/frequency/filter/start transitions and `32` samples for current changes.

### BIOZScan Refactor After Cleanup
Reference: `Refactor.md`.

Reason for refactor:
- The helper-based internal validation path restored correct BIST results, but it hid point acquisition inside `measureBIOZInternalImpedancePoint(...)`.
- The desired production scan architecture is a nonblocking `stepBIOZScan()` state machine where internal and external scans share the same scan-owned discard, collect, process, retry, and fit flow.

Completed refactor status:
- [x] `setupBIOZScan(...)` configures only scan profile/runtime state and does not begin acquisition.
- [x] `startBIOZScan()` only arms the state machine; hardware setup begins in `BIOZ_SCAN_INIT`.
- [x] `start()` and `update()` branch to scan-specific workers only for `PROFILE_BIOZ_SCAN`.
- [x] `stepBIOZScan()` now owns internal and external point acquisition.
- [x] Added explicit scan states for current setup, point begin, sample discard, sample collection, point processing, and phase advance.
- [x] Removed scan-time dependence on `measureBIOZInternalImpedancePoint(...)`.
- [x] Internal and external scans now share the same scan-owned discard/collect/process path.
- [x] FIFO reads use raw BIOZ samples for current range evaluation, then convert raw mean to ohms before impedance fitting.
- [x] Internal BIST still uses the triangular fit; external scan still uses the cosine fit.
- [x] Continuous fixed-frequency BIOZ setup/start/update behavior was not changed.

BIOZScan refactor validation plan:
- [ ] Build examples after each mechanical refactor step.
- [x] Run `BIOZ_Internal_ImpedanceCalibration` after any change to the continuous internal BIST setup/start/update path.
  - Confirmed 2026-04-25:
    - `baseline,8000,8192.0,0.00,891.078,890.899,891.161,8,0x200,OK`
    - `changed_frequency_phase,4000,4096.0,45.00,462.353,462.246,462.377,8,0x200,OK`
    - Register snapshots matched expected internal BIST path: `CNFG_BMUX=0x301C40`, `CNFG_BIOZ_LC=0x800055`, `CNFG_BIOZ=0x951410` at 8 kHz / 0 deg, and `CNFG_BIOZ=0x951514` at 4 kHz / 45 deg.
- [x] Run `BIOZ_Internal_ImpedanceSweep` after any change to internal BIST phase/frequency characterization or the blocking helper.
  - Confirmed 2026-04-25:
    - `CNFG_BMUX=0x301C40`, `CNFG_BIOZ_LC=0x800055`, `STATUS=0x000000`, and `pll=OK` throughout the sweep.
    - `CNFG_BIOZ=0x951710`, `0x951610`, `0x951510`, `0x951410`, `0x951310` for actual `1024`, `2048`, `4096`, `8192`, `18204 Hz`.
    - Sweep point summary:
      - 1024 Hz: `876.412`, `506.398`, `93.304`, `-366.153`, `-745.466 ohm` at `0`, `45`, `90`, `135`, `168.75 deg`.
      - 2048 Hz: `886.862`, `478.758`, `47.612`, `-407.988`, `-766.361 ohm`.
      - 4096 Hz: `890.009`, `462.617`, `23.474`, `-427.754`, `-774.689 ohm`.
      - 8192 Hz: `891.571`, `454.746`, `12.378`, `-436.482`, `-778.264 ohm`.
      - 18204 Hz: `893.834`, `452.133`, `7.458`, `-440.747`, `-781.435 ohm`.
- [x] Run `BIOZScan_Internal` with the scan-owned internal/external point states.
  - Confirmed 2026-04-25 with validation settings `avg=8`, `current=8000nA`, `settle_samples=24`, `current_change_settle_samples=32`.
  - Startup active PLL reported `OK` with `STATUS=0x200`.
  - Final scan spectrum:
    - `18204.0,905.769,0.250`
    - `8192.0,903.750,0.500`
    - `4096.0,902.211,0.750`
    - `2048.0,899.540,1.750`
    - `1024.0,886.187,3.250`
- [ ] Confirm internal 1 kOhm BIST remains close to the known baseline:
  - 18.204 kHz: about 893 ohm,
  - 8.192 kHz: about 891 ohm,
  - 4.096 kHz: about 889 ohm,
  - 2.048 kHz: about 886 ohm,
  - 1.024 kHz: about 873 ohm.
  - 2026-04-25 scan shape and phase are correct, but magnitude is about `12..13 ohm` higher than the same-day `BIOZ_Internal_ImpedanceSweep` reference at each frequency. Keep this item open until scan/sweep scale parity is explained or accepted.
- [x] Run `BIOZ_SettlingCharacterization` only after the scan path is correct.
  - Completed with scan-like setup/collection after the BIOZScan state-machine refactor.
  - Result supports keeping global scan defaults at `settle_samples=24` and `current_change_settle_samples=32`.
  - Frequency-only transitions may be candidates for a later per-transition optimization to `16` samples, but phase/filter/start transitions still justify the 24-sample default.
- [ ] Run external known-load validation before claiming external scan accuracy.
- [ ] Compare external known-load scans with internal-only restart versus restart for both internal and external paths.

BIOZScan later optimization work:
- [ ] Avoid stop/start around phase-only transitions unless hardware validation requires it.
- [ ] Avoid rewriting unchanged filter/gain/path registers between points.
- [ ] Tune discard count below 24 samples only after before/after data proves the first stable sample arrives earlier.
- [x] Remove `measureBIOZInternalImpedancePoint(...)`.
  - The driver no longer exposes the blocking internal impedance point helper.
  - `BIOZ_Internal_ImpedanceCalibration.ino` and `BIOZ_Internal_ImpedanceSweep.ino` now use explicit setup/start/update/FIFO collection paths for internal BIST measurements.
  - Scan behavior is owned by `stepBIOZScan()` rather than hidden in a blocking helper.
- [ ] Consider a shared lower-level point-acquisition engine that a blocking wrapper can call without reintroducing hidden scan logic.

### BIOZ FIFO/Interrupt Data-Path Task
Keep this task, but scope it only to continuous BIOZ streaming and the common `update()` FIFO-drain path.

BIOZ scan no longer depends on this path:
- `BIOZScan_Internal.ino` / `stepBIOZScan()` read FIFO data through scan-owned states after interrupt/status polling.
- Scan correctness should be validated with scan examples and settling characterization, not with continuous streaming examples.

Continuous BIOZ still depends on this path:
- `BIOZ.ino` expects `afe.update()` to service interrupts/status and fill `BIOZ_data`.
- `ECGandBIOZ.ino` expects `afe.update()` to service ECG, BIOZ, and R-to-R without starving BIOZ FIFO reads.
- The examples currently include direct FIFO fallback reads after a timeout, so they prove data can be recovered, but they do not prove the interrupt-driven path alone is healthy.

- [x] Decide whether this task is still valid after the BIOZScan refactor.
  - Keep it for continuous streaming only.
  - Do not use it as an internal-BIST scan correctness gate.
- [x] Add dedicated FIFO validation sketches that run with direct FIFO fallback disabled during the validation window.
  - `ECG_FIFOInterruptValidation.ino`
  - `BIOZ_FIFOInterruptValidation.ino`
  - `ECGandBIOZ_FIFOInterruptValidation.ino`
- [x] Run `ECG_FIFOInterruptValidation.ino` and verify `ECG_data` fills correctly through `afe.update()`.
  - First run failed only because a stale startup overflow flag was counted: `samples_received=3832`, `max_gap_ms=32`, `overflow_events=1`, `pll=OK`.
  - After clearing validation state after warm-up FIFO reset: `samples_received=3832`, `expected_min_samples=1500`, `max_gap_ms=32`, `max_allowed_gap_ms=250`, `overflow_events=0`, `status_hex=0x0`, `pll=OK`, `result=PASS`.
- [x] Run `BIOZ_FIFOInterruptValidation.ino` and verify `BIOZ_data` fills correctly through `afe.update()`.
  - `samples_received=480`, `expected_min_samples=250`, `max_gap_ms=250`, `max_allowed_gap_ms=750`, `overflow_events=0`, `status_hex=0x0`, `pll=OK`, `result=PASS`.
- [x] Run `ECGandBIOZ_FIFOInterruptValidation.ino` and verify `ECG_data` and `BIOZ_data` both fill correctly through `afe.update()` without FIFO starvation.
  - `ecg_samples_received=3832`, `ecg_expected_min_samples=1500`, `ecg_max_gap_ms=32`, `ecg_max_allowed_gap_ms=250`, `ecg_overflow_events=0`.
  - `bioz_samples_received=480`, `bioz_expected_min_samples=250`, `bioz_max_gap_ms=250`, `bioz_max_allowed_gap_ms=750`, `bioz_overflow_events=0`.
  - `status_hex=0x0`, `pll=OK`, `result=PASS`.
  - Non-blocking setup messages observed: `EN_RTOR requires EN_ECG=1` warning and `ECG and BIOZ active, setting ECG bias only` error text. They did not prevent ECG/BIOZ FIFO service from passing, but the lead-bias message severity/text should be reviewed separately.
- [ ] Verify continuous FIFO validation still works with status-polling fallback when interrupt wiring is absent or disabled.
- [x] Keep direct FIFO polling fallback in `BIOZ_Internal_ImpedanceCalibration.ino` for controlled bench validation.
  - It is intentionally a point-measurement validation sketch, not a proof of production streaming interrupt behavior.
- [x] Decide whether `BIOZScan_Internal.ino` should read raw FIFO samples directly during scan states or consume only samples already pushed by the common update/FIFO-drain path.
  - The refactored scan state machine reads FIFO data through scan-owned states after interrupt/status polling, so scan correctness no longer depends on the continuous streaming FIFO-drain path.

### Update MAX30001G

- [x] Rename the interactive sketch from `MAX30001G_Test.ino` to `MAX30001G.ino`.
- [x] Update `showSettings()` so it reports meaningful engineering values with units instead of raw selector integers where possible.
  - ECG and BIOZ settings should print rates in `sps`, gain in `V/V`, frequency in `Hz`, current in `nA`, phase in `deg`, and filter settings as real cutoff frequencies or `bypass`.
  - Internal-BIST calibration settings should print resistor in `Ohm`, RMOD step size in `Ohm`, and modulation frequency in `Hz`.
- [x] Update interactive command acknowledgements so entries like `Ba1`, `Bd2`, `Bh1`, `Bs1`, `Cm3`, and `Cf4` respond with meaningful values and units instead of only echoing the selector code.
- [x] Use applied driver globals after setup when reporting effective hardware settings.
  - Prefer `BIOZ_frequency`, `BIOZ_cgmag`, `BIOZ_phase`, `BIOZ_ahpf`, `BIOZ_dlpf`, `BIOZ_dhpf`, `BIOZ_samplingRate`, `BIOZ_gain`, `BIOZ_test_rnom`, `BIOZ_test_rmod`, and `BIOZ_test_frequency`.
  - Prefer `ECG_samplingRate`, `ECG_gain`, `ECG_hpf`, and `ECG_lpf` for ECG reporting.

### Add Reduced Phase Scan Option

Reason for task:
- Current BIOZ scan uses all hardware-supported phase settings at each frequency:
  - `128 kHz`: 4 phase points
  - `80 kHz`: 8 phase points
  - `40 kHz` and below: 16 phase points
- This is conservative and robust, but each phase transition adds settling time.
- We want a configurable reduced-phase mode that keeps scan quality acceptable while reducing elapsed scan time.
- External/tissue fitting is still under investigation.
  - Current production code uses cosine fit for external scans and triangular fit for internal BIST.
  - Because MAX30001G BIOZ uses switched/square-wave style excitation and synchronous demodulation, external known-load validation must decide whether cosine remains adequate or whether a triangular or other model is more accurate for production.
  - Do not change the external fit model in this task; this task is about configurable phase-count reduction and validation infrastructure.

Implementation plan:
- [x] Add a phase-range/phase-density field to `BIOZScanConfig`.
  - Recommended shape:
    - `BIOZ_SCAN_PHASE_FULL`: use all hardware-supported phase points
    - `BIOZ_SCAN_PHASE_REDUCED`: use 4 phase points at every scanned frequency
  - Keep the default as `FULL` so existing behavior remains unchanged unless explicitly requested.
- [x] Add the new enum/field to the public scan configuration API in `src/max30001g_typedefs.h` and document it in `src/max30001g.h`.
- [x] Update `setupBIOZScan(...)` sanitization so the new phase-range option is validated and stored in `_scanRuntimeConfig`.
- [x] Update `biozScanPhaseCountForFreq(...)` so phase count depends on the selected phase-range mode:
  - `FULL`:
    - frequency index `0` (`128 kHz`) -> 4 points
    - frequency index `1` (`80 kHz`) -> 8 points
    - remaining frequencies -> 16 points
  - `REDUCED`:
    - all frequencies -> 4 points
- [x] Keep reduced-phase selection frequency-aware rather than blindly taking the first 4 selector codes.
  - At `128 kHz`, the hardware already only supports 4 points.
  - At `80 kHz`, reduced mode should still use 45 degree increments by stepping the hardware selector in 22.5 degree units.
  - At `40 kHz` and below, reduced mode should use 45 degree increments by stepping the hardware selector in 11.25 degree units.
  - Therefore `REDUCED` should always target the same actual phase set: `0`, `45`, `90`, `135` degrees.
    - `128 kHz`: selectors `0,1,2,3`
    - `80 kHz`: selectors `0,2,4,6`
    - `40 kHz` and below: selectors `0,4,8,12`
- [x] Add a helper for reduced-phase selector mapping so `stepBIOZScan()` can still iterate phase index `0..N-1` while translating to the correct hardware selector values.
  - Do not overload `_scanPhaseIndex` with raw hardware selector meaning if reduced mode skips selector codes.
- [x] Ensure the scan stores the actual applied `BIOZ_phase` values in `_scanPhaseDeg[][]`, not just the logical reduced-phase index.
- [x] Keep both reduced and full modes compatible with:
  - internal BIST triangular fit
  - external scan fit path
  - current auto-ranging
  - sample-count settling
  - scan result publication through `BIOZ_spectrum`

Validation plan:
- [x] Add a dedicated fast reduced-phase internal validation sketch:
  - `examples/BIOZScan_Internal_Fast/BIOZScan_Internal_Fast.ino`
- [x] `BIOZScan_Internal_Fast.ino` should:
  - use `BIOZScanConfig`
  - set `fast = true`
  - set reduced phase range
  - otherwise use the same BIOZ scan configuration as the full-phase internal validation sketch
  - use the standard full validation frequency span already used by the full-phase internal validation sketch
  - use internal resistor mode with `internal_resistor_ohm = 1000`
  - use the current validation baseline for settling unless later characterization proves a faster baseline is acceptable
  - print final `frequency_hz,magnitude_ohm,phase_deg`
- [x] `BIOZScan_Internal_Fast.ino` should also report total elapsed time to build one spectrum.
- [x] Keep `BIOZScan_Internal.ino` as the full-phase validation sketch unless explicitly renamed or redefined later.
- [x] Update `BIOZScan_Internal.ino` to report total elapsed time to build one full-phase spectrum so reduced vs full timing can be compared directly.
- [x] Compare reduced-phase and full-phase internal-BIST results:
  - same hardware
  - same resistor
  - same current start
  - same averaging
  - same frequency range
  - same settling defaults
- [x] Record for each frequency:
  - fitted magnitude difference between full and reduced phase scan
  - fitted phase difference between full and reduced phase scan
  - total elapsed scan time
- [x] Record repeated-run variability across separate validation runs.
- [ ] Decide whether reduced-phase mode is acceptable for internal regression if:
  - magnitude error versus full-phase baseline stays within a defined target
  - fitted phase remains stable enough for the intended use
  - scan time reduction is meaningful
- [ ] After internal validation, repeat the same comparison with external known loads once the fixture is available.
  - Test pure resistors first.
  - Then test RC loads with measurable reactance.
  - Use that dataset to decide whether external scans can safely use reduced phase count in production.

Questions this task should answer:
- [ ] Is 4-phase reduced scan sufficient for accurate internal-BIST magnitude regression?
- [ ] Is 4-phase reduced scan sufficient for external known-load magnitude and phase extraction?
- [x] Does reduced-phase mode materially reduce elapsed scan time after accounting for phase-change settling?
- [ ] Does reduced-phase mode interact badly with current auto-ranging at any frequency?
- [ ] Does external fitting still behave correctly with reduced phase density, or does it become too sensitive to model mismatch/noise?

Expected output of this task:
- [x] Public `BIOZScanConfig` option for `FULL` vs `REDUCED` phase range
- [x] Driver support for reduced-phase scan orchestration
- [x] `BIOZScan_Internal_Fast.ino` example sketch
- [x] Validation notes comparing reduced-phase and full-phase internal scans
- [ ] A go/no-go decision for using reduced phase count in later external-load calibration tests

Validation results:

Timing summary:

| Mode | Scan time (ms) | Scan time (s) | Relative to fast reduced |
|---|---:|---:|---:|
| Fast reduced | 9893 | 9.9 | 1.0x |
| Slow reduced | 19706 | 19.7 | 2.0x |
| Fast full | 39167 | 39.2 | 4.0x |
| Slow full | 78041 | 78.0 | 7.9x |

Observed timing behavior:
- Slow sampling versus fast sampling is almost exactly a factor of `2`.
- Full phase range versus reduced phase range is almost exactly a factor of `4`.
- Fast sampling with reduced phase range completes one internal spectrum in about `10 s`.
- The timing ratios match the expected phase-point counts:
  - reduced scan: `4` phase points per frequency
  - full scan: up to `16` phase points per frequency

Raw result logs:

Fast reduced:

```text
frequency_hz,magnitude_ohm,phase_deg
18204.0,903.671,0.250
8192.0,902.111,0.500
4096.0,901.310,1.000
2048.0,897.851,2.000
1024.0,882.811,3.750
spectrum_build_time_ms,9893
BIOZ internal-resistor fast reduced-phase scan complete in 9894 ms.
```

Slow reduced:

```text
frequency_hz,magnitude_ohm,phase_deg
18204.0,904.818,0.250
8192.0,903.348,0.500
4096.0,902.412,1.000
2048.0,898.534,2.000
1024.0,884.000,3.750
spectrum_build_time_ms,19706
BIOZ internal-resistor slow reduced-phase scan complete in 19706 ms.
```

Fast full:

```text
frequency_hz,magnitude_ohm,phase_deg
18204.0,904.667,0.250
8192.0,902.939,0.500
4096.0,901.060,0.750
2048.0,898.473,1.750
1024.0,885.150,3.250
spectrum_build_time_ms,39167
BIOZ internal-resistor fast full-phase scan complete in 39168 ms.
```

Slow full:

```text
frequency_hz,magnitude_ohm,phase_deg
18204.0,905.751,0.250
8192.0,903.852,0.500
4096.0,902.153,0.750
2048.0,899.501,1.750
1024.0,886.270,3.250
spectrum_build_time_ms,78041
BIOZ internal-resistor slow full-phase scan complete in 78041 ms.
```

Accuracy observations from this internal `1 kOhm` BIST dataset:
- Fast sampling produces slightly lower fitted magnitude than slow sampling, by about `0.1%`.
- Reduced phase range produces a slightly higher fitted phase than full phase range, typically about `0.25` to `0.5 deg` at the lower frequencies in this dataset.
- Reduced phase range changes fitted magnitude only slightly relative to full phase range, roughly `+0.03%` to `-0.26%` in these measurements.
- For internal BIST regression, reduced phase range appears promising because it preserves magnitude closely while reducing scan time by about `4x`.
- This does not yet establish that reduced phase range is acceptable for external known-load or tissue measurements; that still requires fixture-based validation.

Repeatability observations from four repeated `fast reduced` internal `1 kOhm` BIST runs:

| Frequency (Hz) | Mean magnitude (ohm) | Min-max spread (ohm) | Spread (%) |
|---|---:|---:|---:|
| 18204 | 903.519 | 0.046 | 0.005 |
| 8192 | 902.120 | 0.117 | 0.013 |
| 4096 | 901.130 | 0.117 | 0.013 |
| 2048 | 897.854 | 0.276 | 0.031 |
| 1024 | 882.600 | 0.060 | 0.007 |

Repeatability conclusions:
- The repeated `fast reduced` runs are highly stable for internal BIST.
- Fitted phase was identical across all four runs at every test frequency: `0.25`, `0.5`, `1.0`, `2.0`, and `3.75 deg`.
- Spectrum build time was also stable at `9893` to `9894 ms`.
- The largest observed run-to-run magnitude spread was `0.276 ohm` at `2048 Hz`, which is about `0.031%`.
- These repeated-run results suggest the earlier differences between `fast` versus `slow` and `reduced` versus `full` are dominated by systematic configuration effects, not random run-to-run noise.

### BIOZ Calibration Program Task
- [ ] Create a dedicated BIOZ calibration utility program before subject/sample measurements.
- [ ] Calibration utility should support:
  - internal 1 kOhm BIST resistor phase/frequency sweep,
  - board external 100 ohm resistor check,
  - optional external known resistor or RC load,
  - CSV output with frequency, phase, raw value, converted impedance, status, and PLL state,
  - fitted scale and phase correction factors by frequency,
  - saved calibration constants or a printed calibration table for later entry.
- [ ] Decide how calibration constants should be represented in the driver:
  - global scalar scale factor,
  - per-frequency scale table,
  - per-frequency complex correction,
  - separate internal-BIST validation path and external-load calibration path.

### External Calibration Resistor
Current board external test resistor: `100 ohm`.

Recommendation:
- Keep the 100 ohm resistor for now. It is useful as a low-impedance external path check and should help validate board routing, current drive, and ADC scaling.
- Do not replace it based only on the internal BIST result.
- For a future board revision or external calibration fixture, add multiple known loads instead of relying on a single resistor:
  - `100 ohm` for low impedance,
  - `1 kOhm` to match the internal BIST scale,
  - `10 kOhm` or higher for high-impedance range,
  - at least one known RC load to validate reactance/phase behavior.
- For tissue/sample work, a calibration fixture with selectable resistor and RC standards will be more useful than changing the single onboard 100 ohm resistor.

### External Impedance Test Fixture
Build either individual plug-in loads or a small selectable test PCB. A selectable PCB is preferred for repeatability, but it must connect exactly one load at a time and keep unused loads electrically floating so parallel leakage and capacitance do not corrupt the selected standard.

Recommended first load set:

| ID | Topology | Values | Purpose |
|---|---|---|---|
| R0 | short | 0 ohm jumper | Residual wiring/contact offset check |
| R100 | resistor | 100 ohm | Low-impedance board/external path check |
| R330 | resistor | 330 ohm | Low tissue / good-contact range |
| R1K | resistor | 1 kOhm | Matches internal BIST nominal target |
| R3K3 | resistor | 3.3 kOhm | Wet electrode / moderate tissue path |
| R10K | resistor | 10 kOhm | Common skin/electrode test point |
| R33K | resistor | 33 kOhm | Higher contact impedance |
| R100K | resistor | 100 kOhm | Dry/contact-stress test |
| R330K | resistor | 330 kOhm | Difficult dry electrode range |
| R1M | resistor | 1 Mohm | Upper range / low-current auto-ranging test |
| P1 | parallel RC | `10 kOhm \|\| 100 nF` | Low-frequency corner, wet/contact model |
| P2 | parallel RC | `10 kOhm \|\| 10 nF` | Strong frequency dependence in scan range |
| P3 | parallel RC | `10 kOhm \|\| 1 nF` | Mid/high-frequency capacitive behavior |
| P4 | parallel RC | `100 kOhm \|\| 10 nF` | Higher impedance wet/contact model |
| P5 | parallel RC | `100 kOhm \|\| 1 nF` | Higher impedance frequency-dependent model |
| P6 | parallel RC | `1 Mohm \|\| 220 pF` | Dry/thin-film electrode-like model |
| S1 | series RC | 1 kOhm + 100 nF | Reactive load with corner near 1.6 kHz |
| S2 | series RC | 1 kOhm + 10 nF | Reactive load with corner near 15.9 kHz |
| S3 | series RC | 10 kOhm + 10 nF | Higher impedance reactive load near 1.6 kHz |
| S4 | series RC | 10 kOhm + 1 nF | Higher impedance reactive load near 15.9 kHz |
| S5 | series RC | 100 kOhm + 1 nF | High-impedance dry/contact-like reactive load |

Minimal first build if time is limited:
- Pure resistors: `100 ohm`, `1 kOhm`, `10 kOhm`, `100 kOhm`, `1 Mohm`.
- Parallel RC: `10 kOhm || 10 nF`, `100 kOhm || 1 nF`, `1 Mohm || 220 pF`.
- Series RC: `1 kOhm + 100 nF`, `10 kOhm + 10 nF`, `100 kOhm + 1 nF`.

Fixture layout recommendation:
- Use four front-panel/board nodes: `DRVP`, `DRVN`, `BIP`, and `BIN`.
- Provide a 2-wire mode where `DRVP` ties to `BIP` and `DRVN` ties to `BIN` at the fixture, then the selected load sits between the two combined nodes.
- Provide a 4-wire/Kelvin mode where `DRVP`/`DRVN` drive the outer load pads and `BIP`/`BIN` sense directly at the load pads.
- Use one dual-pole selector per load, or a 2xN header with shunts that connects both sides of only one load. Avoid a single-pole selector because the unselected load side can remain capacitively or resistively coupled.
- Put the selected load footprints close to the selector and sense pads. Keep loop area small, especially for the pF/nF RC loads.
- Add a clearly labeled `OPEN` position and a `SHORT` position.
- Use 1% metal-film resistors or better. Use C0G/NP0 capacitors for pF/nF values when practical; use film capacitors for larger nF values if available. Avoid electrolytic capacitors.
- Label each load with nominal R/C and measured R/C values if measured with a DMM/LCR meter.
- For the first revision, prefer manual jumpers over analog switches or relays. Manual jumpers add less uncertainty and make it obvious which load is connected.
- Need to clean board after soldering and consider guards spacing for 100k, 330k 1M and pf loads.

### External Known-Impedance Validation Task
Create a dedicated test program and test process for known external impedance models before relying on tissue/sample scans.

- [ ] Create an external impedance validation sketch or mode separate from the internal BIST validation path.
- [ ] Validate pure resistor loads:
  - onboard `100 ohm` test resistor,
  - external resistor standards from the impedance test fixture.
- [ ] Validate capacitor/reactive loads:
  - known capacitor in series with a resistor from the fixture,
  - known capacitor in parallel with a resistor from the fixture,
  - choose values that produce measurable reactance in the MAX30001G frequency range.
- [ ] For each load, compute expected complex impedance by frequency:
  - resistor: `Z = R`,
  - capacitor: `Z_C = 1 / (j 2 pi f C)`,
  - RC combinations from series/parallel equations.
- [ ] Print measured and expected values in CSV:
  - frequency,
  - phase offset,
  - raw ADC,
  - fitted magnitude,
  - fitted phase,
  - expected magnitude,
  - expected phase,
  - percent magnitude error,
  - phase error,
  - status and PLL state.
- [ ] Use these external-load results to decide the production fit model.
  - Verify whether `fitImpedanceCosine()` is correct for external resistor/RC loads.
  - Decide whether `fitImpedanceTriangular()` should remain internal-BIST-only, move to a validation-only output path, or be replaced by another internal-BIST summary such as max-absolute response plus scale correction.
- [ ] Use known-load results to design the final calibration process for tissue and sample measurements.

## Phase 2: Subject-Attached Tests
Proceed only after all Phase 1 tests pass.

### Test 2.1: ECG Acquisition
Primary sketch: `ECG.ino`

Interactive alternative:
```text
m1
Es1
Eg2
El3
Er1
.
z
```

Pass criteria:
- [ ] ECG waveform is visible
- [ ] QRS complexes are identifiable
- [ ] RR intervals are reported when enabled

Result: [ ] PASS  [ ] FAIL
Notes: ___________________________________________

### Test 2.2: BIOZ Impedance Acquisition
Primary sketch: `BIOZ.ino`

Interactive alternative:
```text
m2
Bs0
Bg1
Bf8000
Bc8000
Bw2
.
z
```

Pass criteria:
- [ ] BIOZ impedance stream is present
- [ ] Values are stable for a fixed setup
- [ ] Lead configuration behaves as expected

Result: [ ] PASS  [ ] FAIL
Notes: ___________________________________________

### Test 2.3: Combined ECG + BIOZ
Primary sketch: `ECGandBIOZ.ino`

Interactive alternative:
```text
m3
Es1
Eg2
Bs0
Bg1
Bf8000
Bc8000
.
z
```

Pass criteria:
- [ ] ECG data is present
- [ ] BIOZ data is present
- [ ] Combined mode runs without FIFO faults
- [ ] RR output is available when enabled

Result: [ ] PASS  [ ] FAIL
Notes: ___________________________________________

### Test 2.4: External BIOZ Scan
Primary sketch: `BIOZScan.ino`

Interactive alternative:
```text
m8
Si0
Sa2
Sf0
Sr0
.
```

Pass criteria:
- [ ] Scan completes
- [ ] Sweep output is printed
- [ ] Results are consistent for the attached load

Result: [ ] PASS  [ ] FAIL
Notes: ___________________________________________

## Interactive Test Program Coverage
`MAX30001G.ino` should be used as the consolidated manual test utility. It is expected to support:
- SPI/register health check
- ECG mode
- BIOZ impedance mode
- ECG + BIOZ combined mode
- ECG internal calibration
- BIOZ internal signal calibration
- BIOZ internal impedance calibration
- BIOZ scan with external electrodes
- BIOZ scan with internal resistor

## Final Sign-Off
- [ ] Phase 1 bench tests completed without subject attached
- [ ] Phase 2 subject-attached tests completed
- [ ] Command-line interactive test verified
- [ ] Any failures documented with register dump and serial log
