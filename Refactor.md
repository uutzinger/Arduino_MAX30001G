# Refactor Plan

## Desired Driver Layers
========================

### Layer 1: Low-Level Register Setters

Keep existing low-level functions focused on one register group:

- BIOZ sampling rate
- BIOZ gain
- BIOZ filters
- BIOZ modulation frequency
- BIOZ drive current
- BIOZ modulation mode
- BIOZ phase offset
- BIOZ test impedance path
- interrupt/FIFO control

These functions should not own profile lifecycle or sample acquisition.

### Layer 2: Profile Setup Functions

Keep profile setup functions responsible for baseline mode selection:

- `setupECG(...)`
- `setupBIOZ(...)`
- `setupECGandBIOZ(...)`
- `setupBIOZImpedanceCalibration(...)`
- `setupBIOZExternalImpedanceCalibration(...)`
- `setupBIOZScan(...)`

These functions configure a coherent starting register baseline and set `_profile`, `_configured`, `_running`, drain flags, and lead settings intent.

### Layer 3: Point Acquisition Primitives

Add shared nonblocking point-acquisition primitives for scan use.

Suggested responsibilities:

- Clear point-local BIOZ state.
- Start discard window.
- Count discarded samples or FIFO threshold blocks.
- Collect averaged samples.
- Track raw range/validity flags.
- Return one point result when complete.

Important constraint:

- These primitives must not call `setupBIOZ...()`, `start()`, `stop()`, or change `_profile`.
- They may use `servicePendingInterrupts()`, `serviceAllInterrupts()`, `readBIOZ_FIFO(...)`, `FIFOReset()`, and `synch()` because these are part of sample acquisition.

### Layer 4: Scan Orchestration

`setupBIOZScan(...)`, `startBIOZScan()`, and `stepBIOZScan()` should own:

- Frequency range.
- Current profile and retries.
- Internal versus external path selection.
- Frequency, current, modulation, phase ordering.
- Settling discard count.
- Averaging.
- Raw-range evaluation.
- Fit model selection.
- Spectrum publication.

## Current Architecture Check
=============================

### Continuous ECG

- `setupECG(...)` selects `PROFILE_ECG`.
- It configures ECG registers, R-to-R, interrupt behavior, FIFO thresholds, and lead settings intent.
- `start()` enables ECG/R-to-R, applies lead settings, resets FIFO, synchronizes, and marks `_running=true`.
- `update()` services interrupts and drains ECG FIFO/R-to-R when configured.

Status: keep this architecture. No BIOZ scan refactor should change this path.

### Continuous BIOZ

Current flow:

- `setupBIOZ(...)` selects `PROFILE_BIOZ`.
- It configures the normal external BIOZ path at one frequency/current/phase.
- It disables internal test impedance and R-to-R.
- `start()` enables BIOZ, applies lead settings, resets FIFO, synchronizes, and marks `_running=true`.
- `update()` services interrupts and drains BIOZ FIFO when configured.

Status: keep this architecture. Continuous fixed-frequency BIOZ should continue to use `setupBIOZ(...)`, `start()`, and repeated `update()`.

### Continuous ECG and BIOZ

Current flow:

- `setupECGandBIOZ(...)` selects `PROFILE_ECG_AND_BIOZ`.
- It configures ECG, BIOZ, R-to-R, shared interrupts, FIFO thresholds, and lead settings intent.
- `start()` enables ECG/BIOZ/R-to-R, applies lead settings, resets FIFO, synchronizes, and marks `_running=true`.
- `update()` services interrupts and drains ECG, BIOZ, and R-to-R as configured.

Status: keep this architecture. BIOZ scan should not be folded into the combined continuous profile.

## Refactor BIOZ scanning
=========================

Refactor BIOZ scanning so internal BIST and external specimen scans share the same scan-owned acquisition lifecycle:

- Configure scan baseline once.
- Change only the required register group for each frequency/current/phase transition.
- Reset/sync and discard a sample-count-based settling window after each transition.
- Average settled samples in the scan state machine.
- Fit the frequency result after all phase points are collected.

The scan state machine must remain nonblocking and must be driven by `MAX30001G::update()`.

### BIOZ Scan Entry Points

Current flow:

- `setupBIOZScan(...)` selects `PROFILE_BIOZ_SCAN`.
- `start()` checks for `PROFILE_BIOZ_SCAN`; when present, it calls `startBIOZScan()` and returns.
- `update()` checks for `PROFILE_BIOZ_SCAN`; when present, it calls `stepBIOZScan()` and returns scan progress.
- `startBIOZScan()` initializes scan runtime state and marks `_running=true`.
- `stepBIOZScan()` owns the frequency/current/phase scan sequence.

This is intentionally the same lifecycle concept used by the continuous profiles:

- `setup...()` chooses a profile and configures the register baseline.
- `start()` begins that configured profile.
- `update()` services that configured profile.
- `stop()` stops the active profile.

The only difference is that BIOZ scan has its own start/update workers because scan acquisition is algorithmic and multi-step. Continuous profiles use the common `start()` enable/reset/sync path and common `update()` FIFO-drain path.

Status: keep `setupBIOZScan(...)`, `startBIOZScan()`, and the `update()` branch model.

Pre-refactor problem in `PROFILE_BIOZ_SCAN`:

- Internal BIST called `measureBIOZInternalImpedancePoint(...)` from `BIOZ_SCAN_CONFIG_PHASE`.
- That helper stopped/reconfigured/started/acquired/restored internally, so part of the scan state machine was hidden inside a blocking point helper.
- External scan used explicit discard/wait/process states.
- This meant internal and external scans did not share one acquisition lifecycle.


### Proposed BIOZ Scan State Machine

Replace the current hidden internal point measurement path with explicit scan states.

Recommended state sequence:

1. `BIOZ_SCAN_INIT`
   - Reset AFE.
   - Configure scan baseline: sample rate, gain, filters, internal/external path, interrupts, lead settings.
   - Enable BIOZ.
   - Reset FIFO and synchronize.
   - Initialize frequency/current/result arrays.

2. `BIOZ_SCAN_CONFIG_FREQ`
   - Select next frequency.
   - Write frequency register only.
   - Store actual selected `BIOZ_frequency`.
   - Determine number of phase offsets.
   - Move to current setup.

3. `BIOZ_SCAN_CONFIG_CURRENT`
   - Write drive current.
   - Write modulation mode after current is known.
   - Mark whether current changed for this attempt.
   - Reset per-attempt range flags.

4. `BIOZ_SCAN_CONFIG_PHASE`
   - Write phase register only.
   - Store actual selected `BIOZ_phase`.
   - Do not acquire samples in this state.

5. `BIOZ_SCAN_BEGIN_POINT`
   - Clear BIOZ ring buffer and status flags.
   - `FIFOReset()`.
   - `synch()`.
   - Choose discard count:
     - `settle_samples` for frequency/phase changes.
     - `current_change_settle_samples` after current changes.
   - The current validated baseline is 24 discarded samples, with averaging after that window.

6. `BIOZ_SCAN_DISCARD_POINT`
   - Service interrupts or poll status.
   - Read and discard BIOZ samples until the configured discard sample count is reached.
   - Reset/sync between FIFO threshold blocks only if validation shows it remains necessary.

7. `BIOZ_SCAN_COLLECT_POINT`
   - Service interrupts or poll status.
   - Read BIOZ samples until `avg` settled samples are collected.
   - Keep raw samples or robust mean input.

8. `BIOZ_SCAN_PROCESS_POINT`
   - Compute robust mean.
   - Convert raw ADC units to ohms for external scans.
   - For internal BIST, either collect already-scaled ohms through the same conversion path or explicitly apply the internal calibration scaling in one shared place.
   - Update raw range flags for current auto-ranging.
   - Store `_scanImpedance[freq][phase]`.

9. `BIOZ_SCAN_NEXT_PHASE`
   - Advance phase index.
   - Return to `BIOZ_SCAN_CONFIG_PHASE` or move to attempt evaluation.

10. `BIOZ_SCAN_EVALUATE_ATTEMPT`
    - Decide whether current is acceptable.
    - Retry current if needed and retry budget remains.
    - Otherwise accept best effort and continue.

11. `BIOZ_SCAN_FINALIZE_FREQ`
    - Fit internal scans with triangular model.
    - Fit external scans with cosine model.
    - Store frequency, magnitude, phase, and final current.

12. `BIOZ_SCAN_FINISH`
    - Push `ImpedanceSpectrum`.
    - Stop BIOZ.
    - Mark scan complete.

## Implementation Checklist
===========================
- [x] Verify `setupECG(...)`, `setupBIOZ(...)`, and `setupECGandBIOZ(...)` still have correct profile flags, drain flags, lead settings intent, and start/update behavior.

- [x] Verify `start(...)`, `stop(...)`, `update(...)` correctly handle continuous ECG, BIOZ, ECG&BIOZ acquisition.

- [x] Verify `start()` branches to `startBIOZScan()` only for `PROFILE_BIOZ_SCAN`; all continuous profiles use the common enable/lead-settings/FIFO-reset/sync flow.

- [x] Verify `update()` branches to `stepBIOZScan()` only for `PROFILE_BIOZ_SCAN`; all continuous profiles use the common interrupt-service and FIFO-drain flow.

- [x] Verify `setupBIOZScan(...)` only configures scan profile/runtime data and does not begin acquisition.

- [x] Verify `startBIOZScan()` only starts the scan state machine and does not configure hardware beyond scan state initialization.

- [x] Refactor `stepBIOZScan()` so the scan state machine explicitly owns the full internal and external point-acquisition lifecycle.
Internal-resistor scan now completes the same logical work that used to be hidden inside `measureBIOZInternalImpedancePoint(...)`, but does it through scan states owned by `stepBIOZScan()`.

- [x] Add explicit scan states for current setup, point begin, discard, collect, process, and next phase.

- [x] Remove the call to `measureBIOZInternalImpedancePoint(...)` from `BIOZ_SCAN_CONFIG_PHASE`.

- [x] Remove scan-time dependence on `measureBIOZInternalImpedancePoint(...)`; after refactor the helper is no longer part of BIOZ scan execution.

- [x] Keep frequency -> current -> modulation -> phase ordering.

- [x] Move internal BIST acquisition onto the same scan-owned discard/collect/process flow as external scans.

- [x] Preserve 24-sample settling as the validated baseline until repeatable data supports reducing it.

- [x] Keep internal and external paths separate only where the hardware path truly differs.
Differences should be limited to internal/external BMUX or test-impedance selection, internal triangular fit versus external cosine fit, and an internal-specific restart only if validation proves it is required.

- [x] Keep continuous fixed-frequency BIOZ behavior unchanged.

- [ ] Validate whether AFE stop/re-enable at scan point boundaries is required for external BIOZ scans.
Current code only restarts AFE for internal BIST. This is an assumption carried forward from internal-resistor validation, not proof that external scans never need it. Test external known loads with and without point-boundary restart before optimizing this behavior.

- [x] Remove the legacy blocking helper after scan refactor.
`measureBIOZInternalImpedancePoint(...)` has been removed from the driver. `BIOZ_Internal_ImpedanceCalibration.ino` and `BIOZ_Internal_ImpedanceSweep.ino` now demonstrate internal BIST measurements through explicit setup/start/update/FIFO collection paths, so scan behavior is no longer hidden in a blocking helper.

- [x] Split internal impedance examples after the driver scan path stabilized.
`BIOZ_Internal_ImpedanceCalibration.ino` is now the small two-point continuous-path example. `BIOZ_Internal_ImpedanceSweep.ino` preserves the previous frequency/phase sweep use case.

## Verification Notes
=====================

### 2026-04-25: Continuous setup/start/update review

Reviewed continuous acquisition paths before changing BIOZ scan:

- `setupECG(...)`
  - selects `PROFILE_ECG`
  - enables ECG and R-to-R intent
  - drains ECG FIFO and reads R-to-R in `update()`
  - disables BIOZ intent
  - leaves AFE enable, lead settings, FIFO reset, and sync to `start()`

- `setupBIOZ(...)`
  - selects `PROFILE_BIOZ`
  - enables BIOZ intent only
  - drains BIOZ FIFO in `update()`
  - disables ECG, R-to-R, calibration, and internal BIOZ test impedance
  - leaves AFE enable, lead settings, FIFO reset, and sync to `start()`

- `setupECGandBIOZ(...)`
  - selects `PROFILE_ECG_AND_BIOZ`
  - enables ECG, BIOZ, and R-to-R intent
  - drains ECG and BIOZ FIFO and reads R-to-R in `update()`
  - disables calibration and internal BIOZ test impedance
  - leaves AFE enable, lead settings, FIFO reset, and sync to `start()`

- `start()`
  - for scan only: calls `startBIOZScan()` and returns
  - for continuous profiles: enables `_useECG`, `_useBIOZ`, `_useRTOR`, applies lead settings, resets FIFO, synchronizes, and sets `_running=true`

- `stop()`
  - disables ECG, BIOZ, and R-to-R for all profiles
  - clears interrupt/status bookkeeping
  - sets `_running=false`

- `update()`
  - returns immediately if `_running=false`
  - for scan only: calls `stepBIOZScan()`
  - for continuous profiles: services interrupts or fallback-polls status, drains enabled ECG/BIOZ FIFOs, reads R-to-R when enabled, and resets FIFO after overflow flags

Conclusion: continuous ECG, continuous BIOZ, and continuous ECG+BIOZ follow the intended setup/start/update architecture. BIOZ scan can be refactored independently as long as the common continuous path remains unchanged.

### 2026-04-25: BIOZ scan entry-point review

Reviewed scan lifecycle entry points before changing the scan state machine:

- `start()`
  - resets common runtime flags first
  - branches to `startBIOZScan()` only when `_profile == PROFILE_BIOZ_SCAN`
  - otherwise uses the common continuous start path: `enableAFE(...)`, apply lead settings, `FIFOReset()`, `synch()`, `_running=true`

- `update()`
  - returns immediately if `_running == false`
  - branches to `stepBIOZScan()` only when `_profile == PROFILE_BIOZ_SCAN`
  - otherwise uses the common continuous service path: interrupt service or fallback polling, enabled FIFO drains, optional `readRTOR()`, overflow-triggered `FIFOReset()`

- `setupBIOZScan(...)`
  - sanitizes the runtime scan configuration
  - sets `_profile = PROFILE_BIOZ_SCAN`
  - disables continuous drain flags and start-time lead-setting behavior
  - initializes scan runtime members to idle defaults
  - does not enable AFE, reset FIFO, synchronize, or begin acquisition

- `startBIOZScan()`
  - checks that the configured profile is `PROFILE_BIOZ_SCAN`
  - sets `_BIOZScanState = BIOZ_SCAN_INIT`
  - marks `_scanInProgress = true`, `_scanCompleted = false`, `_running = true`
  - does not configure hardware directly; hardware configuration begins when `stepBIOZScan()` runs the `BIOZ_SCAN_INIT` state

Conclusion: the current scan lifecycle entry points already follow the intended setup/start/update split. The remaining refactor work belongs inside `stepBIOZScan()`, not in `start()`, `update()`, `setupBIOZScan(...)`, or `startBIOZScan()`.

### 2026-04-25: BIOZ scan state-machine refactor

Refactored `stepBIOZScan()` so scan execution no longer calls `measureBIOZInternalImpedancePoint(...)`.

- Added explicit scan states for current setup, point begin, sample discard, sample collection, point processing, and phase advance.
- Internal and external scans now share the same scan-owned discard/collect/process path.
- FIFO reads use raw BIOZ samples for current range evaluation, then convert raw mean to ohms before impedance fitting.
- Internal BIST still uses the triangular fit; external scan still uses the cosine fit.
- Internal BIST keeps an internal-only AFE restart at the point boundary for now because prior internal-resistor validation showed sensitivity to running reconfiguration. External scans still need known-load validation to determine whether the same restart is required there.
- The validated sample-settling baseline remains `settle_samples = 24`; this refactor does not reduce settling requirements.
- Continuous fixed-frequency BIOZ setup/start/update behavior was not changed.

Conclusion: BIOZ scan execution is now state-machine-owned. The helper remains available for the sweep characterization example, but it is no longer part of scan execution or the small two-point calibration example.

## Validation Plan
===================
1. Build examples after each mechanical refactor step.
2. Run `BIOZ_Internal_ImpedanceCalibration` after any change to the continuous internal BIST setup/start/update path.
3. Run `BIOZ_Internal_ImpedanceSweep` after any change to internal BIST phase/frequency characterization or the blocking helper.
4. Run `BIOZScan_Internal` after replacing the hidden helper-based internal scan path with scan-owned states.
5. Confirm internal 1 kOhm BIST remains close to the known baseline:
   - 18.204 kHz: about 893 ohm
   - 8.192 kHz: about 891 ohm
   - 4.096 kHz: about 889 ohm
   - 2.048 kHz: about 886 ohm
   - 1.024 kHz: about 873 ohm
5. Run `BIOZ_SettlingCharacterization` only after the scan path is correct.
6. Run external known-load validation before claiming external scan accuracy.
7. Compare external known-load scans with internal-only restart versus restart for both internal and external paths.

## Later Optimization Work
==========================
- Avoid stop/start around phase-only transitions unless hardware validation requires it.
- Avoid rewriting unchanged filter/gain/path registers between points.
- Tune discard count below 24 samples only after before/after data proves the first stable sample arrives earlier.
- Keep calibration and sweep examples on explicit setup/start/update/FIFO collection unless a future use case proves a shared blocking helper is worth reintroducing.
