# External BioZ Test Process

## Purpose

Use the external impedance fixture to validate MAX30001G external BIOZ measurements and BIOZ scan behavior against known resistor and RC loads.

This process uses the merged interactive sketch:

```text
examples/MAX30001G/MAX30001G.ino
```

Continue updating `Validation.md` with recorded results, pass/fail decisions, and any driver changes that follow from the measurements. Use this file for the practical measurement setup and run sequence.

## Initial Approach

Start with a staged process rather than full automation.

First verify the simplest external path:

1. Set the external fixture to `100 ohm`.
2. Run the fixed-frequency external BIOZ mode.
3. Confirm stable readings, no timeout, PLL OK, and no range faults.
4. Then run an external BIOZ scan on the same `100 ohm` load.
5. Only after that works, use a Python host script to automate selected fixture measurements.

This avoids producing a large automated dataset before the fixture wiring, current drive, ADC scaling, serial output, and scan path are known to work.

## Manual Bring-Up: 100 Ohm Fixed BIOZ

Fixture:

```text
Load: 100 ohm
Mode: 2-wire unless explicitly testing 4-wire/Kelvin behavior
```

Serial setup:

```text
Baud: 115200
Sketch: examples/MAX30001G/MAX30001G.ino
```

Serial commands:

```text
h
m7
Bf8000
Bc8000
Bp0
.
z
```

Expected behavior:

- Health check passes.
- External BIOZ mode configures successfully.
- BIOZ output is stable.
- No timeout is reported.
- PLL status is OK.
- No persistent range or fault flags are reported.

Record in `Validation.md`:

- Fixture load ID and measured nominal value if available.
- Requested frequency.
- Actual/observed output value.
- Current setting.
- Phase setting.
- Status register.
- PLL state.
- Notes about stability, noise, or faults.

## Manual Bring-Up: 100 Ohm BIOZ Scan

After fixed-frequency external BIOZ works, stop the current measurement and run an external scan on the same `100 ohm` load.

Before starting the scan, upload a sketch version where scan mode does not drain `BIOZ_data` through the normal data display path. If using an older uploaded sketch, check `s`; if `Data Display` is `ON`, send `z` once to turn it off before starting the scan. The final scan spectrum is printed from `BIOZ_spectrum` regardless of the continuous data-display setting.

Serial commands:

```text
<
s
m8
Si0
Sa8
Sf1
Sr0
Sp1
St24
Sc24
Bc8000
a
.
```

Meaning:

- `m8`: BIOZ impedance spectroscopy mode.
- `Si0`: external electrodes/fixture source.
- `Sa8`: average 8 samples per phase point.
- `Sf1`: fast BIOZ sampling.
- `Sr0`: scan `128 kHz` down to `1 kHz`.
- `Sp1`: reduced phase range.
- `St24`: discard 24 settling samples after phase/frequency/filter changes.
- `Sc24`: discard 24 settling samples after current changes.
- `Bc8000`: start scan current at `8000 nA`.
- `a`: apply the displayed settings to the driver before starting.
- `.`: start measurement.

Expected scan output:

```text
BIOZ scan complete
frequency_hz,magnitude_ohm,phase_deg
...
```

Record in `Validation.md`:

- Full spectrum table.
- Whether the magnitude is close to `100 ohm`.
- Whether phase is close to the expected resistor phase.
- Any failed, missing, or suspicious frequency bins.
- Status/PLL output after the scan.

## 100 Ohm Results So Far

These results are from the first external fixture bring-up on 2026-07-15 and should be kept as context for the next diagnostics.

### Fixed External BIOZ: `m7`

Setup:

```text
m7
Bf8000
Bc8000
Bp0
.
z
```

Observed result:

- Stable external `100 ohm` reading.
- 34 reported samples ranged from `97.80` to `97.85 ohm`.
- Mean: `97.831 ohm`.
- Population standard deviation: `0.016 ohm`.
- Error versus nominal `100 ohm`: about `-2.17%`.

Assessment:

- The external fixture path, wiring, current drive, FIFO path, and basic Ohm conversion work for a simple `100 ohm` load.
- This is the current practical baseline for the external `100 ohm` load.

### First Scan Attempt: Data Display Interference

Setup:

```text
m8
Si0
Sa8
Sf1
Sr0
Sp1
St24
Sc24
Bc8000
.
```

Observed result:

- Scan completed, but all spectrum rows were `0.000,0.000`.
- Serial log showed per-phase BIOZ samples were present.
- Scan reported `scanBIOZ: no valid samples` and `using best-effort data`.

Cause:

- The merged interactive sketch had `Data Display: ON`.
- `displayData()` drained `BIOZ_data` during scan mode before the scan state machine processed the samples.

Corrective action:

- `MAX30001G.ino` was patched so scan mode does not drain `BIOZ_data`.
- Scan output should come from `BIOZ_spectrum`.

### Second Scan Attempt: Settings Not Applied

Observed result:

- Data display no longer drained scan-owned samples.
- Scan timed out before spectrum output.
- Displayed settings showed reduced phase mode, but the actual scan stepped through full phase density at lower frequency bins.

Cause:

- Displayed requested scan settings had not been applied to the already-configured driver profile before `start()`.

Corrective action:

- `MAX30001G.ino` was patched so `startMeasurement()` applies current settings before `afe.start()`.
- The manual command sequence now includes `a` before `.`.

### Third Scan Attempt: Completed But Low

Setup:

```text
<
m8
Si0
Sa8
Sf1
Sr0
Sp1
St24
Sc24
Bc8000
a
.
```

Observed spectrum:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,82.117,-0.796
81920.0,82.120,-0.291
40960.0,81.971,0.174
18204.0,81.656,0.479
8192.0,81.499,0.853
4096.0,81.371,1.669
2048.0,81.041,3.640
1024.0,80.341,1.940
```

Assessment:

- External scan now runs end-to-end.
- The `8192 Hz` scan result, `81.499 ohm`, is about `16..18%` low compared with the fixed external BIOZ baseline.
- Do not move to the full fixture set until this bias is understood.

### Scan-Like Continuous BIOZ: `m2`

Setup:

```text
<
m2
Bs1
Bg1
Ba1
Bd1
Bh0
Bf8000
Bc8000
Bp0
Bl0
a
.
z
```

Applied settings:

```text
64.0 sps, 20 V/V, AHPF 150 Hz, DLPF 4.10 Hz, DHPF bypass, 8192.0 Hz, 8000 nA, 0.00 deg
```

Observed result:

- Stable reading around `93.33 ohm`.

Assessment:

- The scan-like continuous setup is about `4.6%` below the `m7` external baseline, but still much higher than the scan result.
- The scan error is therefore not explained by gain/filter/current setup alone.
- Remaining likely causes are scan settling, per-phase scan acquisition behavior, current auto-ranging/retry behavior, or the external scan fit model.

### Conservative-Settling Scan

Completed on the `100 ohm` fixture load.

Purpose: determine whether the low scan result around `81..82 ohm` improves when scan settling is increased.

Fixture:

```text
Load: 100 ohm
Mode: 2-wire
```

Serial commands:

```text
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

Observed output:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,82.120,-0.801
81920.0,82.074,-0.243
40960.0,81.893,0.224
18204.0,81.662,0.480
8192.0,81.502,0.881
4096.0,81.407,1.686
2048.0,80.927,3.631
1024.0,80.310,1.880
```

Comparison at `8192 Hz`:

- `97.831 ohm`: prior `m7` fixed external BIOZ baseline.
- `93.33 ohm`: prior scan-like continuous `m2` baseline.
- `81.499 ohm`: prior reduced-phase scan result with `St24` and `Sc24`.
- `81.502 ohm`: conservative-settling scan result with `St48` and `Sc64`.

Assessment:

- Increasing global settling from `24/24` to `48/64` did not materially change the scan result.
- The dominant error is probably not the global scan settling count.

### Manual Reduced-Phase Continuous Points

Completed manual continuous BIOZ measurements at the four reduced-scan phase points. This test determines whether the low scan result comes from the external fit model or from scan-owned acquisition/current-ranging.

Base setup at `8192 Hz`:

```text
<
m2
Bs1
Bg1
Ba1
Bd1
Bh0
Bf8000
Bc8000
Bl0
Bp0
a
.
z
```

Recorded results:

```text
phase_deg,mean_ohm,min_ohm,max_ohm,std_ohm,n
0,93.295,93.22,93.34,0.021,360
45,46.998,46.93,47.05,0.021,280
90,0.124,0.07,0.20,0.021,288
135,-47.433,-47.49,-47.37,0.023,320
```

Interpretation:

- A no-offset cosine fit to these four manual phase points gives about `80.0 ohm`, close to the external scan result around `81.5 ohm`.
- The direct `0 deg` continuous value is about `93.3 ohm`.
- Therefore the low external scan magnitude is primarily a fit-model issue, not global settling and not basic continuous acquisition.
- The external resistor phase response is closer to triangular/linear behavior than sinusoidal/cosine behavior over the reduced phase points.

### External Scan Summary Model

Completed on the `100 ohm` external load with temporary scan diagnostics enabled. The normal spectrum output remains unchanged, but the driver also prints one diagnostic summary row for each external scan frequency.

Upload the current merged example:

```text
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn esp32:esp32:adafruit_feather_esp32s3 examples/MAX30001G
```

After upload completes, open the serial monitor at `115200 baud`, leave the fixture on the `100 ohm` external load, and wait for the sketch prompt/banner.

Diagnostic header:

```text
scan_external_summary,frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
```

Run sequence used:

```text
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

Observed output order:

```text
scan_external_summary,frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
scan_external_summary,...
...
BIOZ scan complete
frequency_hz,magnitude_ohm,phase_deg
...
```

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,82.118,-0.801,97.081,0.250,95.631,96000,4
81920.0,82.074,-0.244,96.647,0.250,95.904,96000,4
40960.0,81.888,0.221,96.100,0.250,95.928,96000,4
18204.0,81.662,0.479,95.658,0.250,95.675,96000,4
8192.0,81.535,0.831,95.579,0.500,95.476,80000,4
4096.0,81.373,1.694,95.395,1.000,95.121,32000,4
2048.0,81.372,3.369,95.388,2.000,94.970,16000,4
1024.0,80.288,1.905,94.609,1.500,93.558,8000,4
```

Normal spectrum result:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,82.118,-0.801
81920.0,82.074,-0.244
40960.0,81.888,0.221
18204.0,81.662,0.479
8192.0,81.535,0.831
4096.0,81.373,1.694
2048.0,81.372,3.369
1024.0,80.288,1.905
```

Comparison for the `100 ohm` external load at `8192 Hz`:

- normal spectrum/cosine fit from manual points: about `80.0..82.0 ohm`
- observed cosine scan result: `81.535 ohm`
- observed triangular scan result: `95.579 ohm`
- observed max-absolute phase value: `95.476 ohm`
- `m7` external baseline: about `97.8 ohm`

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- The triangular and max-absolute summaries are close to the fixed-frequency/continuous resistor baseline.
- The low external scan magnitude is mainly a cosine-model summary bias for this reduced-phase resistor scan, not missing scan samples.
- Auto-ranging requested `96000 nA` at high frequencies and was clamped by FCGEN limits at lower frequencies. This should be cleaned up later, but it did not prevent a stable diagnostic result.

### R1K External Scan Diagnostic

Completed on the fixture `1 kOhm` resistor. This checks whether triangular/max-absolute behavior scales correctly above the low-impedance `100 ohm` case and provides a bridge to the prior internal `1 kOhm` BIST work.

Fixture:

```text
Load: 1 kOhm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,798.688,-5.882,973.775,0.250,901.157,8000,4
81920.0,807.225,-3.444,970.796,0.250,925.787,8000,4
40960.0,813.897,-1.409,966.200,0.250,944.498,8000,4
18204.0,816.236,-0.227,961.055,0.250,952.734,8000,4
8192.0,817.341,0.618,959.647,0.500,955.898,8000,4
4096.0,817.510,1.699,958.375,1.000,956.455,8000,4
2048.0,816.295,3.720,954.116,2.000,954.150,8000,4
1024.0,812.134,2.814,949.991,1.500,950.135,8000,4
```

Normal spectrum result:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,798.688,-5.882
81920.0,807.225,-3.444
40960.0,813.897,-1.409
18204.0,816.236,-0.227
8192.0,817.341,0.618
4096.0,817.510,1.699
2048.0,816.295,3.720
1024.0,812.134,2.814
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- The same model split seen on `100 ohm` repeats at `1 kOhm`.
- At `8192 Hz`, cosine is `817.341 ohm`, triangular is `959.647 ohm`, and max-absolute is `955.898 ohm`.
- Triangular/max-absolute are about `4..5%` below nominal, while cosine is about `18%` below nominal.
- All bins used `8000 nA`; no current-limit warnings were present.

### R10K External Scan Diagnostic

Completed on the fixture `10 kOhm` resistor. This checks whether the summary-model behavior still scales when the load is closer to electrode/contact impedance levels.

Fixture:

```text
Load: 10 kOhm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,5432.697,-45.334,7574.339,-57.250,5509.320,8000,4
81920.0,6302.942,-34.019,8657.549,-52.250,6249.851,1100,4
40960.0,7253.381,-18.458,9389.433,-46.750,7376.779,1100,4
18204.0,7669.661,-7.901,9443.198,0.250,8521.990,1100,4
8192.0,7862.218,-2.728,9444.657,0.500,9025.097,1100,4
4096.0,7953.963,0.029,9442.645,1.000,9229.779,1100,4
2048.0,7989.309,2.874,9404.938,2.000,9307.298,1100,4
1024.0,7974.188,2.408,9358.850,1.500,9312.705,1100,4
```

Normal spectrum result:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,5432.697,-45.334
81920.0,6302.942,-34.019
40960.0,7253.381,-18.458
18204.0,7669.661,-7.901
8192.0,7862.218,-2.728
4096.0,7953.963,0.029
2048.0,7989.309,2.874
1024.0,7974.188,2.408
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- At `8192 Hz`, cosine is `7862.218 ohm`, triangular is `9444.657 ohm`, and max-absolute is `9025.097 ohm`.
- Triangular remains closest to nominal but is about `5.6%` low; max-absolute is about `9.7%` low; cosine is about `21.4%` low.
- High-frequency bins show strong magnitude loss and negative phase shift, especially `131072 Hz`.
- From `81920 Hz` down, auto-ranging selected `1100 nA` and the low-current path printed `BMUX_CG_MODE=0` warnings.

### R100K External Scan Diagnostic

Completed on the fixture `100 kOhm` resistor. This is a high-impedance stress test, not a clean calibration point across the full scan band.

Fixture:

```text
Load: 100 kOhm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,7020.149,-84.039,7481.817,-77.500,6767.197,1100,4
81920.0,11055.985,-80.342,12420.265,-75.000,10553.880,1100,4
40960.0,20979.338,-72.008,25873.686,-70.000,19295.898,1100,4
18204.0,40376.883,-55.262,55331.609,-61.750,40327.355,1100,4
8192.0,61266.023,-33.237,84382.844,-52.250,60325.074,660,4
4096.0,71171.281,-16.949,90997.695,-46.000,73075.461,440,4
2048.0,75093.234,-5.812,93392.312,1.750,83339.055,440,4
1024.0,76903.797,-2.066,93248.328,1.500,87876.000,440,4
```

Normal spectrum result:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,7020.149,-84.039
81920.0,11055.985,-80.342
40960.0,20979.338,-72.008
18204.0,40376.883,-55.262
8192.0,61266.023,-33.237
4096.0,71171.281,-16.949
2048.0,75093.234,-5.812
1024.0,76903.797,-2.066
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- At low frequencies, triangular is closest to nominal: about `93.2..93.4 kOhm` at `1024..2048 Hz`.
- At `8192 Hz`, triangular has already fallen to `84.4 kOhm`; high-frequency bins collapse strongly and show large negative phase.
- This is consistent with high-impedance parasitic/reactive behavior, input capacitance, fixture capacitance, or current-source/low-current limitations becoming significant.
- Current ranged down to `660 nA` at `8192 Hz` and `440 nA` from `4096 Hz` downward.

### 1M External Scan Stress Test

Completed as a stress/limit test. This load is outside the useful resistor-calibration range for the current external scan setup.

Fixture:

```text
Load: 1 MOhm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,7046.247,-88.583,6996.213,-81.250,6830.909,1100,4
81920.0,11121.822,-87.508,11252.189,-80.250,10780.833,1100,4
40960.0,21908.881,-86.209,22678.793,-79.250,21203.918,1100,4
18204.0,48863.902,-83.755,52547.316,-77.500,47099.523,660,4
8192.0,106219.305,-78.701,122179.766,-74.000,100969.430,440,4
4096.0,129461.242,-68.652,174922.234,-71.000,113433.492,440,4
2048.0,137150.078,-48.328,226707.500,-68.500,113433.492,440,4
1024.0,133026.750,-33.594,208181.266,-61.250,113433.492,440,4
```

Normal spectrum result:

```text
frequency_hz,magnitude_ohm,phase_deg
131072.0,7046.247,-88.583
81920.0,11121.822,-87.508
40960.0,21908.881,-86.209
18204.0,48863.902,-83.755
8192.0,106219.305,-78.701
4096.0,129461.242,-68.652
2048.0,137150.078,-48.328
1024.0,133026.750,-33.594
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- No bin approaches `1 MOhm`; even the largest triangular estimate is about `227 kOhm`.
- Max-absolute appears to plateau around `113 kOhm` from `4096 Hz` downward.
- All bins show large negative phase, consistent with capacitance/leakage/current-limit behavior dominating the measurement.
- Current used `440 nA` from `8192 Hz` downward.
- Treat this as a boundary/limit test, not a calibration point.

### X2 RC External Scan Diagnostic

Completed on `X2 = (620 ohm + 3.3 nF series) || 1 kOhm`.

Fixture:

```text
Load: (620 ohm + 3.3 nF series) || 1 kOhm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,346.629,-19.801,485.873,-52.250,373.426,80000,4
81920.0,395.406,-24.242,567.047,-53.750,410.797,80000,4
40960.0,527.112,-26.135,748.886,-53.250,527.609,64000,4
18204.0,694.262,-17.946,923.052,-48.750,726.186,8000,4
8192.0,770.992,-8.357,951.073,0.250,854.105,8000,4
4096.0,794.103,-2.787,960.796,1.000,906.867,8000,4
2048.0,803.994,1.442,957.358,2.000,930.628,8000,4
1024.0,805.804,1.710,951.012,1.500,938.062,8000,4
```

Theoretical impedance:

```text
frequency_hz,expected_magnitude_ohm,expected_phase_deg
131072.0,433.987,-17.892
81920.0,496.030,-23.546
40960.0,664.462,-26.220
18204.0,876.195,-18.273
8192.0,969.496,-9.373
4096.0,992.040,-4.820
2048.0,997.988,-2.427
1024.0,999.496,-1.216
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- Cosine phase tracks the theoretical RC phase shape closely.
- Cosine magnitude is low by an approximately consistent scale factor around `0.79..0.80` for most bins.
- Triangular/max-absolute are useful resistor diagnostics, but triangular is not a valid complex-impedance phase model for this RC load.
- This result argues for keeping cosine as the external complex-impedance estimator and applying calibration/scale correction, rather than replacing external scan output with triangular fit.

### X1 Low-Impedance RC External Scan

Completed on `X1 = (27 ohm + 47 nF series) || 60 ohm`. This checks whether cosine still tracks RC theory in the low-impedance range where the `100 ohm` resistor scan was stable.

Fixture:

```text
Load: (27 ohm + 47 nF series) || 60 ohm
Physical wiring: 4-wire fixture setup
Driver setting: 2-wire
```

Run sequence used:

```text
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

Observed diagnostic result:

```text
frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
131072.0,20.430,-27.097,29.670,-54.750,20.735,96000,4
81920.0,25.227,-30.408,36.675,-55.250,24.422,96000,4
40960.0,34.876,-26.495,48.965,-52.500,34.226,96000,4
18204.0,43.128,-15.042,55.684,-47.000,45.744,96000,4
8192.0,45.990,-6.588,56.272,0.250,51.552,80000,4
4096.0,46.911,-2.194,56.546,1.000,53.739,32000,4
2048.0,47.454,1.358,56.731,2.250,54.823,16000,4
1024.0,46.980,0.251,56.065,1.500,54.322,8000,4
```

Theoretical impedance:

```text
frequency_hz,expected_magnitude_ohm,expected_phase_deg
131072.0,24.706,-27.198
81920.0,30.755,-31.434
40960.0,43.479,-28.374
18204.0,54.919,-16.807
8192.0,58.839,-8.148
4096.0,59.702,-4.137
2048.0,59.925,-2.076
1024.0,59.981,-1.039
```

Interpretation:

- The normal spectrum is exactly the cosine diagnostic output.
- Cosine phase and frequency shape again track the theoretical RC model.
- Cosine magnitude is low by a similar scale factor to X2, roughly `0.78..0.83`.
- Triangular/maxabs do not provide the expected complex phase and should remain diagnostics rather than final external scan output.

## Model Decision Checkpoint

At this point the resistor trend is clear:

- cosine output is consistently low for reduced-phase resistor scans,
- triangular output is closest to nominal across `100 ohm`, `1 kOhm`, `10 kOhm`, and the low-frequency part of `100 kOhm`,
- `1 MOhm` is outside the current useful resistor-calibration range,
- high-frequency behavior on `10 kOhm`, `100 kOhm`, and `1 MOhm` shows non-ideal parasitic/reactive effects.

Working interpretation after the resistor and RC runs:

- Keep cosine for external complex impedance because it tracks RC phase.
- Use triangular/maxabs as resistor diagnostics, not as the final complex impedance model.
- Add calibration/scale correction on cosine magnitude before finalizing external BIOZ scan output.

### Next Implementation Step: Calibrated External Cosine Output

The next driver change should keep the existing cosine fit and add a calibrated magnitude output path.

Initial conservative approach:

- keep normal `frequency_hz,magnitude_ohm,phase_deg` as cosine output until the calibration factor is explicit,
- add a temporary calibrated diagnostic column or row that reports `cosine_calibrated_magnitude_ohm`,
- derive the first calibration factor from multiple known loads, not only one resistor,
- preserve cosine phase unchanged.

Candidate first-order factor:

```text
cosine_calibrated_magnitude_ohm = cosine_magnitude_ohm * 1.25
```

This is consistent with the two RC loads and the low/mid impedance resistor data, but it should remain a validation calibration until confirmed against a broader fixture set.

## Calibration Planning

Do not finalize the calibration from only one resistor or one RC load. The current data supports the direction, but the calibration method should be chosen after a focused calibration dataset.

Current working model:

- Use cosine fit for external complex impedance because it tracks RC phase.
- Preserve cosine phase unless later data shows a repeatable phase offset.
- Apply a magnitude calibration to cosine output.
- Keep triangular and max-absolute outputs as diagnostics for now.

Candidate calibration models to compare:

1. Global magnitude scale:
   ```text
   Z_calibrated = Z_cosine * K
   ```
   This is easiest to implement and currently looks plausible with `K` around `1.25`.

2. Per-frequency magnitude scale:
   ```text
   Z_calibrated(f) = Z_cosine(f) * K[f]
   ```
   This may be needed if high-frequency or low-frequency bins show repeatable scale differences after excluding fixture parasitics.

3. Per-current or per-current-range scale:
   ```text
   Z_calibrated(f, current_range) = Z_cosine(f) * K[f][current_range]
   ```
   This should only be used if repeated tests show the `8000 nA`, high-current, and low-current paths have different scale factors.

4. Complex calibration:
   ```text
   Z_calibrated = (Z_cosine_complex - Z_open_or_offset) * K_complex[f]
   ```
   This is more powerful but should wait until open/short/load standards and repeatability data justify the extra complexity.

Recommended automated calibration dataset before choosing the final calibration:

Use as many known fixture impedances as practical, not only the five loads used for manual bring-up. The fixture has enough known standards to fit and compare models instead of guessing from a small set.

Include:

- all pure resistors up to and including `100 kOhm`,
- `1 MOhm` and any larger/highest impedance only as limit/parasitic checks,
- all available `10 kOhm` series-RC and parallel-RC loads,
- both custom loads already measured:
  - `X1 = (27 ohm + 47 nF series) || 60 ohm`,
  - `X2 = (620 ohm + 3.3 nF series) || 1 kOhm`,
- open/short or lowest/highest available settings if the fixture provides them.

Do not exclude `100 kOhm` from calibration collection. It should be included because it helps identify the useful frequency/impedance boundary. However, it should not be allowed to dominate a simple scale fit at frequencies where parasitics clearly dominate.

Decision rule:

- If one `K` fits the usable subset of resistors and RC loads within the desired tolerance across the useful frequency range, start with a global scale factor.
- If scale error is frequency-dependent but repeatable, use a per-frequency scale table.
- If scale changes when the scan changes current range, add current-range metadata or current-dependent correction.
- If phase errors become repeatable on RC loads, add phase correction only after magnitude calibration is stable.

Storage decision:

- First implement calibration constants as compile-time defaults or serial-settable runtime values.
- Do not write persistent settings until the calibration structure is stable.
- On ESP32, prefer `Preferences`/NVS over classic EEPROM emulation for persistent calibration constants.
- Store a version number, checksum, model type, timestamp or sequence number, and the constants.
- Add serial commands for print/load/save/clear calibration only after the calibration model is selected.

## Automated Calibration Program

Use a host-side Python program for the full calibration dataset. The fixture remains manually selected, so the program should be semi-automated:

```text
Set fixture to R100. Press Enter when ready.
```

Then the program should:

1. Open the serial port at `115200`.
2. Clear any stale input.
3. Send the scan setup commands:

   ```text
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

4. Capture the entire serial response for that load.
5. Parse all temporary diagnostic rows:

   ```text
   scan_external_summary,frequency_hz,cosine_magnitude_ohm,cosine_phase_deg,triangular_magnitude_ohm,triangular_phase_deg,maxabs_ohm,current_nA,num_phase_points
   ```

6. Parse the normal spectrum rows:

   ```text
   frequency_hz,magnitude_ohm,phase_deg
   ```

7. Compute theoretical complex impedance for the selected load at each frequency.
8. Write raw log, parsed per-frequency CSV, and a run metadata file.
9. Continue to the next fixture setting only after the user confirms the load was changed.

Keep the temporary `scan_external_summary` output for the automated calibration run. The normal spectrum alone is not sufficient because the diagnostic rows carry current, point count, triangular magnitude, and max-absolute values needed to compare model behavior and identify current-range effects.

The diagnostic summary rows are log messages and may be prefixed by the logging library, for example:

```text
[INFO] scan_external_summary,...
```

The normal spectrum remains raw data and should not have a log prefix:

```text
frequency_hz,magnitude_ohm,phase_deg
...
```

The Python parser should accept `scan_external_summary` lines either with or without a leading logger prefix, then parse the comma-separated payload starting at `scan_external_summary`.

Set the sketch log level low enough that parsing is reliable. `LOG_LEVEL_INFO` is acceptable if the parser only consumes CSV-prefix lines and ignores `[INFO]`/`[WARN]` lines. If parsing becomes fragile, add a command or compile-time option for a calibration log mode that prints only:

- command echo or run marker,
- `scan_external_summary` header and rows,
- normal spectrum header and rows,
- final status line.

Recommended output files:

```text
calibration_runs/
  YYYYMMDD_HHMMSS/
    manifest.json
    raw/
      R100.txt
      R1K.txt
      ...
    parsed/
      measurements.csv
      fit_summary.csv
```

Suggested `manifest.json` fields:

```text
run_id
date_time
operator
board
fixture_revision
physical_wiring
driver_wire_setting
sketch
git_commit_or_dirty_note
scan_commands
loads
notes
```

Suggested `measurements.csv` fields:

```text
load_id,
load_description,
topology,
component_values,
frequency_hz,
expected_real_ohm,
expected_imag_ohm,
expected_magnitude_ohm,
expected_phase_deg,
cosine_magnitude_ohm,
cosine_phase_deg,
cosine_real_ohm,
cosine_imag_ohm,
triangular_magnitude_ohm,
triangular_phase_deg,
maxabs_ohm,
normal_magnitude_ohm,
normal_phase_deg,
current_nA,
num_phase_points,
scale_expected_over_cosine,
cosine_magnitude_error_pct,
cosine_phase_error_deg,
scan_avg,
scan_fast,
scan_fullrange,
scan_phase_range,
settle_samples,
current_change_settle_samples,
initial_current_nA,
warnings,
raw_log_file,
notes
```

Initial calibration program:

```text
calibration/
  fixture_loads.json
  bioz_calibration_runner.py
  requirements.txt
```

Install the Python serial dependency:

```bash
python3 -m pip install -r calibration/requirements.txt
```

Check the fixture manifest and theoretical impedance calculations without using the board:

```bash
python3 calibration/bioz_calibration_runner.py --dry-run --load R100 --load X1 --load X2
```

Run a selected calibration collection:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0 --load R100 --load R1K --load R10K --load P2 --load S3 --load X1 --load X2
```

Run the full fixture collection:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0
```

Resume an interrupted collection or add more loads to an existing collection:

```bash
python3 calibration/bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/YYYYMMDD_HHMMSS --load R330 --load P1 --load S1
```

The runner skips loads that already have a parseable `raw/<load_id>.txt` file. Use `--force-rerun` if a load should be measured again. Raw logs, `measurements.csv`, `fit_summary.csv`, and `manifest.json` are flushed and fsynced so completed loads survive an interruption.

For each prompted load, set the fixture to the displayed setting and press Enter. Keep Arduino IDE Serial Monitor closed while the Python program owns the serial port.

On connection, the runner sends a blank line to release the sketch startup prompt, waits for `Ready for commands` if the board reset happened, and only then sends the scan commands. This prevents the sketch startup input flush from discarding the measurement commands.

During each scan the runner prints elapsed time plus diagnostic and spectrum row counts. It also reports each `scan_external_summary` row and normal spectrum row as it arrives. This progress output is operator feedback only; the saved raw log remains the unchanged serial response.

Additional calibration collection sequence:

The first automated run collected:

```text
R100, R1K, R10K, P2, S3, X1, X2
```

Continue the same run with the remaining unique standards before choosing a final calibration model.

Batch 1, resistor range fill-in:

```bash
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/20260716_162307 --load R330 --load R3K3 --load R33K --load R100K
```

Batch 2, remaining 10 kOhm and 100 kOhm parallel RC standards:

```bash
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/20260716_162307 --load P1 --load P3 --load P4 --load P5
```

Batch 3, remaining series RC standards:

```bash
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/20260716_162307 --load S1 --load S2 --load S4 --load S5
```

Batch 4, boundary/parasitic checks:

```bash
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --resume calibration_runs/20260716_162307 --load R330K --load R1M --load P6
```

Do not use `--force-rerun` for repeatability measurements in the same run because it overwrites the existing `raw/<load_id>.txt` file. Use a new timestamped run for repeatability checks after all unique fixture standards have been collected.

Analyze the full fixture run:

```bash
python3 calibration/bioz_calibration_analyze.py --out calibration/calibration_analysis/20260716_full_only calibration/calibration_runs/20260716_162307
```

After reviewing the provisional usable region, rerun model fitting using only validated candidate rows:

```bash
python3 calibration/bioz_calibration_analyze.py --model-fit-source validated_candidate --out calibration/calibration_analysis/20260717_validated calibration/calibration_runs/20260716_162307
```

Analyze the full fixture run plus repeatability run:

```bash
python3 calibration/bioz_calibration_analyze.py --out calibration/calibration_analysis/20260716_initial calibration/calibration_runs/20260716_162307 calibration/calibration_runs/20260716_173454
```

The analyzer computes provisional validation classes internally. It does not require `measurements.csv` to already contain validation columns. If `--model-fit-source validated_candidate` is requested and no validated rows exist, the analyzer falls back to first-pass `fit_eligible` rows and records this in `summary.txt`.

Primary analysis outputs:

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

Use the full fixture run for calibration model choice. Use the repeatability run to confirm stability and to distinguish systematic residuals from random run-to-run variation.

## Calibration Analysis Plan

### Overall Goal

The purpose of the external fixture calibration is not only to make the 100 ohm measurement correct. The goal is to define the valid operating region for external BIOZ scan and choose the simplest correction that works across resistors, RC networks, and tissue-like fixture loads.

Current conclusions:

- Use the cosine fit as the primary external scan result because it preserves RC phase behavior.
- Keep triangular and max-absolute estimates as diagnostics only.
- Apply magnitude correction to the cosine result.
- Do not apply phase or complex correction unless residual phase errors remain systematic after unusable regions are excluded.
- Use repeatability data to verify stability, not to double-weight the same fixture standards during model selection.

### What The Analysis Program Computes

`bioz_calibration_analyze.py` reads one or more `parsed/measurements.csv` files and computes:

- uncorrected cosine magnitude and phase residuals,
- candidate magnitude scale factors,
- corrected residuals for each candidate model,
- per-frequency and per-current scale tables,
- leave-one-load-out cross-validation errors,
- rows that should be reviewed as exclusion candidates.

Magnitude residual:

```text
error_pct = 100 * (corrected_magnitude - expected_magnitude) / expected_magnitude
```

Phase residual:

```text
phase_error_deg = measured_cosine_phase_deg - expected_phase_deg
```

The current script corrects magnitude only. It does not change phase.

### Correction Models

All current candidate models multiply cosine magnitude by a scale factor `K`.

Uncalibrated:

```text
Z_corrected_mag = Z_cosine_mag
```

This is the current driver behavior and is retained as a baseline.

Global median:

```text
K = median(expected_mag / cosine_mag) over eligible fit rows
Z_corrected_mag = K * Z_cosine_mag
```

This is robust against outliers and is the preferred first candidate if it is close enough to the more complex models.

Global least-squares:

```text
K = argmin sum((K * cosine_mag - expected_mag)^2)
```

This can be pulled toward large-impedance rows, so it is useful for comparison but is less robust than the median.

Frequency median:

```text
K[f] = median(expected_mag / cosine_mag) for each scan frequency
Z_corrected_mag = K[f] * Z_cosine_mag
```

Use this only if residuals show repeatable frequency-dependent scale errors after excluding invalid high-impedance/parasitic regions.

Current median:

```text
K[current] = median(expected_mag / cosine_mag) for each selected BIOZ current
Z_corrected_mag = K[current] * Z_cosine_mag
```

Use this only if residuals truly group by current after frequency and impedance effects are separated. Current is selected by the scan optimizer, so it is partially confounded with impedance range. A current-based table can look better numerically while actually encoding load-range behavior.

Complex correction:

```text
Z_corrected_complex = (Z_cosine_complex - offset[f]) * K_complex[f]
```

Do not choose this yet. It requires stable phase-error patterns, offset/open/short characterization, and a clear benefit over magnitude-only correction.

### Fit Eligibility Versus Exclusion Candidates

The analysis program distinguishes two concepts:

1. Fit eligibility: whether a row is allowed to determine calibration constants.
2. Exclusion candidate: whether a row should be reviewed as outside the reliable operating region.

A row can be excluded from fitting but still appear in residual output. Boundary rows are useful because they define where the measurement starts to fail.

`fit_hint` by itself should not mean that a measurement is bad. It means the row is not intended to determine calibration constants. For example, `R330K`, `R1M`, and `P6` are useful boundary/parasitic measurements even though they should not pull the scale factor.

Current default fit eligibility rules:

```text
fit_hint must be calibration or tissue_model
expected_magnitude_ohm must be >= 20 ohm
expected_magnitude_ohm must be <= 50000 ohm
abs(cosine_phase_error_deg) must be <= 10 deg
cosine magnitude must be present and > 0
```

Optional rule:

```text
--exclude-warnings
```

This excludes rows with warning text from fitting. It is not enabled by default because many warnings currently come from scan current-limit behavior and do not always mean the measurement is unusable.

### Exclusion Reason Meanings

`fit_hint`:

The fixture manifest labels each load with a purpose. Rows with `fit_hint` outside `calibration` or `tissue_model` are not used to fit scale constants. Examples are high-impedance boundary/parasitic checks such as `R330K`, `R1M`, and `P6`.

`below_min_fit_ohm`:

The theoretical impedance magnitude is below the default fitting lower limit, currently 20 ohm. Very low impedances can be useful as boundary checks but should not dominate a model intended for general BIOZ scan.

`above_max_fit_ohm`:

The theoretical impedance magnitude is above the default fitting upper limit, currently 50 kOhm. This protects the fit from high-impedance regions where fixture parasitics and input capacitance dominate, especially at high frequency.

`phase_error`:

The row has more than 10 deg difference between measured cosine phase and expected phase. This usually indicates the row is outside the reliable region, or that the simple theoretical fixture model is no longer sufficient at that frequency/impedance.

`global_residual`:

After applying the global median magnitude scale, the magnitude error still exceeds the reporting threshold, currently 10 percent. This does not automatically mean the row is excluded from raw data; it means it should be reviewed as a poor fit for a simple global correction.

`phase_residual`:

The measured phase differs from expected phase by more than the reporting threshold, currently 10 deg. This is the same physical quantity as `phase_error`, but it is reported in `exclusion_candidates.csv` to make residual review explicit.

### How To Determine Official Usable Region

Use `exclusion_candidates.csv` and `residuals.csv` to define rules that can be applied to future scans.

Use `usable_region_by_load.csv` as the first review table. It summarizes each fixture setting and counts how many frequency bins are:

```text
validated_candidate
boundary_candidate
questionable
unreliable
```

Use `usable_region_review.csv` when a specific load/frequency needs detail. This table includes fit eligibility, magnitude residual, phase residual, selected current, and the provisional usability class.

Use `residuals.csv` for detailed model comparison and rerunning fits. It is intentionally wide and is not meant to be the first human review table.

Proposed decision sequence:

1. Exclude boundary fixture categories from fitting: `R330K`, `R1M`, `P6`, and any row marked `parasitic_check`.
2. Exclude rows above the validated impedance limit. Start with 50 kOhm expected magnitude, then adjust based on residuals.
3. Exclude rows where absolute phase residual exceeds 10 deg, unless a specific RC load shows a repeatable and explainable fixture parasitic effect.
4. Exclude rows where global-median corrected magnitude residual exceeds 10 percent when neighboring frequencies/loads do not show the same behavior.
5. Review high-frequency bins separately. High-frequency errors on 10 kOhm and larger loads are the first likely unusable region.
6. Recompute model summaries after exclusions.
7. Prefer the simplest correction model whose cross-validation residuals are close to the best model.

Use this command for step 6 after the first review:

```bash
python3 calibration/bioz_calibration_analyze.py --model-fit-source validated_candidate --out calibration/calibration_analysis/20260717_validated calibration/calibration_runs/20260716_162307
```

Current model-selection guidance:

- Choose `global_median` if it is within roughly 1 percentage point mean absolute residual of `frequency_median` or `current_median`.
- Choose `frequency_median` if residuals remain frequency-dependent after official exclusions.
- Choose `current_median` only if current grouping improves residuals on held-out loads and does not merely track impedance range.
- Do not choose complex correction until magnitude-only correction and usable-region rules are stable.

Calibration model decision:

Use global magnitude median scale as the first correction model.

Reasoning:

- `current_median` gives slightly lower in-sample residuals.
- The improvement does not hold up meaningfully in cross-validation.
- Current is selected by the scan optimizer and is confounded with impedance range.
- `frequency_median` is not materially better than `global_median` after validated-candidate filtering.
- `global_median` is simplest to implement, easiest to validate, and gives equivalent practical results.

Implementation target:

```text
Z_corrected_mag = K_global_median * Z_cosine_mag
phase_corrected_deg = Z_cosine_phase_deg
```

Use the validated-candidate analysis output to choose the initial `K_global_median` value. Keep phase unmodified.

Current initial value from `calibration/calibration_analysis/20260717_validated`:

```text
K_global_median = 1.25065
```

### Firmware Calibration Storage

Current firmware status:

- The repo does not currently use ESP32 `Preferences`, Arduino `EEPROM`, or another persistent settings library.
- The existing serial commands `(` and `)` call `afe.saveConfig()` and `afe.restoreConfig()`.
- That snapshot is RAM-only and stores MAX30001G register values in `_savedConfig`.
- It does not persist across reboot and is not suitable for calibration constants.

Implementation recommendation:

- Start with compile-time defaults and serial commands for print/set/clear calibration while validating behavior.
- Add persistent storage only after the calibration structure is stable.
- On ESP32, use `Preferences`/NVS rather than EEPROM emulation.
- Store calibration as a versioned profile, not as an anonymous scalar.

Suggested calibration profile fields:

```text
magic
version
profile_name
hardware_layout
wire_mode
calibration_model
K_global_median
valid_frequency_mask
valid_magnitude_min_ohm
valid_magnitude_max_ohm
reliability_table_version
created_or_sequence
checksum
```

Rationale:

- The scale factor and reliability map may change between MAX30001G devices.
- They may also change with electrode wiring, 2-wire versus 4-wire layout, cable/fixture parasitics, board revision, analog front-end layout, or scan settings.
- A global compile-time constant is acceptable for initial validation but should not be treated as a universal production calibration.
- The driver should have safe defaults if no stored calibration is present.

Planned serial commands should include:

```text
print calibration profile
set global K
enable/disable calibration
save calibration profile
clear stored calibration profile
print reliability rules
```

Initial runtime-only implementation in `MAX30001G.ino`:

```text
Kp          print BIOZ scan calibration settings
Ke0         disable external scan correction
Ke1         enable external scan correction
Kg1250650   set global K to 1.250650, encoded as ppm
Kr          reset runtime calibration defaults
```

Current behavior:

- Correction is runtime-only and is not persisted across reboot.
- Default `K` is `1.25065`.
- Correction defaults to OFF to preserve raw calibration data collection unless explicitly enabled.
- When correction is OFF, scan output remains:

```text
frequency_hz,magnitude_ohm,phase_deg
```

- When correction is ON for external scans, scan output becomes:

```text
frequency_hz,magnitude_ohm,phase_deg,reliability
```

- The printed magnitude is corrected by `K_global_median`.
- Phase is unchanged.
- Reliability is provisional:

```text
3 = validated region
2 = boundary region
1 = questionable region
0 = outside validated region
```

Provisional reliability map:

```text
Corrected       Frequency (Hz)
magnitude       1024  2048  4096  8192  >=18204
--------------------------------------------------
<20 ohm            1     1     1     1      1
20 ohm-10 kOhm     3     3     3     3      3
10 kOhm-33 kOhm    3     3     3     3      1
33 kOhm-50 kOhm    3     3     3     1      1
50 kOhm-100 kOhm   2     2     1     1      1
100 kOhm-150 kOhm  1     0     0     0      0
>150 kOhm          0     0     0     0      0
```

### Firmware Reliability Score

A firmware reliability score has been added as a runtime-only provisional output in `MAX30001G.ino`. It should be validated on the fixture before it is treated as final or persisted.

The firmware cannot know the fixture's theoretical impedance during a real measurement. It can only score reliability from observed measurement conditions and calibrated validity maps:

- scan frequency,
- corrected magnitude,
- measured phase,
- selected current range,
- warning/status flags,
- whether the point is inside the validated magnitude/frequency/current region,
- whether the point is near a known parasitic/high-impedance boundary.

Proposed firmware output later:

```text
frequency_hz,magnitude_ohm,phase_deg,reliability
```

Possible reliability values:

```text
3 = validated region
2 = usable but near boundary
1 = questionable, high residual risk
0 = outside validated region
```

Do not persist this map until the firmware verification run confirms the corrected output and reliability labels.

## Next Verification: Runtime Correction And Reliability Output

Status: firmware uploaded on 2026-07-17. Corrected-output verification is pending.

Purpose:

- Verify `K_global_median = 1.25065` improves external BIOZ scan magnitude on representative fixture loads.
- Verify phase remains unchanged.
- Verify reliability values match the provisional usability map.
- Verify the calibration runner and parser behavior with corrected output enabled.

Correction defaults to OFF and is runtime-only. Do not rely on enabling it in a
separate serial monitor before starting the Python runner because opening the
runner serial connection may reset the board. For corrected-output verification,
let the runner send the correction commands in the same serial session used for
the scan.

The runner sends these before the scan only when `--enable-correction` is used:

```text
Ke1
Kg1250650
```

Recommended verification subset:

```text
R100, R1K, R10K, R33K, R100K, P2, P4, S3, X1, X2
```

Run from the calibration directory:

```bash
cd /home/uutzinger/Documents/GitHub/UUtzinger_MAX30001G/calibration
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --enable-correction --load R100 --load R1K --load R10K --load R33K --load R100K --load P2 --load P4 --load S3 --load X1 --load X2
```

Runner/parser note:

- With correction disabled, scan output is the original 3-column table.
- With correction enabled, scan output includes `reliability` as a fourth column.
- The Python runner accepts both formats and stores the fourth column as `normal_reliability` when present.
- The Python runner defaults to raw uncorrected mode for consistency with the prior calibration datasets. Use `--enable-correction` only for verification runs or corrected-output characterization.

Expected checks:

- Corrected `R100`, `R1K`, `X1`, and `X2` magnitudes should be close to theoretical values in validated bins.
- `R10K` and `R33K` high-frequency bins should show lower reliability than low-frequency bins.
- `R100K` should remain mostly boundary/questionable rather than being treated as fully validated.
- The scan should not require persistence or `Preferences` yet.

### Runtime Correction Verification Result

Completed on 2026-07-17 with firmware correction enabled by the Python runner:

```bash
python3 ./bioz_calibration_runner.py --port /dev/ttyACM0 --enable-correction --load R100 --load R1K --load R10K --load R33K --load R100K --load P2 --load P4 --load S3 --load X1 --load X2
```

Run folder:

```text
calibration/calibration_runs/20260717_133127
```

Run checks:

- Manifest recorded `correction_enabled: true`.
- Manifest recorded `correction_scale: 1.25065`.
- All 10 selected fixture loads were captured.
- Parsed data contains 80 measured rows.
- Raw spectrum output used the corrected 4-column format:

```text
frequency_hz,magnitude_ohm,phase_deg,reliability
```

Corrected magnitude summary:

```text
Load   Mean error   Median error   Reliability summary
P2       -3.00%       -2.97%       all 3
P4       -4.61%       -4.33%       all 3
R100     +1.94%       +2.03%       all 3
R1K      +1.61%       +1.94%       all 3
R10K     -8.73%       -2.92%       all 3
R33K    -25.59%      -11.58%       5 rows 3, 3 rows 1
R100K   -43.07%      -36.31%       1 row 3, 2 rows 2, 5 rows 1
S3      -10.93%       -5.67%       all 3
X1       -0.38%       -1.53%       all 3
X2       -0.11%       -0.20%       all 3
```

Assessment:

- The global median correction works well for the key verification anchors `R100`, `R1K`, `X1`, and `X2`.
- The RC loads `X1` and `X2` are especially important because they confirm the cosine magnitude correction still preserves the complex impedance behavior that triangular/maxabs could not represent.
- `P2` and `P4` also remain within a practical error band across the scan.
- `R10K`, `R33K`, `R100K`, and `S3` still show high-frequency collapse and large negative phase in the upper bins. This is consistent with the earlier boundary/parasitic observations and is not fixed by a magnitude scale factor.
- The provisional reliability map is not final. Some high-frequency resistor stress rows still report `reliability=3` because the firmware currently scores only the measured corrected magnitude/frequency region. It does not know that the fixture switch was set to a nominal high-value resistor.

Reliability follow-up:

- Keep the global magnitude correction.
- Do not persist the current reliability map as final.
- Refine reliability scoring before README/production cleanup. The next pass should consider whether measured phase, current range, warnings/status flags, and known high-frequency/high-impedance fixture behavior should reduce reliability.
- Be careful not to reject valid capacitive samples only because phase is large; phase can be expected for real RC/tissue-like loads.

### Reliability Map Revision 1

Implemented after reviewing `calibration/calibration_runs/20260717_133127`.

Problem found:

- The first reliability map scored only corrected magnitude and frequency.
- Some high-frequency resistor-stress rows collapsed into lower measured magnitude bins and therefore reported `reliability=3`.
- Examples from the corrected-output verification:
  - `R10K` at `131072 Hz`: corrected magnitude about `6.78 kOhm`, phase about `-45 deg`, error about `-32%`, but old score `3`.
  - `R33K` at `131072 Hz`: corrected magnitude about `8.48 kOhm`, phase about `-74 deg`, error about `-74%`, but old score `3`.
  - `R100K` at `131072 Hz`: corrected magnitude about `8.81 kOhm`, phase about `-84 deg`, error about `-91%`, but old score `3`.

Firmware change:

- Keep the base corrected-magnitude/frequency table.
- Add a conservative high-frequency phase downgrade only for moderate/high measured impedances:

```text
if corrected_magnitude >= 5 kOhm:
  if frequency >= 81920 Hz and abs(phase) >= 25 deg:
    reliability <= 1
  else if frequency >= 40960 Hz and abs(phase) >= 20 deg:
    reliability <= 2
```

Rationale:

- This catches the verified high-frequency resistor-stress failures.
- It does not downgrade low-ohm RC loads such as `X1`.
- It does not downgrade capacitor-like loads such as `P2`/`P4`, where large phase is expected and the corrected magnitude error was small.
- It is still provisional. Later versions can add current-range/status-warning information if needed.

Simulation against `calibration/calibration_runs/20260717_133127`:

```text
Old reliability counts: 1=8, 2=2, 3=70
New reliability counts: 1=14, 2=3, 3=63

Changed rows:
R10K   131072 Hz  3 -> 1
R10K    81920 Hz  3 -> 1
R33K   131072 Hz  3 -> 1
R100K  131072 Hz  3 -> 1
S3     131072 Hz  3 -> 1
S3      81920 Hz  3 -> 1
S3      40960 Hz  3 -> 2
```

Build status:

- Repo sketch compile passed.
- Installed Arduino library sketch compile passed after syncing the updated example.

## Post-Test Process Cleanup

After corrected-output verification, decide and clean up the following items:

1. Current request warnings:
   - The scan currently requests high currents that are later clamped by FCGEN-dependent limits.
   - Clean up the current optimizer so it does not repeatedly request impossible currents.
   - Keep warnings only for unexpected clamp/failure conditions.
   - 2026-07-17 cleanup status: implemented scan-side current clamping before current requests are sent to `setBIOZmag()`.
   - The scan now clamps initial, reused, and retry-adjusted high-current requests to the FCGEN-specific current ceiling before applying them.
   - Repo and installed-library builds passed after the change.

2. Diagnostic output policy:
   - Decision: keep `scan_external_summary` as a supported calibration diagnostic interface.
   - It is emitted with `LOGI`, so calibration runs that need the diagnostic rows should keep the sketch log level at `INFO` or `DEBUG`.
   - Keep it available because the Python calibration and analysis tools use cosine, triangular, max-absolute, current, and phase-point count.
   - Document it as a calibration diagnostic interface rather than temporary debug text.

3. Parser compatibility:
   - Ensure the Python runner accepts both normal 3-column raw spectrum and corrected 4-column spectrum with reliability.
   - Keep `scan_external_summary` parsing tolerant of logger prefixes such as `[INFO]`.
   - 2026-07-17 check passed:
     - Rebuilt parsed outputs from `calibration/calibration_runs/20260716_162307`; parsed 176 measured rows from 22 loads with blank `normal_reliability`.
     - Rebuilt parsed outputs from `calibration/calibration_runs/20260717_133127`; parsed 80 measured rows from 10 loads with populated `normal_reliability`.

4. Persistent calibration:
   - Decision: keep volatile register snapshots `(`/`)` separate from non-volatile Preferences.
   - Implement Preferences in `examples/MAX30001G/MAX30001G.ino` first, not in the driver library, until the calibration profile format is stable.
   - Persist semantic calibration and scan defaults, not raw register snapshots or optimized current profiles.
   - Persisted fields:
     - schema version,
     - calibration version,
     - reliability map version,
     - correction enable,
     - global `K` in ppm,
     - scan averages, fast/full/internal/source, phase range, internal AHPF override, settle counts,
     - default BIOZ current,
     - BIOZ 2-wire/4-wire preference.
   - Do not persist the optimized scan current profile; it depends on attached load/electrodes and should remain runtime state.
   - Commands:

```text
Ps  save current preferences profile
Pl  load saved preferences profile
Pd  print active preferences profile
Pc  clear saved preferences profile
```

   - Startup behavior: source defaults are applied first, then a compatible saved Preferences profile is loaded if present. Measurements are never auto-started after boot.

5. Documentation:
   - 2026-07-17 cleanup status: completed first README/Validation update after corrected-output validation.
   - Moved finalized fixture process, correction model, reliability map, and cleanup decisions into `Validation.md` and README as appropriate.
   - Keep `External BioZ Test Process.md` as the chronological working record plus reference appendix.
   - Updated README impedance spectroscopy status lines after corrected-output validation, especially the previous statements that external known-load validation was still required:
     - top summary line near `Impedance Spectroscopy`.
     - impedance-unit description near the end of README.
   - README now reflects the selected global-median correction model, provisional reliability map, remaining limitations, NVS Preferences commands, and `scan_external_summary` as a supported calibration diagnostic.

## Reference Appendix

The sections below are reference material and older bring-up guidance. They are not the next chronological test steps.


Impedance fixture

Resistors:
R0, R100, R330, R1K, R3K3, R10K, R33K, R100K, R330K, R1M

Parallel:
P1: 10 kOhm || 100 nF
P2: 10 kOhm || 10 nF
P3: 10 kOhm || 1 nF
P4: 100 kOhm || 10 nF
P5: 100 kOhm || 1 nF
P6: 1000 kOhm || 220 pF

Serial:
S1: 1 k + 100 nF
S2: 1 k + 10nF
S3: 10 kOhm + 10 nF
S4: 10 kOhm + 1 nF
S5: 100 kOhm + 1nF

Tissue:
(27 Ohm + 47nF) // 60 Ohm
(620 Ohm + 3.3nF) // 1 kOhm

The analysis program should generate:

- error tables for uncalibrated cosine magnitude and phase,
- best global `K`,
- best per-frequency `K[f]`,
- optional current-range grouped `K`,
- residual plots/tables by load and frequency,
- exclusion recommendations for loads/frequencies dominated by parasitics.

Calibration model selection should be based on cross-validation:

- Fit calibration constants on a subset of loads.
- Test on held-out loads, especially RC loads.
- Prefer the simplest model that fits both resistor and RC magnitude without degrading RC phase interpretation.

## Theoretical Impedance Models

The Python script should compute expected impedance by frequency.

Resistor:

```text
Z = R
```

Capacitor:

```text
Zc = 1 / (j * 2 * pi * f * C)
```

Series RC:

```text
Z = R + Zc
```

Parallel network:

```text
Z = 1 / ((1 / Z1) + (1 / Z2) + ...)
```

For the custom fixture loads:

```text
X1 = parallel(series(27 ohm, 47 nF), 60 ohm)
X2 = parallel(series(620 ohm, 3.3 nF), 1 kOhm)
```

## Main Validation Questions

Use the external fixture measurements to answer:

- Does fixed-frequency external BIOZ measure known resistors correctly?
- Does BIOZ scan measure known resistors correctly across frequency?
- Does the external scan fit model match resistor and RC loads?
- Is reduced-phase scan accurate enough for external known loads?
- Do external scans need different settling or AFE restart behavior than internal BIST scans?
- Which frequency bins are reliable enough for production/tissue measurements?

## Current Recommendation

Proceed in this order:

1. Manual `100 ohm` fixed-frequency external BIOZ.
2. Manual `100 ohm` external BIOZ scan.
3. Manual selected-load scans for `R1K`, `R10K`, and one RC load.
4. Create Python serial automation.
5. Run the focused load set.
6. Update `Validation.md` with results and driver decisions.

## Immediate External Scan Diagnostics

If the `100 ohm` scan completes but reports a magnitude substantially below the fixed-frequency or continuous BIOZ result, do not move to the full fixture set yet.

### Next Test: Conservative-Settling Scan

Repeat the scan with conservative settling:

```text
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

Compare the `8192 Hz` scan result against:

- `m7` external calibration baseline at `8192 Hz`, `8000 nA`, `0 deg`.
- `m2` scan-like continuous BIOZ at `8192 Hz`, `8000 nA`, `0 deg`, `20 V/V`, AHPF `150 Hz`, DLPF `4 Hz`, DHPF bypass.

Interpretation:

- If the `8192 Hz` scan result moves meaningfully upward toward `93..98 ohm`, settling is part of the error.
- If it remains near `81..82 ohm`, the dominant error is probably not the global scan settling count.

### Follow-Up Test: Manual Reduced-Phase Continuous Points

Run a manual reduced-phase continuous check at `8192 Hz`:

```text
<
m2
Bs1
Bg1
Ba1
Bd1
Bh0
Bf8000
Bc8000
Bl0
Bp0
a
.
z
```

Record a stable value at `0 deg`, then stop, change phase, apply settings, and repeat for `45`, `90`, and `135 deg`.

For each phase:

```text
<
Bp0
a
.
z
```

Then:

```text
<
Bp45
a
.
z
```

Then:

```text
<
Bp90
a
.
z
```

Then:

```text
<
Bp135
a
.
z
```

For each phase, record about 10 stable samples or their mean.

This gives the four phase points used by the reduced scan fit. If these phase points fit near the scan result, the issue is probably the external fit model. If the manual phase points fit near the continuous 0-degree result but the scan remains low, the issue is probably scan acquisition, settling, or current-ranging behavior.

### Optional Driver Diagnostic

If the manual phase-point test is inconclusive, add temporary scan diagnostics to print one row per processed phase point:

```text
scan_phase,frequency_hz,phase_deg,current_nA,raw_adc,ohm,valid,range_or_invalid
```

This should be guarded by an explicit debug flag or removed after validation. The goal is to compare scan-owned per-phase values directly against manual continuous phase values.
