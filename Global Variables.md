# Global Variables

Global variables are updated by the driver and can be read by user application. They should not be written to.

## AFE globals

| Variable | Unit | Description |
|---|---|---|
| `fmstr` | Hz | Main clock frequency |
| `tres` | us | Timing resolution |
| `CAL_fcal` | Hz | Calibration source frequency |
| `CAL_resolution` | us | Calibration timing resolution |
| `RCAL_freq` | Hz | Calibration resistor modulation frequency |
| `RBIASV_res` | MOhm | Common-mode feedback resistance (ECG/BIOZ high-current mode) |
| `V_ref` | mV | ADC reference output |
| `V_AVDD` | mV | Analog supply voltage |

## BIOZ globals

| Variable | Unit | Description |
|---|---|---|
| `BIOZ_samplingRate` | sps | BIOZ sample rate |
| `BIOZ_frequency` | Hz | BIOZ modulation frequency |
| `BIOZ_ahpf` | Hz | BIOZ analog HPF (before demodulator) |
| `BIOZ_dlpf` | Hz | BIOZ digital LPF (after demodulator) |
| `BIOZ_dhpf` | Hz | BIOZ digital HPF (after demodulator) |
| `BIOZ_phase` | deg | BIOZ demodulation phase offset |
| `BIOZ_gain` | V/V | BIOZ gain |
| `BIOZ_cgmag` | nA | BIOZ current-generator magnitude |
| `BIOZ_cmres` | kOhm | BIOZ common-mode resistance (low-current mode) |
| `BIOZ_test_rnom` | kOhm | BIOZ test resistor nominal value |
| `BIOZ_test_rmod` | kOhm | BIOZ test resistor modulation value |
| `BIOZ_test_frequency` | Hz | BIOZ test resistor modulation frequency |

## ECG globals

| Variable | Unit | Description |
|---|---|---|
| `ECG_samplingRate` | sps | ECG sample rate |
| `ECG_lpf` | Hz | ECG low-pass filter |
| `ECG_hpf` | Hz | ECG high-pass filter |
| `ECG_latency` | ms | ECG signal group delay |
| `ECG_gain` | V/V | ECG amplifier gain |
| `ECG_progression` | per second | ECG progression rate |
| `RtoR_resolution` | ms | R-to-R timing resolution |
| `RtoR_delay` | ms | R-to-R detection delay |

## Data related globals

| Variable | Type | Description |
|---|---|---|
| `ECG_data` | `RingBuffer<float,128>` | ECG samples buffer |
| `BIOZ_data` | `RingBuffer<float,128>` | BIOZ samples buffer |
| `RTOR_data` | `RingBuffer<float,16>` | RR interval samples buffer (`ms`) |
| `ecg_available` | `bool` | ECG data-ready latch |
| `bioz_available` | `bool` | BIOZ data-ready latch |
| `rtor_available` | `bool` | RRINT data-ready latch |
| `rr_interval` | `float` | Most recent RR interval (`ms`) |
| `ecg_counter` | `int` | Samples added by last ECG FIFO read |
| `bioz_counter` | `int` | Samples added by last BIOZ FIFO read |
| `rtor_counter` | `int` | Samples added by last RTOR read |

## Impedance spectroscopy

| Variable | Type | Unit | Description |
|---|---|---|---|
| `impedance_magnitude[11]` | `float[11]` | Ohm | Fit result magnitude |
| `impedance_phase[11]` | `float[11]` | deg | Fit result phase |
| `impedance_frequency[11]` | `float[11]` | Hz | Frequency bins |

## Error & Status globals

| Group | Variables |
|---|---|
| Latched event/fault flags (sticky until cleared) | `ecg_fast_recovery_occurred`, `ecg_overflow_occurred`, `bioz_cgm_occurred`, `bioz_undervoltage_occurred`, `bioz_overvoltage_occurred`, `bioz_overflow_occurred`, `ecg_lead_off`, `leads_on_detected`, `pll_unlocked_occurred`, `ecg_available`, `bioz_available`, `rtor_available` |
| Per-read result flags | `valid_data_detected`, `EOF_detected` |

Reset behavior:

- Use `clearLatchedStatusFlags()` when your application has consumed latched events/faults.
- `valid_data_detected` and `EOF_detected` are reset/updated inside `readECG()`, `readBIOZ()`, `readECG_FIFO()`, `readBIOZ_FIFO()`.

Pending IRQ flags used by `servicePendingInterrupts()`:

- `afe_irq_pending` (INT1 or INT2 edge seen)
- `afe_irq1_pending`
- `afe_irq2_pending`

STATUS ownership/clearing strategy:

- MAX30001 STATUS bits like `DCLOFFINT`, `BOVER`, `BUNDR`, `BCGMON`, `LONINT`, `PLLINT` are held until STATUS readback (datasheet).
- The driver therefore latches software flags on every STATUS read path before those hardware bits can clear.
- Reading BioZ range tags (`BTAG=001/011`) can trigger a STATUS read to disambiguate over-vs-under range; this also goes through the same latching path.
- FIFO overflow tags (`ETAG/BTAG=111`) require `FIFO_RST` or `SYNCH` per datasheet, and the driver issues `FIFOReset()`.
