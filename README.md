# Maxim 30001 Arduino Library

This is the MAX30001 library for Arduino. It attempts to be `complete` and supports impedance spectroscopy.

This library has been validated for ECG, continuous BIOZ, combined ECG+BIOZ, internal calibration paths, and the refactored nonblocking BIOZ scan flow that is driven through `setup...()`, `start()`, and repeated `update()`.

Impedance spectroscopy calibration with external known samples is still in progress.

# Installation

Install the library in Arduino IDE or clone the GitHub repository into your Arduino `libraries` folder.

# Dependencies

This driver depends on
- [Arduino Ring Buffer](https://github.com/uutzinger/Arduino_RingBuffer)
- [Arduino Logger](https://github.com/uutzinger/logger)

# Quick Start

Using Arduino-style `setup()` / `loop()` with `setup` + `start/update/stop`:

```cpp
#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN  = 10; // SPI chip select pin
const int AFE_INT1_PIN = 2;     // AFE INTB interrupt
const int AFE_INT2_PIN = -1;    // AFE INT2B optional

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
bool measurementRunning = false; // Global status

void setup() {
  // LOG_LEVEL_NONE, LOG_LEVEL_ERROR, LOG_LEVEL_WARN, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG
  currentLogLevel = LOG_LEVEL_DEBUG;

  Serial.begin(115200);         // Baudrate, up to 921600 for ESP

  afe.begin();

  // Configure for ECG and BIOZ
  afe.setupECGandBIOZ(
    1, 2, true,        // ECG: speed=256sps, gain=80V/V, 3-lead
    0, 1,              // BIOZ: speed=25sps, gain=20V/V
    1, 0,              // BIOZ: DLPF=4Hz, DHPF=bypass
    8000, 8000, 0.0,   // BIOZ: frequency=8kHz, current[nA], phase[deg]
    true, false, false // leadbias=enabled, leadsoffdetect=off, bioz fourleads=false
  );
}

void loop() {
  // PSEUDO CODE: replace with your own user-action logic.
  bool userRequestedStart = /* e.g., UI/event says "start" */ false;
  bool userRequestedStop  = /* e.g., UI/event says "stop"  */ false;

  if (userRequestedStart && !measurementRunning) {
    afe.start();
    measurementRunning = true;
  }

  if (userRequestedStop && measurementRunning) {
    afe.stop();
    measurementRunning = false;
  }

  if (!measurementRunning) {
    return;
  }

  // Service IRQs and drain FIFOs.
  if (afe.update()) { // Check for pending interrupts and service them
    float value = 0.0f;

    // Print all new ECG samples collected during update().
    while (ECG_data.available() > 0) {
      ECG_data.pop(value);
      Serial.print("ECG [mV]: ");
      Serial.println(value, 3);
    }

    // Print all new BIOZ samples collected during update().
    while (BIOZ_data.available() > 0) {
      BIOZ_data.pop(value);
      Serial.print("BIOZ [ohm]: ");
      Serial.println(value, 3);
    }

    // Print all new RTOR samples collected during update().
    while (RTOR_data.available() > 0) {
      RTOR_data.pop(value);
      Serial.print("RR [ms]: ");
      Serial.println(value, 1);
      if (value > 0.0f) {
        Serial.print("HR [bpm]: ");
        Serial.println(60000.0f / value, 1);
      }
    }
  }
}
```

BIOZ spectroscopy uses the same configure/start/update lifecycle, but `setupBIOZScan(...)` only configures the scan. Call `start()` once, then keep calling `update()` until a spectrum becomes available in `BIOZ_spectrum`.

# Example Sketches

The maintained example sketches match the current driver structure:

- `examples/ECG/ECG.ino`: continuous ECG using `setupECG(...)`, `start()`, and repeated `update()`
- `examples/BIOZ/BIOZ.ino`: continuous fixed-frequency BIOZ using `setupBIOZ(...)`
- `examples/ECGandBIOZ/ECGandBIOZ.ino`: simultaneous ECG and BIOZ using `setupECGandBIOZ(...)`
- `examples/BIOZScan/BIOZScan.ino`: external nonblocking BIOZ spectroscopy using `setupBIOZScan(...)`
- `examples/BIOZScan_Internal/BIOZScan_Internal.ino`: internal-resistor scan validation using the same scan-owned state machine
- `examples/Hardware_HealthCheck/Hardware_HealthCheck.ino`: startup communication and hardware checks
- `examples/MAX30001G/MAX30001G.ino`: interactive serial test program covering mode switches, setup helpers, calibration, scan control, and register inspection
- `examples/ECG_FIFOInterruptValidation/ECG_FIFOInterruptValidation.ino`: ECG FIFO interrupt and `ECG_data` validation
- `examples/BIOZ_FIFOInterruptValidation/BIOZ_FIFOInterruptValidation.ino`: BIOZ FIFO interrupt and `BIOZ_data` validation
- `examples/ECGandBIOZ_FIFOInterruptValidation/ECGandBIOZ_FIFOInterruptValidation.ino`: combined FIFO drain validation
- `examples/BIOZ_Internal_ImpedanceCalibration/BIOZ_Internal_ImpedanceCalibration.ino`: internal BIST point measurements outside the scan flow
- `examples/BIOZ_External_ImpedanceCalibration/BIOZ_External_ImpedanceCalibration.ino`: external known-load impedance calibration
- `examples/BIOZ_SignalCalibration/BIOZ_SignalCalibration.ino` and `examples/ECG_SignalCalibration/ECG_SignalCalibration.ino`: internal signal-generator calibration paths

If you want one sketch that reflects the latest driver organization most closely, start with `examples/MAX30001G/MAX30001G.ino` for interactive coverage or `examples/BIOZScan_Internal/BIOZScan_Internal.ino` for the current BIOZ scan lifecycle.

# Documentation
 - [API Documentation (generated)](https://uutzinger.github.io/Arduino_MAX30001G/)
 - \subpage md_Global_Variables "Global Variables"
 - \subpage md_Interrupts "Interrupt Handling"
 - \subpage md_CHANGELOG "Changelog"

# Hardware with MAX30001G

- [MediBrick ECG and BIOZ](https://github.com/MediBrick/ECG_BIOZ_Brick)
- [Protocentral tinyECG](https://protocentral.com/product/protocentral-tinyecg-max30001-ecg-respiration-module-for-qt-py-xiao/)
- [Protocentral MAX30001](https://protocentral.com/product/protocentral-max30001/)
- [Protocentral Healthy Pi 5](https://protocentral.com/product/healthypi-5-vital-signs-monitoring-hat-kit)

# Contributing

- Urs Utzinger, 2025-2026
- GPT, 2025- 2026

# License

See [LICENSE](License.txt).

# Block Diagram of MAX 30001G

The MAX30001G is a highly integrated analog front end that consists of a differential `ECG channel` with optional right-leg drive. It employs standard high-pass and low-pass filters, an instrumentation amplifier, and an analog-to-digital converter.

The impedance unit consists of a current driver, analog high-pass filter, and phase-shifted demodulator to measure impedance from 128kHz down to 125Hz modulation frequency at varying phase shifts. Since the analog HPF has limited ability to suppress 60Hz noise, measurements below 500Hz might not be reliable.

<a href="assets/Blockdiagram.png" target="_blank">
  <img src="assets/Blockdiagram.png" style="width: 800px;">
</a>

# Input MUX ECG

Besides ESD protection, the input MUX can detect lead-off and lead-on conditions. In addition, it can switch input electrode polarity and correct common-voltage bias on the subject. Input can be turned off and replaced with an internal signal generator for testing purposes.

<a href="assets/ECG_InputMUX.png" target="_blank">
  <img src="assets/ECG_InputMUX.png" style="width: 800px;">
</a>

# Input MUX BIOZ
Similar to the ECG input MUX, the BIOZ input MUX has ESD protection as well as lead-on and lead-off detection. Besides input signal calibration, a programmable resistor can be measured internally as simulated impedance.

<a href="assets/BIOZ_InputMUX.png" target="_blank">
  <img src="assets/BIOZ_InputMUX.png" style="width: 800px;">
</a>
