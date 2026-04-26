#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = -1;
const uint32_t BIOZ_DATA_TIMEOUT_MS = 500U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_bioz_data_ms = 0;

bool drainBiozData() {
  float value = 0.0f;
  bool had_data = false;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ External Cal [ohm]: ");
    Serial.println(value, 3);
    had_data = true;
  }
  return had_data;
}

void setup() {
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  // setupBIOZExternalImpedanceCalibration(frequency, phase)
  // frequency: requested BIOZ modulation frequency in Hz
  // phase: demodulation phase in degrees
  // Example below measures the external calibration path at 8kHz and 0 degrees.
  afe.setupBIOZExternalImpedanceCalibration(8000, 0.0f);
  afe.start();
  last_bioz_data_ms = millis();

  Serial.println("MAX30001G BIOZ external impedance calibration example started.");
}

void loop() {
  afe.update();
  if (drainBiozData()) {
    last_bioz_data_ms = millis();
    return;
  }

  const uint32_t now = millis();
  if ((now - last_bioz_data_ms) >= BIOZ_DATA_TIMEOUT_MS) {
    Serial.println("No BIOZ external calibration data in BIOZ_data after 500 ms; switching to direct FIFO read.");
    afe.readBIOZ_FIFO(false);
    if (drainBiozData()) {
      last_bioz_data_ms = millis();
      return;
    }

    Serial.println("FAIL: No BIOZ external calibration data from BIOZ_data or direct FIFO read.");
    afe.readStatusRegisters();
    afe.printStatus();
    afe.stop();
    while (true) {
      delay(1000);
    }
  }
}
