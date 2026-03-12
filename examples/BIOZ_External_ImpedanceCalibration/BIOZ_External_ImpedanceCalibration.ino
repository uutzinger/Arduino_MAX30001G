#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

int currentLogLevel = LOG_LEVEL_INFO;

const uint8_t AFE_CS_PIN = 10;
const int AFE_INT1_PIN = 2;
const int AFE_INT2_PIN = -1;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);

  afe.begin();
  // setupBIOZExternalImpedanceCalibration(frequency, phase)
  // frequency: requested BIOZ modulation frequency in Hz
  // phase: demodulation phase in degrees
  // Example below measures the external calibration path at 8kHz and 0 degrees.
  afe.setupBIOZExternalImpedanceCalibration(8000, 0.0f);
  afe.start();

  Serial.println("MAX30001G BIOZ external impedance calibration example started.");
}

void loop() {
  if (!afe.update()) {
    return;
  }

  float value = 0.0f;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ External Cal [ohm]: ");
    Serial.println(value, 3);
  }
}
