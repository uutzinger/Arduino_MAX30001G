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
  // setupBIOZSignalCalibration(speed, gain)
  // speed: 0=~25..32sps, 1=~50..64sps
  // gain:  0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
  // Example below uses 0=low-rate mode and 1=20V/V gain.
  afe.setupBIOZSignalCalibration(0, 1);
  afe.start();

  Serial.println("MAX30001G BIOZ signal calibration example started.");
}

void loop() {
  if (!afe.update()) {
    return;
  }

  float value = 0.0f;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ Cal [ohm]: ");
    Serial.println(value, 3);
  }
}
