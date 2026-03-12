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
  // setupECGSignalCalibration(speed, gain)
  // speed: 0=~125sps, 1=~256sps, 2=~512sps
  // gain:  0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
  // Example below uses 1=~256sps and 2=80V/V.
  afe.setupECGSignalCalibration(1, 2);
  afe.start();

  Serial.println("MAX30001G ECG signal calibration example started.");
}

void loop() {
  if (!afe.update()) {
    return;
  }

  float value = 0.0f;
  while (ECG_data.available() > 0) {
    ECG_data.pop(value);
    Serial.print("ECG Cal [mV]: ");
    Serial.println(value, 3);
  }
}
