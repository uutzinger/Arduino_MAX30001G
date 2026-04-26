#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = 13;
const uint32_t ECG_DATA_TIMEOUT_MS = 100U;
const uint8_t ECG_FIFO_INTERRUPT_THRESHOLD = 8U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_ecg_data_ms = 0;

bool drainEcgData() {
  float value = 0.0f;
  bool had_data = false;
  while (ECG_data.available() > 0) {
    ECG_data.pop(value);
    Serial.print("ECG [mV]: ");
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
  // setupECG(speed, gain, threeleads)
  // speed: 0=~125sps, 1=~256sps, 2=~512sps
  // gain:  0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
  // threeleads: true=3-lead ECG, false=2-lead ECG
  afe.setupECG(1, 2, true);
  afe.setFIFOInterruptThreshold(ECG_FIFO_INTERRUPT_THRESHOLD, 8);
  afe.start();
  last_ecg_data_ms = millis();

  Serial.println("MAX30001G ECG example started.");
}

void loop() {
  afe.update();
  if (drainEcgData()) {
    last_ecg_data_ms = millis();
    return;
  }

  const uint32_t now = millis();
  if ((now - last_ecg_data_ms) >= ECG_DATA_TIMEOUT_MS) {
    Serial.println("No ECG data in ECG_data after 100 ms; switching to direct FIFO read.");
    afe.readECG_FIFO(false);
    if (drainEcgData()) {
      last_ecg_data_ms = millis();
      return;
    }

    Serial.println("No ECG data from ECG_data or direct FIFO read.");
    afe.readStatusRegisters();
    afe.printStatus();
    last_ecg_data_ms = now;
  }
}
