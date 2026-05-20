#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t  AFE_CS_PIN                   = 6;
const int      AFE_INT1_PIN                 = 12;
const int      AFE_INT2_PIN                 = -1;
const uint32_t PLL_STARTUP_SETTLE_MS        = 500U;
const uint32_t PLL_STATUS_CLEAR_MS          = 10U;
const uint32_t ECG_DATA_TIMEOUT_MS          = 100U;
const uint8_t  ECG_FIFO_INTERRUPT_THRESHOLD = 8U;
const uint8_t  ECG_SPEED                    = 1U;    // 0,1,2 = ~125, 256, 512 sps
const uint8_t  ECG_GAIN                     = 2U;    // 0,1,2,3 = 20, 40, 80, 160 V/V
const uint8_t  ECG_LPF                      = 0U;    // 0,1,2,3 = bypass, 40, 100, 150 Hz
const uint8_t  ECG_HPF                      = 0U;    // 0,1 = bypass, 0.5 Hz
// true  = 3-lead ECG path
// false = 2-lead ECG path with internal lead bias
// Keep false for 2-lead external simulator validation.
const bool     ECG_THREE_LEADS              = true;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_ecg_data_ms = 0;

bool drainECGData() {
  float ecg_mV = 0.0f;
  bool had_data = false;

  while (ECG_data.available() > 0) {
    ECG_data.pop(ecg_mV);
    Serial.print("ECG Ext [mV]: ");
    Serial.print(ecg_mV, 3);

    if (RTOR_data.available() > 0) {
      float rr_ms = 0.0f;
      RTOR_data.pop(rr_ms);
      Serial.print(" RR [ms]: ");
      Serial.print(rr_ms, 1);
      if (rr_ms > 0.0f) {
        Serial.print(" HR [bpm]: ");
        Serial.print(60000.0f / rr_ms, 1);
      }
    }

    Serial.println();
    had_data = true;
  }

  return had_data;
}

void setup() {
  
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  afe.setupECG(ECG_SPEED, ECG_GAIN, ECG_THREE_LEADS);
  afe.setECGfilter(ECG_LPF, ECG_HPF);
  afe.setFIFOInterruptThreshold(ECG_FIFO_INTERRUPT_THRESHOLD, 8);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it was latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset(); // Discard samples collected during PLL warm-up.
  last_ecg_data_ms = millis();

}

void loop() {

  afe.update();
  if (drainECGData()) {
    last_ecg_data_ms = millis();
    return;
  }

  const uint32_t now = millis();
  if ((now - last_ecg_data_ms) >= ECG_DATA_TIMEOUT_MS) {
    afe.readECG_FIFO(false);
    if (drainECGData()) {
      last_ecg_data_ms = millis();
    }
  }
}
