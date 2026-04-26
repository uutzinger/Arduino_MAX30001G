#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t  AFE_CS_PIN            = 6;
const int      AFE_INT1_PIN          = 12;
const int      AFE_INT2_PIN          = -1;
const uint32_t PLL_STARTUP_SETTLE_MS = 500U;
const uint32_t PLL_STATUS_CLEAR_MS   = 10U;
const uint32_t ECG_DATA_TIMEOUT_MS   = 100U;
const uint8_t  ECG_FIFO_INTERRUPT_THRESHOLD = 8U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_data_check_ms = 0;
bool stopped_after_failure = false;

bool reportPllStatus(const char* context) {
  afe.readStatusRegisters();
  const uint32_t status_word = status.all & 0x00FFFFFFUL;
  const bool pll_unlocked = ((status_word & MAX30001_STATUS_PLLINT) != 0U);

  Serial.print(context);
  Serial.print(" PLL: ");
  Serial.print(pll_unlocked ? "UNLOCKED" : "OK");
  Serial.print(" (STATUS=0x");
  Serial.print(status_word, HEX);
  Serial.println(")");

  if (pll_unlocked) {
    Serial.println("PLL remains unlocked after ECG calibration is active; check the MAX30001G external clock/oscillator.");
  }

  return !pll_unlocked;
}

bool drainEcgData() {
  float value = 0.0f;
  bool had_data = false;
  while (ECG_data.available() > 0) {
    ECG_data.pop(value);
    Serial.print("ECG_Cal_[mV]: ");
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
  // setupECGSignalCalibration(speed, gain)
  // speed: 0=~125sps, 1=~256sps, 2=~512sps
  // gain:  0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
  // Example below uses 1=~256sps and 2=80V/V.
  afe.setupECGSignalCalibration(1, 2);
  afe.setFIFOInterruptThreshold(ECG_FIFO_INTERRUPT_THRESHOLD, 8);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it was latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset(); // Discard samples collected during PLL warm-up.
  last_data_check_ms = millis();

  Serial.println("MAX30001G ECG signal calibration example started.");
  Serial.print("ECG FIFO interrupt threshold: ");
  Serial.println(ECG_FIFO_INTERRUPT_THRESHOLD);
  reportPllStatus("Startup active");
}

void loop() {
  if (stopped_after_failure) {
    delay(1000);
    return;
  }

  afe.update();
  bool had_data = drainEcgData();
  if (had_data) {
    last_data_check_ms = millis();
    return;
  }

  const uint32_t now = millis();

  if ((now - last_data_check_ms) >= ECG_DATA_TIMEOUT_MS) {
    Serial.println("No ECG calibration data in ECG_data after 100 ms; switching to direct FIFO read.");
    afe.readECG_FIFO(false);

    if (drainEcgData()) {
      last_data_check_ms = millis();
      return;
    }

    Serial.println("FAIL: No ECG calibration data from ECG_data or direct FIFO read.");
    reportPllStatus("Failure");
    afe.printStatus();
    afe.stop();
    stopped_after_failure = true;
    Serial.println("ECG signal calibration stopped.");
  }
}
