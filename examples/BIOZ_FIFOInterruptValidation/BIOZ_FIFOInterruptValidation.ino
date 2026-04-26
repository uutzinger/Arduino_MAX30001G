#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = -1;

const uint32_t PLL_STARTUP_SETTLE_MS = 500U;
const uint32_t PLL_STATUS_CLEAR_MS = 10U;
const uint32_t VALIDATION_DURATION_MS = 15000U;
const uint32_t EXPECTED_MIN_SAMPLES = 250U;
const uint32_t MAX_ALLOWED_GAP_MS = 750U;
const uint8_t BIOZ_FIFO_THRESHOLD = 8U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

uint32_t started_ms = 0U;
uint32_t last_sample_ms = 0U;
uint32_t max_gap_ms = 0U;
uint32_t bioz_samples = 0U;
uint32_t overflow_events = 0U;
bool completed = false;

void clearValidationState() {
  BIOZ_data.clear();
  bioz_available = false;
  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  bioz_overflow_occurred = false;
  valid_data_detected = false;
  over_voltage_detected = false;
  under_voltage_detected = false;
  EOF_detected = false;
}

uint32_t statusWord() {
  afe.readStatusRegisters();
  return status.all & 0x00FFFFFFUL;
}

bool pllOk(uint32_t word) {
  return (word & MAX30001_STATUS_PLLINT) == 0U;
}

void drainBiozData() {
  float value = 0.0f;
  while (BIOZ_data.available() > 0U) {
    BIOZ_data.pop(value);
    const uint32_t now = millis();
    if (last_sample_ms > 0U) {
      const uint32_t gap = now - last_sample_ms;
      if (gap > max_gap_ms) {
        max_gap_ms = gap;
      }
    }
    last_sample_ms = now;
    bioz_samples++;
  }
}

void printSummary(bool pass, uint32_t final_status) {
  Serial.println();
  Serial.println("BIOZ FIFO interrupt validation summary");
  Serial.print("samples_received=");
  Serial.println(bioz_samples);
  Serial.print("expected_min_samples=");
  Serial.println(EXPECTED_MIN_SAMPLES);
  Serial.print("max_gap_ms=");
  Serial.println(max_gap_ms);
  Serial.print("max_allowed_gap_ms=");
  Serial.println(MAX_ALLOWED_GAP_MS);
  Serial.print("overflow_events=");
  Serial.println(overflow_events);
  Serial.print("status_hex=0x");
  Serial.println(final_status, HEX);
  Serial.print("pll=");
  Serial.println(pllOk(final_status) ? "OK" : "UNLOCKED");
  Serial.print("result=");
  Serial.println(pass ? "PASS" : "FAIL");
}

void setup() {
  currentLogLevel = LOG_LEVEL_WARN;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  afe.setupBIOZImpedanceCalibration(
    0,      // low-rate BIOZ
    1,      // 20 V/V
    1,      // AHPF 150 Hz
    1,      // DLPF 4 Hz
    0,      // DHPF bypass
    8000,   // requested frequency
    8000,   // current nA
    0.0f,   // phase
    1000,   // internal resistor ohm
    0,      // fixed resistor
    0
  );
  afe.setFIFOInterruptThreshold(32U, BIOZ_FIFO_THRESHOLD);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters();
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset();
  clearValidationState();

  started_ms = millis();
  last_sample_ms = started_ms;

  Serial.println("MAX30001G BIOZ FIFO interrupt validation started.");
  Serial.println("Direct FIFO fallback is disabled during the validation window.");
}

void loop() {
  if (completed) {
    delay(1000);
    return;
  }

  const bool overflow_before = bioz_overflow_occurred;
  afe.update();
  if (overflow_before || bioz_overflow_occurred) {
    overflow_events++;
    bioz_overflow_occurred = false;
  }
  drainBiozData();

  const uint32_t now = millis();
  if ((now - started_ms) >= VALIDATION_DURATION_MS) {
    const uint32_t final_status = statusWord();
    const bool pass = (bioz_samples >= EXPECTED_MIN_SAMPLES) &&
                      (max_gap_ms <= MAX_ALLOWED_GAP_MS) &&
                      (overflow_events == 0U) &&
                      pllOk(final_status);
    printSummary(pass, final_status);
    afe.stop();
    completed = true;
  }
}
