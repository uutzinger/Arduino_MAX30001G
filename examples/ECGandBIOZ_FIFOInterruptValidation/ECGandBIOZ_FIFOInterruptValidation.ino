#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = 13;

const uint32_t PLL_STARTUP_SETTLE_MS = 500U;
const uint32_t PLL_STATUS_CLEAR_MS = 10U;
const uint32_t VALIDATION_DURATION_MS = 15000U;
const uint32_t EXPECTED_MIN_ECG_SAMPLES = 1500U;
const uint32_t EXPECTED_MIN_BIOZ_SAMPLES = 250U;
const uint32_t MAX_ALLOWED_ECG_GAP_MS = 250U;
const uint32_t MAX_ALLOWED_BIOZ_GAP_MS = 750U;
const uint8_t ECG_FIFO_THRESHOLD = 8U;
const uint8_t BIOZ_FIFO_THRESHOLD = 8U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

uint32_t started_ms = 0U;
uint32_t last_ecg_sample_ms = 0U;
uint32_t last_bioz_sample_ms = 0U;
uint32_t max_ecg_gap_ms = 0U;
uint32_t max_bioz_gap_ms = 0U;
uint32_t ecg_samples = 0U;
uint32_t bioz_samples = 0U;
uint32_t ecg_overflow_events = 0U;
uint32_t bioz_overflow_events = 0U;
bool completed = false;

void clearValidationState() {
  ECG_data.clear();
  BIOZ_data.clear();
  RTOR_data.clear();
  ecg_available = false;
  bioz_available = false;
  rtor_available = false;
  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  ecg_overflow_occurred = false;
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

void drainEcgData() {
  float value = 0.0f;
  while (ECG_data.available() > 0U) {
    ECG_data.pop(value);
    const uint32_t now = millis();
    if (last_ecg_sample_ms > 0U) {
      const uint32_t gap = now - last_ecg_sample_ms;
      if (gap > max_ecg_gap_ms) {
        max_ecg_gap_ms = gap;
      }
    }
    last_ecg_sample_ms = now;
    ecg_samples++;
  }
}

void drainBiozData() {
  float value = 0.0f;
  while (BIOZ_data.available() > 0U) {
    BIOZ_data.pop(value);
    const uint32_t now = millis();
    if (last_bioz_sample_ms > 0U) {
      const uint32_t gap = now - last_bioz_sample_ms;
      if (gap > max_bioz_gap_ms) {
        max_bioz_gap_ms = gap;
      }
    }
    last_bioz_sample_ms = now;
    bioz_samples++;
  }
}

void drainRtorData() {
  float value = 0.0f;
  while (RTOR_data.available() > 0U) {
    RTOR_data.pop(value);
  }
}

void printSummary(bool pass, uint32_t final_status) {
  Serial.println();
  Serial.println("ECG and BIOZ FIFO interrupt validation summary");
  Serial.print("ecg_samples_received=");
  Serial.println(ecg_samples);
  Serial.print("ecg_expected_min_samples=");
  Serial.println(EXPECTED_MIN_ECG_SAMPLES);
  Serial.print("ecg_max_gap_ms=");
  Serial.println(max_ecg_gap_ms);
  Serial.print("ecg_max_allowed_gap_ms=");
  Serial.println(MAX_ALLOWED_ECG_GAP_MS);
  Serial.print("ecg_overflow_events=");
  Serial.println(ecg_overflow_events);
  Serial.print("bioz_samples_received=");
  Serial.println(bioz_samples);
  Serial.print("bioz_expected_min_samples=");
  Serial.println(EXPECTED_MIN_BIOZ_SAMPLES);
  Serial.print("bioz_max_gap_ms=");
  Serial.println(max_bioz_gap_ms);
  Serial.print("bioz_max_allowed_gap_ms=");
  Serial.println(MAX_ALLOWED_BIOZ_GAP_MS);
  Serial.print("bioz_overflow_events=");
  Serial.println(bioz_overflow_events);
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
  afe.setupECGandBIOZ(
    1, 2, true,   // ECG ~256 sps, 80 V/V, three leads
    0, 1,         // BIOZ low rate, 20 V/V
    1, 0,         // BIOZ DLPF 4 Hz, DHPF bypass
    8000, 8000, 0.0f,
    true, false, false
  );
  afe.setFIFOInterruptThreshold(ECG_FIFO_THRESHOLD, BIOZ_FIFO_THRESHOLD);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters();
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset();
  clearValidationState();

  started_ms = millis();
  last_ecg_sample_ms = started_ms;
  last_bioz_sample_ms = started_ms;

  Serial.println("MAX30001G ECG and BIOZ FIFO interrupt validation started.");
  Serial.println("Direct FIFO fallback is disabled during the validation window.");
  Serial.println("Attach normal ECG/BIOZ leads or a known BIOZ load for this combined-profile test.");
}

void loop() {
  if (completed) {
    delay(1000);
    return;
  }

  const bool ecg_overflow_before = ecg_overflow_occurred;
  const bool bioz_overflow_before = bioz_overflow_occurred;
  afe.update();
  if (ecg_overflow_before || ecg_overflow_occurred) {
    ecg_overflow_events++;
    ecg_overflow_occurred = false;
  }
  if (bioz_overflow_before || bioz_overflow_occurred) {
    bioz_overflow_events++;
    bioz_overflow_occurred = false;
  }
  drainEcgData();
  drainBiozData();
  drainRtorData();

  const uint32_t now = millis();
  if ((now - started_ms) >= VALIDATION_DURATION_MS) {
    const uint32_t final_status = statusWord();
    const bool pass = (ecg_samples >= EXPECTED_MIN_ECG_SAMPLES) &&
                      (bioz_samples >= EXPECTED_MIN_BIOZ_SAMPLES) &&
                      (max_ecg_gap_ms <= MAX_ALLOWED_ECG_GAP_MS) &&
                      (max_bioz_gap_ms <= MAX_ALLOWED_BIOZ_GAP_MS) &&
                      (ecg_overflow_events == 0U) &&
                      (bioz_overflow_events == 0U) &&
                      pllOk(final_status);
    printSummary(pass, final_status);
    afe.stop();
    completed = true;
  }
}
