#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t  AFE_CS_PIN   =  6;
const int      AFE_INT1_PIN = 12;
const int      AFE_INT2_PIN = -1;
const uint32_t PLL_STARTUP_SETTLE_MS =    500U;
const uint32_t PLL_STATUS_CLEAR_MS   =     10U;
const uint32_t SCAN_TIMEOUT_MS       = 180000U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
bool scanStarted = false;
bool scanReported = false;
uint32_t scanStartMs = 0U;

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
    Serial.println("PLL remains unlocked after BIOZ scan is active; check the MAX30001G external clock/oscillator.");
  }

  return !pll_unlocked;
}

void printSpectrum() {
  ImpedanceSpectrum spectrum;
  if (BIOZ_spectrum.pop(spectrum) != 1) {
    return;
  }

  Serial.println("frequency_hz,magnitude_ohm,phase_deg");
  for (uint8_t i = 0; i < MAX30001_BIOZ_NUM_FREQUENCIES; ++i) {
    if (spectrum.frequency[i] <= 0.0f) {
      continue;
    }
    Serial.print(spectrum.frequency[i], 1);
    Serial.print(',');
    Serial.print(spectrum.magnitude[i], 3);
    Serial.print(',');
    Serial.println(spectrum.phase[i], 3);
  }

  const uint32_t elapsed_ms = millis() - scanStartMs;
  Serial.print("spectrum_build_time_ms,");
  Serial.println(elapsed_ms);
}

void setup() {
  currentLogLevel = LOG_LEVEL_WARN;

  Serial.begin(115200);
  delay(1000);

  BIOZScanConfig config;
  // Same internal-BIST validation baseline as BIOZScan_Internal.ino,
  // but with fast BIOZ sampling and reduced 45 degree phase steps.
  config.avg = 8;
  config.fast = true;
  config.fourleads = false;
  config.freq_start_index = 3;
  config.freq_end_index = 7;
  config.phase_range = BIOZ_SCAN_PHASE_REDUCED;
  config.max_retries = 1;
  config.initial_current_nA = 8000;
  config.use_internal_resistor = true;
  config.internal_resistor_ohm = 1000;
  config.settle_samples = 24;
  config.current_change_settle_samples = 32;

  afe.begin();
  afe.setupBIOZScan(config);
  afe.start();
  afe.update(); // Run scan init so BIOZ/PLL are active before the warm-up status check.
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it was latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset(); // Discard samples collected during PLL warm-up.
  reportPllStatus("Startup active");
  scanStarted = true;
  scanStartMs = millis();

  Serial.println("MAX30001G BIOZ internal-resistor fast reduced-phase scan example started.");
  Serial.println("Internal 1kOhm BIST validation scan range: 17.78kHz down to 1kHz.");
  Serial.println("Validation settings: avg=8, fast=true, phase_range=reduced(0/45/90/135 deg), current=8000nA, settle=24 samples, current-change settle=32 samples.");
  Serial.println("Final spectrum table follows when the scan completes.");
}

void loop() {
  if (!scanStarted || scanReported) {
    return;
  }

  afe.update();

  if (BIOZ_spectrum.available() > 0) {
    printSpectrum();
    scanReported = true;
    afe.stop();
    Serial.print("BIOZ internal-resistor fast reduced-phase scan complete in ");
    Serial.print(millis() - scanStartMs);
    Serial.println(" ms.");
    return;
  }

  if ((millis() - scanStartMs) >= SCAN_TIMEOUT_MS) {
    Serial.println("BIOZ internal-resistor fast reduced-phase scan timed out before a spectrum was available.");
    reportPllStatus("Timeout");
    scanReported = true;
    afe.stop();
  }
}
