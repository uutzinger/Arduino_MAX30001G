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

const char* ahpfLabel(uint8_t code) {
  switch (code) {
    case 255U: return "dynamic";
    case 0U: return "60Hz";
    case 1U: return "150Hz";
    case 2U: return "500Hz";
    case 3U: return "1kHz";
    case 4U: return "2kHz";
    case 5U: return "4kHz";
    case 6U:
    case 7U: return "bypass";
    default: return "unknown";
  }
}

const char* phaseRangeLabel(BIOZScanPhaseRange range) {
  return (range == BIOZ_SCAN_PHASE_REDUCED) ? "reduced(0/45/90/135 deg)" : "full";
}

float nominalFrequencyHz(uint8_t freq_idx) {
  static const float kFreqHz[MAX30001_BIOZ_NUM_FREQUENCIES] = {
    128000.0f, 80000.0f, 40000.0f, 17780.0f, 8000.0f, 4000.0f,
    2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f
  };
  if (freq_idx >= MAX30001_BIOZ_NUM_FREQUENCIES) {
    return 0.0f;
  }
  return kFreqHz[freq_idx];
}

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
  // but this sketch is typically used to validate faster scan settings.

  // Number of raw BIOZ samples averaged per phase point.
  // Typical values: 2..8. Larger values reduce noise but increase scan time.
  config.avg = 8;

  // false = low-rate BIOZ sampling (~25..32 sps depending on FMSTR)
  // true  = fast BIOZ sampling (~50..64 sps depending on FMSTR)
  config.fast = true;

  // false = 2-wire style measurement path
  // true  = 4-wire/Kelvin style measurement path
  // Internal resistor validation should normally stay false.
  config.fourleads = false;

  // Frequency index range to scan, inclusive:
  // 0=128kHz, 1=80kHz, 2=40kHz, 3=17.78kHz, 4=8kHz, 5=4kHz,
  // 6=2kHz, 7=1kHz, 8=500Hz, 9=250Hz, 10=125Hz.
  // Current default scans the full spectrum: start=0, end=10.
  // Example top-bin only scan: start=0, end=1.
  config.freq_start_index = 0;
  config.freq_end_index = 10;

  // FULL:
  //   128kHz -> 4 phases
  //   80kHz  -> 8 phases
  //   <=40kHz -> 16 phases
  // REDUCED always uses 0/45/90/135 degree logical phase steps.
  config.phase_range = BIOZ_SCAN_PHASE_REDUCED;

  // Retry count for a failed point before the scan gives up on that point.
  // 0 disables retries, 1 is a reasonable validation default.
  config.max_retries = 1;

  // Initial current seed used by the scan engine.
  // Valid selectable values are approximately:
  // 55,110,220,330,440,550,660,880,1100,8000,16000,32000,48000,64000,80000,96000 nA
  // The driver snaps to the nearest supported value and may further clamp by frequency.
  config.initial_current_nA = 8000;

  // true  = use the MAX30001G internal BIST resistor path
  // false = use the external BIOZ input path
  config.use_internal_resistor = true;

  // Internal resistor request in ohms.
  // 1000 selects the low-resistance internal BIST path used for validation.
  // Larger values request the high-resistance internal path.
  config.internal_resistor_ohm = 1000;

  // Internal-resistor AHPF override:
  // 255 = use the normal per-frequency scan table
  // 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass
  // For "same behavior as normal scan", keep 255.
  config.internal_bist_ahpf = 255U;

  // Number of samples discarded after phase/frequency/filter changes.
  // Increase this when checking whether settling is affecting a result.
  // Typical validated values: 24, 48, 64.
  config.settle_samples = 24;

  // Number of samples discarded after a drive-current change.
  // Must be >= settle_samples; the driver enforces that.
  config.current_change_settle_samples = 24;

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

  Serial.println("MAX30001G BIOZ internal-resistor scan example started.");
  Serial.print("Internal 1kOhm BIST validation scan range: ");
  Serial.print(nominalFrequencyHz(config.freq_start_index), 0);
  Serial.print("Hz down to ");
  Serial.print(nominalFrequencyHz(config.freq_end_index), 0);
  Serial.println("Hz.");
  Serial.print("Validation settings: avg=");
  Serial.print(config.avg);
  Serial.print(", fast=");
  Serial.print(config.fast ? "true" : "false");
  Serial.print(", phase_range=");
  Serial.print(phaseRangeLabel(config.phase_range));
  Serial.print(", current=");
  Serial.print(config.initial_current_nA);
  Serial.print("nA, ahpf=");
  Serial.print(ahpfLabel(config.internal_bist_ahpf));
  Serial.print(", settle=");
  Serial.print(config.settle_samples);
  Serial.print(" samples, current-change settle=");
  Serial.print(config.current_change_settle_samples);
  Serial.println(" samples.");
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
    Serial.print("BIOZ internal-resistor scan complete in ");
    Serial.print(millis() - scanStartMs);
    Serial.println(" ms.");
    return;
  }

  if ((millis() - scanStartMs) >= SCAN_TIMEOUT_MS) {
    Serial.println("BIOZ internal-resistor scan timed out before a spectrum was available.");
    reportPllStatus("Timeout");
    scanReported = true;
    afe.stop();
  }
}
