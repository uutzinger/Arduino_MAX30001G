#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 10;
const int AFE_INT1_PIN = 2;
const int AFE_INT2_PIN = -1;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
bool scanStarted = false;
bool scanReported = false;

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
}

void setup() {
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  BIOZScanConfig config;
  // BIOZScanConfig
  // avg: number of samples averaged per frequency/phase point (1..8)
  // fast: false=~30sps BIOZ, true=~60sps BIOZ
  // fourleads: true=4-wire BIOZ, false=2-wire BIOZ
  // freq_start_index / freq_end_index:
  //   0=128kHz, 1=80kHz, 2=40kHz, 3=17.78kHz, 4=8kHz, 5=4kHz,
  //   6=2kHz, 7=1kHz, 8=500Hz, 9=250Hz, 10=125Hz
  // initial_current_nA: requested starting current in nA for scan auto-ranging
  // use_internal_resistor: true switches the scan from external electrodes to the internal test resistor
  // internal_resistor_ohm: requested nominal internal resistor, nearest supported value is used
  config.avg = 2;
  config.fast = false;
  config.fourleads = false;
  config.freq_end_index = 7;
  config.initial_current_nA = 8000;
  config.use_internal_resistor = true;
  config.internal_resistor_ohm = 1000;

  afe.begin();
  afe.setupBIOZScan(config);
  afe.start();
  scanStarted = true;

  Serial.println("MAX30001G BIOZ internal-resistor scan example started.");
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
  }
}
