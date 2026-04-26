#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = -1;
const uint32_t BIOZ_DATA_TIMEOUT_MS = 500U;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_bioz_data_ms = 0;

bool drainBiozData() {
  float value = 0.0f;
  bool had_data = false;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ [ohm]: ");
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
  // setupBIOZ(speed, gain, ahpf, dlpf, dhpf, frequency, current, phase, leadbias, leadsoffdetect, fourleads)
  // speed:    0=~25..32sps, 1=~50..64sps
  // gain:     0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
  // ahpf:     0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass
  // dlpf:     0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz
  // dhpf:     0=bypass, 1=0.05Hz, 2/3=0.5Hz class
  // frequency valid targets are mapped to supported BIOZ modulation frequencies.
  // current is requested in nA and clamped/mapped by the driver.
  // phase is in degrees.
  // leadbias: true=enable subject bias, leadsoffdetect: true=enable lead-off detection
  // fourleads: true=4-wire BIOZ, false=2-wire BIOZ
  afe.setupBIOZ(0, 1, 1, 1, 0, 8000, 8000, 0.0f, true, false, false);
  afe.start();
  last_bioz_data_ms = millis();

  Serial.println("MAX30001G BIOZ example started.");
}

void loop() {
  afe.update();
  if (drainBiozData()) {
    last_bioz_data_ms = millis();
    return;
  }

  const uint32_t now = millis();
  if ((now - last_bioz_data_ms) >= BIOZ_DATA_TIMEOUT_MS) {
    Serial.println("No BIOZ data in BIOZ_data after 500 ms; switching to direct FIFO read.");
    afe.readBIOZ_FIFO(false);
    if (drainBiozData()) {
      last_bioz_data_ms = millis();
      return;
    }

    Serial.println("No BIOZ data from BIOZ_data or direct FIFO read.");
    afe.readStatusRegisters();
    afe.printStatus();
    last_bioz_data_ms = now;
  }
}
