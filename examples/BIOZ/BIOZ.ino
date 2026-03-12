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

  Serial.println("MAX30001G BIOZ example started.");
}

void loop() {
  if (!afe.update()) {
    return;
  }

  float value = 0.0f;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ [ohm]: ");
    Serial.println(value, 3);
  }
}
