#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 10;
const int AFE_INT1_PIN = 2;
const int AFE_INT2_PIN = -1;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

void setup() {
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  // setupECGandBIOZ(
  //   ecg_speed, ecg_gain, ecg_threeleads,
  //   bioz_speed, bioz_gain, bioz_dlpf, bioz_dhpf,
  //   bioz_frequency, bioz_current, bioz_phase,
  //   leadbias, leadsoffdetect, bioz_fourleads)
  //
  // ecg_speed: 0=~125sps, 1=~256sps, 2=~512sps
  // ecg_gain:  0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
  // ecg_threeleads: true=3-lead ECG, false=2-lead ECG
  // bioz_speed: 0=~25..32sps, 1=~50..64sps
  // bioz_gain:  0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
  // bioz_dlpf:  0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz
  // bioz_dhpf:  0=bypass, 1=0.05Hz, 2/3=0.5Hz class
  // bioz_frequency: requested modulation frequency in Hz
  // bioz_current: requested drive current in nA
  // bioz_phase: demodulation phase in degrees
  // leadbias: true=enable subject bias
  // leadsoffdetect: true=enable lead-off detection
  // bioz_fourleads: true=4-wire BIOZ, false=2-wire BIOZ
  afe.setupECGandBIOZ(
    1, 2, true,
    0, 1,
    1, 0,
    8000, 8000, 0.0f,
    true, false, false
  );
  afe.start();

  Serial.println("MAX30001G ECG and BIOZ example started.");
}

void loop() {
  if (!afe.update()) {
    return;
  }

  float value = 0.0f;

  while (ECG_data.available() > 0) {
    ECG_data.pop(value);
    Serial.print("ECG [mV]: ");
    Serial.println(value, 3);
  }

  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ [ohm]: ");
    Serial.println(value, 3);
  }

  while (RTOR_data.available() > 0) {
    RTOR_data.pop(value);
    Serial.print("RR [ms]: ");
    Serial.println(value, 1);
    if (value > 0.0f) {
      Serial.print("HR [bpm]: ");
      Serial.println(60000.0f / value, 1);
    }
  }
}
