#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t  AFE_CS_PIN   =  6;
const int      AFE_INT1_PIN = 12;
const int      AFE_INT2_PIN = -1;
const uint32_t PLL_STARTUP_SETTLE_MS  =  500U;
const uint32_t PLL_STATUS_CLEAR_MS    =   10U;
const uint32_t POINT_TIMEOUT_MS       = 2500U;
const uint32_t DIRECT_FIFO_POLL_MS    =   20U;
const uint16_t DISCARD_SAMPLES        =   24U;
const uint16_t AVERAGE_SAMPLES        =    8U;
const uint8_t  FIFO_THRESHOLD_SAMPLES =    1U;

// setupBIOZImpedanceCalibration(speed, gain, ahpf, dlpf, dhpf, frequency, current,
//                               phase, resistance, modulation, modulation_frequency)
const uint8_t  BIOZ_SPEED_LOW_RATE = 0;       // 0=~25..32sps, 1=~50..64sps
const uint8_t  BIOZ_GAIN_20_VV = 1;           // 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
const uint8_t  BIOZ_AHPF_150HZ = 1;           // 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, >=6=bypass
const uint8_t  BIOZ_DLPF_4HZ = 1;             // 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz
const uint8_t  BIOZ_DHPF_BYPASS = 0;          // 0=bypass, 1=0.05Hz, 2/3=0.5Hz
const uint32_t BIOZ_DRIVE_CURRENT_NA = 8000UL;
const uint32_t BIOZ_INTERNAL_RESISTOR_OHM = 1000UL;
const uint8_t  BIOZ_RESISTOR_MODULATION = 0;  // 0=constant resistor, 1..3=modulated resistor
const uint8_t  BIOZ_RESISTOR_MOD_FREQ = 0;    // Used only when BIOZ_RESISTOR_MODULATION is non-zero

struct InternalImpedancePoint {
  const char* label;
  uint32_t requested_frequency_hz;
  float phase_deg;
};

const InternalImpedancePoint POINTS[] = {
  {"baseline", 18204UL, 0.0f},
  {"baseline",  8000UL, 0.0f},
  {"baseline",  1024UL, 0.0f},
  {"changed_frequency_phase", 4000UL, 45.0f}
};

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

void clearBiozAcquisitionState() {
  BIOZ_data.clear();
  bioz_available = false;
  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  valid_data_detected = false;
  over_voltage_detected = false;
  under_voltage_detected = false;
  EOF_detected = false;
}

void printBiozRegisterSnapshot(const char* context, uint32_t requested_frequency_hz, float phase_deg) {
  Serial.println();
  Serial.print(context);
  Serial.print(" registers at requested ");
  Serial.print(requested_frequency_hz);
  Serial.print("Hz, phase ");
  Serial.print(phase_deg, 2);
  Serial.println("deg:");
  afe.printBiozDiagnosticRegisters(nullptr);
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
    Serial.println("PLL remains unlocked after BIOZ impedance calibration is active; check the MAX30001G external clock/oscillator.");
  }

  return !pll_unlocked;
}

void printCalibrationConfig() {
  Serial.println();
  Serial.println("BIOZ internal impedance example setup:");
  Serial.print("speed=");
  Serial.print(BIOZ_SPEED_LOW_RATE);
  Serial.print(" (~32sps), gain=");
  Serial.print(BIOZ_GAIN_20_VV);
  Serial.println(" (20V/V)");
  Serial.print("drive current=");
  Serial.print(BIOZ_DRIVE_CURRENT_NA);
  Serial.println("nA");
  Serial.print("internal resistor requested=");
  Serial.print(BIOZ_INTERNAL_RESISTOR_OHM);
  Serial.println(" ohm");
  Serial.print("samples: discard=");
  Serial.print(DISCARD_SAMPLES);
  Serial.print(", average=");
  Serial.println(AVERAGE_SAMPLES);
  Serial.println("label,requested_frequency_hz,actual_frequency_hz,phase_deg,mean_ohm,min_ohm,max_ohm,samples,status_hex,pll");
}

void configureInternalPoint(const InternalImpedancePoint& point) {
  afe.stop();
  clearBiozAcquisitionState();

  afe.setupBIOZImpedanceCalibration(
    BIOZ_SPEED_LOW_RATE,
    BIOZ_GAIN_20_VV,
    BIOZ_AHPF_150HZ,
    BIOZ_DLPF_4HZ,
    BIOZ_DHPF_BYPASS,
    point.requested_frequency_hz,
    BIOZ_DRIVE_CURRENT_NA,
    point.phase_deg,
    BIOZ_INTERNAL_RESISTOR_OHM,
    BIOZ_RESISTOR_MODULATION,
    BIOZ_RESISTOR_MOD_FREQ
  );

  afe.setFIFOInterruptThreshold(32U, FIFO_THRESHOLD_SAMPLES);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters();
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset();
  afe.synch();
  clearBiozAcquisitionState();
}

bool collectInternalPoint(float& mean_ohm, float& min_ohm, float& max_ohm, uint16_t& samples) {
  mean_ohm = 0.0f;
  min_ohm = 0.0f;
  max_ohm = 0.0f;
  samples = 0U;

  const uint16_t total_needed = static_cast<uint16_t>(DISCARD_SAMPLES + AVERAGE_SAMPLES);
  uint16_t total_seen = 0U;
  float sum_ohm = 0.0f;
  uint32_t last_direct_read_ms = 0U;
  const uint32_t start_ms = millis();

  while (((millis() - start_ms) < POINT_TIMEOUT_MS) && (samples < AVERAGE_SAMPLES)) {
    afe.update(false);

    const uint32_t now = millis();
    if ((now - last_direct_read_ms) >= DIRECT_FIFO_POLL_MS) {
      afe.readBIOZ_FIFO(false);
      last_direct_read_ms = now;
    }

    float value_ohm = 0.0f;
    while ((BIOZ_data.available() > 0U) && (total_seen < total_needed)) {
      if (BIOZ_data.pop(value_ohm) != 1U) {
        break;
      }

      total_seen++;
      if (total_seen <= DISCARD_SAMPLES) {
        continue;
      }

      if (samples == 0U) {
        min_ohm = value_ohm;
        max_ohm = value_ohm;
      } else {
        if (value_ohm < min_ohm) {
          min_ohm = value_ohm;
        }
        if (value_ohm > max_ohm) {
          max_ohm = value_ohm;
        }
      }

      sum_ohm += value_ohm;
      samples++;
    }

    delay(2);
  }

  if (samples > 0U) {
    mean_ohm = sum_ohm / static_cast<float>(samples);
  }

  return samples == AVERAGE_SAMPLES;
}

void printPointResult(const InternalImpedancePoint& point) {
  configureInternalPoint(point);
  printBiozRegisterSnapshot("BIOZ internal impedance", point.requested_frequency_hz, point.phase_deg);

  float mean_ohm = 0.0f;
  float min_ohm = 0.0f;
  float max_ohm = 0.0f;
  uint16_t samples = 0U;
  const bool complete = collectInternalPoint(mean_ohm, min_ohm, max_ohm, samples);

  afe.readStatusRegisters();
  const uint32_t status_word = status.all & 0x00FFFFFFUL;
  const bool pll_unlocked = ((status_word & MAX30001_STATUS_PLLINT) != 0U);

  Serial.print(point.label);
  Serial.print(",");
  Serial.print(point.requested_frequency_hz);
  Serial.print(",");
  Serial.print(BIOZ_frequency, 1);
  Serial.print(",");
  Serial.print(point.phase_deg, 2);
  Serial.print(",");
  Serial.print(mean_ohm, 3);
  Serial.print(",");
  Serial.print(min_ohm, 3);
  Serial.print(",");
  Serial.print(max_ohm, 3);
  Serial.print(",");
  Serial.print(samples);
  Serial.print(",0x");
  Serial.print(status_word, HEX);
  Serial.print(",");
  Serial.print(pll_unlocked ? "UNLOCKED" : "OK");
  if (!complete) {
    Serial.print(",INCOMPLETE");
  }
  Serial.println();
}

void runCalibration() {
  printCalibrationConfig();

  for (uint8_t i = 0U; i < (sizeof(POINTS) / sizeof(POINTS[0])); ++i) {
    printPointResult(POINTS[i]);
  }

  afe.stop();
  Serial.println("BIOZ internal impedance example complete.");
}

void setup() {
  currentLogLevel = LOG_LEVEL_WARN;

  Serial.begin(115200);
  delay(1000);

  afe.begin();

  Serial.println("MAX30001G BIOZ internal impedance example started.");
  reportPllStatus("Startup idle");
  runCalibration();
}

void loop() {
  delay(1000);
}
