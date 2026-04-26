#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t  AFE_CS_PIN   =  6;
const int      AFE_INT1_PIN = 12;
const int      AFE_INT2_PIN = -1;
const uint32_t BIOZ_FIFO_POLL_MS = 250U;
const uint32_t BIOZ_DATA_TIMEOUT_MS = 1000U;
const uint32_t PLL_STARTUP_SETTLE_MS = 500U;
const uint32_t PLL_STATUS_CLEAR_MS = 10U;

// setupBIOZSignalCalibration(speed, gain)
const uint8_t BIOZ_SPEED_LOW_RATE = 0; // 0=~25..32sps, 1=~50..64sps
const uint8_t BIOZ_GAIN_20_VV = 1;     // 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V

// setTestSignal(enableECGCalSignal, enableBIOZCalSignal, unipolar, cal_vmag, freq, dutycycle)
const bool ROUTE_VCAL_TO_ECG = false;    // ECG path is not part of this BIOZ test.
const bool ROUTE_VCAL_TO_BIOZ = true;    // Route internal VCAL to BIOZP/BIOZN.
const bool VCAL_UNIPOLAR_MODE = false;   // false=bipolar, true=unipolar.
const bool VCAL_MAG_0P5_MV = true;       // false=0.25mV, true=0.5mV.
const uint8_t VCAL_FCAL_1HZ = 0b100;     // At FMSTR=32768Hz: 0=256Hz, 1=64Hz, 2=16Hz, 3=4Hz, 4=1Hz.
const uint8_t VCAL_DUTY_PERCENT = 50;    // 50 selects the dedicated 50% duty mode.

// setBIOZfilter(ahpf, lpf, hpf)
const uint8_t BIOZ_AHPF_BYPASS = 6; // 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, >=6=bypass.
const uint8_t BIOZ_DLPF_BYPASS = 0; // 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz.
const uint8_t BIOZ_DHPF_BYPASS = 0; // 0=bypass, 1=0.05Hz, 2/3=0.5Hz.

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
uint32_t last_bioz_poll_ms = 0;
uint32_t last_bioz_data_ms = 0;
bool stopped_after_failure = false;

float calibrationFrequencyHz(uint8_t fcal) {
  const uint32_t divisors[] = {
    128UL, 512UL, 2048UL, 8192UL, 32768UL, 131072UL, 524288UL, 2097152UL
  };
  return fmstr / static_cast<float>(divisors[fcal & 0x07U]);
}

void printRegister24(const char* name, uint32_t value) {
  Serial.printf("%-9s = 0x%06lX\n", name, static_cast<unsigned long>(value & 0x00FFFFFFUL));
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
    Serial.println("PLL remains unlocked after BIOZ calibration is active; check the MAX30001G external clock/oscillator.");
  }

  return !pll_unlocked;
}

bool drainBiozData() {
  float value = 0.0f;
  bool had_data = false;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    Serial.print("BIOZ_Cal_[raw]: ");
    Serial.println(value, 0);
    had_data = true;
  }
  return had_data;
}

void printBiozCalibrationConfig() {
  afe.readAllRegisters();

  Serial.println();
  Serial.println("BIOZ signal calibration setup:");
  Serial.print("setupBIOZSignalCalibration: speed=");
  Serial.print(BIOZ_SPEED_LOW_RATE);
  Serial.print(" (~32sps), gain=");
  Serial.print(BIOZ_GAIN_20_VV);
  Serial.println(" (20V/V)");
  Serial.print("setTestSignal: ECG route=");
  Serial.print(ROUTE_VCAL_TO_ECG ? "on" : "off");
  Serial.print(", BIOZ route=");
  Serial.print(ROUTE_VCAL_TO_BIOZ ? "on" : "off");
  Serial.print(", mode=");
  Serial.print(VCAL_UNIPOLAR_MODE ? "unipolar" : "bipolar");
  Serial.print(", magnitude=");
  Serial.print(VCAL_MAG_0P5_MV ? "0.5mV" : "0.25mV");
  Serial.print(", FCAL=");
  Serial.print(VCAL_FCAL_1HZ);
  Serial.print(" (");
  Serial.print(calibrationFrequencyHz(VCAL_FCAL_1HZ), 6);
  Serial.print("Hz), duty=");
  Serial.print(VCAL_DUTY_PERCENT);
  Serial.println("%");
  Serial.print("setBIOZfilter: AHPF=");
  Serial.print(BIOZ_AHPF_BYPASS);
  Serial.print(" (bypass), DLPF=");
  Serial.print(BIOZ_DLPF_BYPASS);
  Serial.print(" (bypass), DHPF=");
  Serial.print(BIOZ_DHPF_BYPASS);
  Serial.println(" (bypass)");
  Serial.println();

  Serial.println("BIOZ signal calibration registers:");
  printRegister24("STATUS", status.all);
  printRegister24("CNFG_GEN", cnfg_gen.all);
  printRegister24("CNFG_CAL", cnfg_cal.all);
  printRegister24("CNFG_BMUX", cnfg_bmux.all);
  printRegister24("CNFG_BIOZ", cnfg_bioz.all);
  printRegister24("MNGR_INT", mngr_int.all);
  printRegister24("EN_INT1", en_int1.all);
  printRegister24("EN_INT2", en_int2.all);

  Serial.print("VCAL: ");
  Serial.print(cnfg_cal.bit.vcal ? "enabled" : "disabled");
  Serial.print(", ");
  Serial.print(cnfg_cal.bit.vmode ? "bipolar" : "unipolar");
  Serial.print(", ");
  Serial.print(cnfg_cal.bit.vmag ? "0.5mV" : "0.25mV");
  Serial.print(", ");
  Serial.print(calibrationFrequencyHz(cnfg_cal.bit.fcal), 6);
  Serial.print("Hz, ");
  Serial.println(cnfg_cal.bit.fifty ? "50% duty" : "custom duty");

  Serial.print("BIOZ mux: P=");
  Serial.print(cnfg_bmux.bit.calp_sel == 0b10 ? "VCALP" : cnfg_bmux.bit.calp_sel == 0b11 ? "VCALN" : "none");
  Serial.print(", N=");
  Serial.print(cnfg_bmux.bit.caln_sel == 0b10 ? "VCALP" : cnfg_bmux.bit.caln_sel == 0b11 ? "VCALN" : "none");
  Serial.print(", external inputs ");
  Serial.println((cnfg_bmux.bit.openp && cnfg_bmux.bit.openn) ? "disconnected" : "connected");

  Serial.print("BIOZ filters: AHPF=");
  Serial.print(BIOZ_ahpf == 0.0f ? "bypass" : "active");
  Serial.print(", DLPF=");
  Serial.print(BIOZ_dlpf == 0.0f ? "bypass" : "active");
  if (BIOZ_dlpf > 0.0f) {
    Serial.print(" ");
    Serial.print(BIOZ_dlpf, 3);
    Serial.print("Hz");
  }
  Serial.print(", DHPF=");
  Serial.println(BIOZ_dhpf == 0.0f ? "bypass" : "active");
  Serial.println();
}

void setup() {
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  afe.setupBIOZSignalCalibration(BIOZ_SPEED_LOW_RATE, BIOZ_GAIN_20_VV);
  afe.setTestSignal(
    ROUTE_VCAL_TO_ECG,
    ROUTE_VCAL_TO_BIOZ,
    VCAL_UNIPOLAR_MODE,
    VCAL_MAG_0P5_MV,
    VCAL_FCAL_1HZ,
    VCAL_DUTY_PERCENT
  );
  afe.setBIOZfilter(BIOZ_AHPF_BYPASS, BIOZ_DLPF_BYPASS, BIOZ_DHPF_BYPASS);
  afe.start();
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it was latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset(); // Discard samples collected during PLL warm-up.

  Serial.println("MAX30001G BIOZ signal calibration example started.");
  reportPllStatus("Startup active");
  printBiozCalibrationConfig();
  afe.FIFOReset(); // Discard samples collected while printing diagnostics.
  last_bioz_poll_ms = millis();
  last_bioz_data_ms = millis();
}

void loop() {
  if (stopped_after_failure) {
    delay(1000);
    return;
  }

  const uint32_t now = millis();

  if ((now - last_bioz_poll_ms) >= BIOZ_FIFO_POLL_MS) {
    last_bioz_poll_ms = now;
    afe.readBIOZ_FIFO(true); //raw as we use generated signals
  }

  if (drainBiozData()) {
    last_bioz_data_ms = millis();
    return;
  }

  if ((now - last_bioz_data_ms) >= BIOZ_DATA_TIMEOUT_MS) {
    Serial.println("No raw BIOZ calibration data after 1000 ms.");
    afe.readBIOZ_FIFO(true); // raw
    if (drainBiozData()) {
      last_bioz_data_ms = millis();
      return;
    }

    Serial.println("FAIL: No raw BIOZ calibration data from BIOZ FIFO.");
    reportPllStatus("Failure");
    afe.stop();
    stopped_after_failure = true;
    Serial.println("BIOZ signal calibration stopped.");
  }
}
