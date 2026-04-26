/******************************************************************************************************/
// MAX30001G Interactive Program
// 
// Allows user to experiment with ECG and BIOZ settings for the MAX30001G AFE at runtime through a
// serial terminal command-line interface.
//
// This program demonstrates:
//   - Initialization and configuration of MAX30001G AFE
//   - Switching between operation modes (ECG, BIOZ, combined, calibration, scanning)
//   - Register inspection and modification
//   - Health checks and diagnostics
//   - Real-time data acquisition and display
//   - Testing all library features interactively
//
// Operation Modes:
//   - ECG:         Single-channel ECG acquisition (2-lead or 3-lead)
//   - BIOZ:        Bio-impedance  measurement
//   - ECG+BIOZ:    Simultaneous ECG and BIOZ
//   - ECG Cal:     ECG signal calibration (internal)
//   - BIOZ Cal:    BIOZ signal calibration (internal)
//   - BIOZ Int Cal: Internal impedance calibration with known resistor
//   - BIOZ Ext Cal: External impedance calibration for custom resistors
//   - BIOZ Scan:   Impedance spectroscopy frequency sweep
//
// Command Interface:
//   Type '?' for help menu listing all commands
//   Single-letter commands control operation modes and settings
//   Commands can be followed by numbers to set parameter values
//
// Urs Utzinger, 2026
// GPT-05.3 assisted development
/******************************************************************************************************/

#include <Arduino.h>
#include <stdlib.h> 
#include "max30001g.h"
#include "logger.h"

// Default Log Level (can be changed at runtime)
#define LOG_LEVEL LOG_LEVEL_INFO 

// Serial Communication
#define BAUD_RATE 115200  

// Hardware Pin Connections
// Adjust these to match your hardware configuration
const uint8_t AFE_CS_PIN  = 6;    // SPI chip select
const int AFE_INT1_PIN    = 12;   // INTB interrupt pin
const int AFE_INT2_PIN    = 13;   // INT2B interrupt pin

/******************************************************************************************************/
// Operation Modes
/******************************************************************************************************/
enum OperationMode {
  MODE_IDLE = 0,
  MODE_ECG,
  MODE_BIOZ,
  MODE_ECG_BIOZ,
  MODE_ECG_CAL,
  MODE_BIOZ_CAL,
  MODE_BIOZ_INTERNAL_CAL,
  MODE_BIOZ_EXTERNAL_CAL, 
  MODE_BIOZ_SCAN
};

/******************************************************************************************************/
// Global Configuration Variables
/******************************************************************************************************/
OperationMode current_mode = MODE_IDLE;
bool measurement_running = false;
bool data_reporting = false;  // Continuous data reporting  

// ECG Configuration
uint8_t ecg_speed = 1;        // 0=~125sps, 1=~256sps, 2=~512sps
uint8_t ecg_gain = 2;         // 0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
bool ecg_threeleads = true;   // true=3-lead, false=2-lead
bool ecg_rtor = true;         // Enable R-to-R detection

// BIOZ Configuration  
uint8_t bioz_speed = 0;       // 0=low-rate (~25-32sps), 1=high-rate (~50-64sps)
uint8_t bioz_gain = 1;        // 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
uint8_t bioz_ahpf = 1;        // Analog HPF: 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass
uint8_t bioz_dlpf = 1;        // Digital LPF: 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz
uint8_t bioz_dhpf = 0;        // Digital HPF: 0=bypass, 1=0.05Hz, 2/3=0.5Hz
uint32_t bioz_frequency = 8000;   // Modulation frequency in Hz
uint32_t bioz_current = 8000;     // Drive current in nA
float bioz_phase = 0.0f;          // Demodulation phase offset in degrees
bool bioz_leadbias = true;        // Enable lead bias
bool bioz_leadsoffdetect = false; // Enable lead-off detection
bool bioz_fourleads = false;      // 4-wire vs 2-wire BIOZ

// BIOZ Scan Configuration
uint8_t scan_avg = 8;         // Number of averages per frequency point (1-8)
bool scan_fast = false;       // Fast scan mode (60sps vs 30sps)
bool scan_fullrange = false;  // Include lowest frequencies
bool scan_internal = false;   // false=external scan, true=internal resistor scan
BIOZScanPhaseRange scan_phase_range = BIOZ_SCAN_PHASE_FULL; // FULL or REDUCED
uint8_t scan_internal_bist_ahpf = 255U; // 255=dynamic table, 0..7=fixed internal-BIST AHPF
uint8_t scan_settle_samples = 24U; // Samples to discard after phase/frequency/filter changes
uint8_t scan_current_change_settle_samples = 24U; // Samples to discard after current changes

// BIOZ Calibration
uint32_t cal_resistance = 1000;  // Internal calibration resistor value (Ohms)
uint8_t cal_modulation = 0;      // Calibration modulation mode
uint8_t cal_mod_freq = 3;        // Calibration modulation frequency

// BIOZ signal calibration uses internal VCAL voltage, not drive current.
// Keep this mode raw because Ohm conversion divides by BIOZ current magnitude.
const bool BIOZ_CAL_ROUTE_ECG = false;
const bool BIOZ_CAL_ROUTE_BIOZ = true;
const bool BIOZ_CAL_UNIPOLAR = false; // false=bipolar, true=unipolar
const bool BIOZ_CAL_0P5_MV = true;    // false=0.25mV, true=0.5mV
const uint8_t BIOZ_CAL_FCAL_1HZ = 0b100;
const uint8_t BIOZ_CAL_DUTY_PERCENT = 50;
const uint8_t BIOZ_CAL_AHPF_BYPASS = 6;
const uint8_t BIOZ_CAL_DLPF_BYPASS = 0;
const uint8_t BIOZ_CAL_DHPF_BYPASS = 0;

// Program Globals
uint32_t current_time;
uint32_t last_time = 0;
uint32_t sample_count = 0; 
uint32_t last_sample_count = 0;
float sampling_rate = 0.0f;
char serial_input_buff[128];
uint8_t serial_index = 0;
bool serial_command_complete = false;
const char waitmsg[] = {"Waiting for serial terminal (5 seconds, press Enter to skip)"};
extern int currentLogLevel;

// Instantiate the MAX30001G driver
MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

/******************************************************************************************************/
// Helper Functions
/******************************************************************************************************/

// Boot helper - prints message and waits until timeout or user input
void serialTrigger(const char* mess, int timeout) {
  uint32_t startTime = millis();
  Serial.println();
  Serial.println(mess);
  while (!Serial.available() && ((millis() - startTime) < timeout)) {
    delay(500);
  }
  // Clear the serial input buffer
  while (Serial.available()) { Serial.read(); }
}

// Print Mode Name
const char* getModeName(OperationMode mode) {
  switch (mode) {
    case MODE_IDLE:               return "IDLE";
    case MODE_ECG:                return "ECG";
    case MODE_BIOZ:               return "BIOZ";
    case MODE_ECG_BIOZ:           return "ECG+BIOZ";
    case MODE_ECG_CAL:            return "ECG Calibration";
    case MODE_BIOZ_CAL:           return "BIOZ Calibration";
    case MODE_BIOZ_INTERNAL_CAL:  return "BIOZ Internal Cal";
    case MODE_BIOZ_EXTERNAL_CAL:  return "BIOZ External Cal";
    case MODE_BIOZ_SCAN:          return "BIOZ Scan";
    default:                      return "UNKNOWN";
  }
}

const char* onOffLabel(bool enabled) {
  return enabled ? "ON" : "OFF";
}

const char* scanPhaseRangeLabel(BIOZScanPhaseRange range) {
  return (range == BIOZ_SCAN_PHASE_REDUCED) ? "reduced(0/45/90/135 deg)" : "full";
}

float requestedEcgSamplingRateSps(uint8_t selector) {
  switch (selector) {
    case 0: return 125.0f;
    case 1: return 256.0f;
    case 2: return 512.0f;
    default: return 0.0f;
  }
}

float requestedBiozSamplingRateSps(uint8_t selector) {
  return (selector == 0U) ? 32.0f : 64.0f;
}

float requestedBiozAhpfHz(uint8_t selector) {
  switch (selector) {
    case 0: return 60.0f;
    case 1: return 150.0f;
    case 2: return 500.0f;
    case 3: return 1000.0f;
    case 4: return 2000.0f;
    case 5: return 4000.0f;
    default: return 0.0f;
  }
}

float requestedBiozDlpfHz(uint8_t selector) {
  switch (selector) {
    case 1: return 4.0f;
    case 2: return 8.0f;
    case 3: return 16.0f;
    default: return 0.0f;
  }
}

float requestedBiozDhpfHz(uint8_t selector) {
  switch (selector) {
    case 1: return 0.05f;
    case 2: return 0.5f;
    default: return 0.0f;
  }
}

float requestedCalibrationModulationFrequencyHz(uint8_t selector) {
  switch (selector) {
    case 1: return 0.0625f;
    case 2: return 0.25f;
    case 3: return 1.0f;
    case 4: return 4.0f;
    default: return 4.0f;
  }
}

float requestedCalibrationModulationOhm(uint32_t resistance, uint8_t modulation) {
  if (modulation == 0U || resistance > 5000U) {
    return 0.0f;
  }

  const uint16_t rnom_thresholds[] = {2500U, 1667U, 1250U, 1000U, 833U, 714U, 625U, 0U};
  const float rmod_table[8][4] = {
    {0.0f, 247.5f, 980.6f, 2960.7f},
    {0.0f,  61.9f, 245.2f,  740.4f},
    {0.0f,  27.5f, 109.0f,  329.1f},
    {0.0f,   0.0f,  61.3f,  185.1f},
    {0.0f,   0.0f,  39.2f,  118.5f},
    {0.0f,   0.0f,  27.2f,   82.3f},
    {0.0f,   0.0f,  20.0f,   60.5f},
    {0.0f,   0.0f,  15.3f,   46.3f}
  };

  uint8_t row = 7U;
  for (uint8_t i = 0U; i < 8U; ++i) {
    if (resistance > rnom_thresholds[i]) {
      row = i;
      break;
    }
  }

  if (modulation > 3U) {
    modulation = 3U;
  }
  return rmod_table[row][modulation];
}

const char* scanAhpfOverrideLabel(uint8_t selector) {
  switch (selector) {
    case 255U: return "dynamic";
    case 0U: return "60 Hz";
    case 1U: return "150 Hz";
    case 2U: return "500 Hz";
    case 3U: return "1 kHz";
    case 4U: return "2 kHz";
    case 5U: return "4 kHz";
    case 6U:
    case 7U: return "bypass";
    default: return "unknown";
  }
}

void printHzOrBypass(float hz, uint8_t decimals = 2U) {
  if (hz <= 0.0f) {
    Serial.print("bypass");
  } else {
    Serial.print(hz, decimals);
    Serial.print(" Hz");
  }
}

void printAppliedBiozSummary() {
  Serial.print("  Applied BIOZ: ");
  Serial.print(BIOZ_samplingRate, 1);
  Serial.print(" sps, ");
  Serial.print(BIOZ_gain);
  Serial.print(" V/V, AHPF ");
  printHzOrBypass(BIOZ_ahpf, 0U);
  Serial.print(", DLPF ");
  printHzOrBypass(BIOZ_dlpf, 2U);
  Serial.print(", DHPF ");
  printHzOrBypass(BIOZ_dhpf, 2U);
  Serial.print(", ");
  Serial.print(BIOZ_frequency, 1);
  Serial.print(" Hz, ");
  Serial.print(BIOZ_cgmag);
  Serial.print(" nA, ");
  Serial.print(BIOZ_phase, 2);
  Serial.println(" deg");
}

void printAppliedEcgSummary() {
  Serial.print("  Applied ECG: ");
  Serial.print(ECG_samplingRate, 1);
  Serial.print(" sps, ");
  Serial.print(ECG_gain);
  Serial.print(" V/V, HPF ");
  printHzOrBypass(ECG_hpf, 2U);
  Serial.print(", LPF ");
  printHzOrBypass(ECG_lpf, 0U);
  Serial.println();
}

// Help Menu
void helpMenu() {
  Serial.println("================================================================================");
  Serial.println("| MAX30001G ECG and Bio-Impedance Program                                      |");  
  Serial.println("| 2026 Urs Utzinger & GPT                                                      |");
  Serial.println("================================================================================");
  Serial.println("| GENERAL COMMANDS                       | DATA COMMANDS                       |");
  Serial.println("|----------------------------------------|-------------------------------------|");
  Serial.println("| ?: help screen                         | z: toggle data display on/off       |");
  Serial.println("| s: show current settings               | c: reset sample counter             |");
  Serial.println("| h: run health check                    | (: save config snapshot             |");
  Serial.println("| i: print device info                   | ): restore config snapshot          |");
  Serial.println("| r: print all registers                 | p: print config registers           |");
  Serial.println("| t: print status registers              | f: FIFO reset                       |");
  Serial.println("|========================================|=====================================|");
  Serial.println("| OPERATION MODES (auto-stop previous)   | START/STOP                          |");
  Serial.println("|----------------------------------------|-------------------------------------|");
  Serial.println("| m1: ECG mode                           | .: toggle start/stop                |");
  Serial.println("| m2: BIOZ mode                          | >: start measurement                |");
  Serial.println("| m3: ECG + BIOZ mode                    | <: stop measurement                 |");
  Serial.println("| m4: ECG calibration                    |                                     |");
  Serial.println("| m5: BIOZ calibration                   |                                     |");
  Serial.println("| m6: BIOZ internal cal                  |                                     |");
  Serial.println("| m7: BIOZ external cal                  |                                     |");
  Serial.println("| m8: BIOZ scan                          |                                     |");
  Serial.println("|========================================|=====================================|");
  Serial.println("| ECG SETTINGS                           | BIOZ SETTINGS                       |");
  Serial.println("|----------------------------------------|-------------------------------------|");
  Serial.println("| Es<n>: speed      (0-2)     Es1        | Bs<n>: speed      (0-1)     Bs0     |");
  Serial.println("| Eg<n>: gain       (0-3)     Eg2        | Bg<n>: gain       (0-3)     Bg1     |");
  Serial.println("| El<n>: leads      (2 or 3)  El3        | Ba<n>: analog HPF (0-7)     Ba1     |");
  Serial.println("| Er<n>: R-to-R     (0=off,1) Er1        | Bd<n>: digital LPF(0-3)     Bd1     |");
  Serial.println("|                                        | Bh<n>: digital HPF(0-3)     Bh0     |");
  Serial.println("|                                        | Bf<n>: frequency Hz         Bf8000  |");
  Serial.println("|                                        | Bc<n>: current nA           Bc8000  |");
  Serial.println("|                                        | Bp<n>: phase deg            Bp0     |");
  Serial.println("|                                        | Bl<n>: lead bias  (0=off,1) Bl1     |");
  Serial.println("|                                        | Bo<n>: lead-off   (0=off,1) Bo0     |");
  Serial.println("|                                        | Bw<n>: wires      (2 or 4)  Bw2     |");
  Serial.println("|========================================|=====================================|");
  Serial.println("| SCAN SETTINGS                          | CALIBRATION SETTINGS                |");
  Serial.println("|----------------------------------------|-------------------------------------|");
  Serial.println("| Sa<n>: averages   (1-8)     Sa8        | Cr<n>: internal resistor    Cr1000  |");
  Serial.println("| Sf<n>: fast mode  (0=off,1) Sf0        | Cm<n>: cal modulation(0-3)  Cm0     |");
  Serial.println("| Sr<n>: full range (0=off,1) Sr0        | Cf<n>: mod frequency(0-4)   Cf3     |");
  Serial.println("| Si<n>: source     (0=ext,1=int) Si0    |                                     |");
  Serial.println("| Sp<n>: phase rng  (0=full,1) Sp0       |                                     |");
  Serial.println("| Sh<n>: int AHPF   (255,0-7) Sh255      |                                     |");
  Serial.println("| St<n>: settle     (1-64)    St24       |                                     |");
  Serial.println("| Sc<n>: cur settle (1-64)    Sc24       |                                     |");
  Serial.println("|========================================|=====================================|");
  Serial.println("| LOG LEVEL                              | SPECIAL                             |");
  Serial.println("|----------------------------------------|-------------------------------------|");
  Serial.println("| l0: none (silent)                      | w: software reset                   |");
  Serial.println("| l1: errors only                        | y: synchronize                      |");
  Serial.println("| l2: warnings                           | k: clear latched status flags       |");
  Serial.println("| l3: info (default)                     | a: apply current settings (re-setup)|");
  Serial.println("| l4: debug (verbose)                    |                                     |");
  Serial.println("================================================================================");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  m1       - Switch to ECG mode");
  Serial.println("  Eg3      - Set ECG gain to 160 V/V (level 3)");
  Serial.println("  Bf40000  - Set BIOZ frequency to 40 kHz");
  Serial.println("  .        - Start/stop measurement");
  Serial.println("  z        - Toggle continuous data display");
  Serial.println();
}

// Show Current Settings
void showSettings() {
  Serial.println("================================================================================");
  Serial.println("Current Settings:");
  Serial.println("================================================================================");
  Serial.print("Mode: ");
  Serial.print(getModeName(current_mode));
  Serial.print(" | Running: ");
  Serial.print(measurement_running ? "YES" : "NO");
  Serial.print(" | Data Display: ");
  Serial.println(data_reporting ? "ON" : "OFF");
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("ECG Settings:");
  Serial.print("  Requested Speed: ~");
  Serial.print(requestedEcgSamplingRateSps(ecg_speed), 0);
  Serial.print(" sps (selector ");
  Serial.print(ecg_speed);
  Serial.println(")");
  Serial.print("  Requested Gain: ");
  Serial.print(20 << ecg_gain);
  Serial.print(" V/V (selector ");
  Serial.print(ecg_gain);
  Serial.println(")");
  Serial.print("  Leads: ");
  Serial.println(ecg_threeleads ? "3-lead" : "2-lead");
  Serial.print("  R-to-R: ");
  Serial.println(onOffLabel(ecg_rtor));
  if (current_mode == MODE_ECG || current_mode == MODE_ECG_BIOZ || current_mode == MODE_ECG_CAL) {
    printAppliedEcgSummary();
  }
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("BIOZ Settings:");
  Serial.print("  Requested Speed: ~");
  Serial.print(requestedBiozSamplingRateSps(bioz_speed), 0);
  Serial.print(" sps (selector ");
  Serial.print(bioz_speed);
  Serial.println(")");
  Serial.print("  Requested Gain: ");
  Serial.print(10 * (1 << bioz_gain));
  Serial.print(" V/V (selector ");
  Serial.print(bioz_gain);
  Serial.println(")");
  Serial.print("  Requested AHPF: ");
  printHzOrBypass(requestedBiozAhpfHz(bioz_ahpf), 0U);
  Serial.print(" (selector ");
  Serial.print(bioz_ahpf);
  Serial.print(") | Requested DLPF: ");
  printHzOrBypass(requestedBiozDlpfHz(bioz_dlpf), 0U);
  Serial.print(" (selector ");
  Serial.print(bioz_dlpf);
  Serial.print(") | Requested DHPF: ");
  printHzOrBypass(requestedBiozDhpfHz(bioz_dhpf), 2U);
  Serial.print(" (selector ");
  Serial.print(bioz_dhpf);
  Serial.println(")");
  Serial.print("  Requested Frequency: ");
  Serial.print(bioz_frequency);
  Serial.print(" Hz | Requested Current: ");
  Serial.print(bioz_current);
  Serial.print(" nA | Requested Phase: ");
  Serial.print(bioz_phase, 1);
  Serial.println(" deg");
  Serial.print("  Lead Bias: ");
  Serial.print(onOffLabel(bioz_leadbias));
  Serial.print(" | Lead-Off Detect: ");
  Serial.print(onOffLabel(bioz_leadsoffdetect));
  Serial.print(" | Wires: ");
  Serial.println(bioz_fourleads ? "4-wire" : "2-wire");
  if (current_mode == MODE_BIOZ || current_mode == MODE_ECG_BIOZ ||
      current_mode == MODE_BIOZ_CAL || current_mode == MODE_BIOZ_INTERNAL_CAL ||
      current_mode == MODE_BIOZ_EXTERNAL_CAL) {
    printAppliedBiozSummary();
  }
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("Scan Settings:");
  Serial.print("  Averages: ");
  Serial.print(scan_avg);
  Serial.print(" | Fast Mode: ");
  Serial.print(scan_fast ? "ON" : "OFF");
  Serial.print(" | Full Range: ");
  Serial.print(scan_fullrange ? "ON" : "OFF");
  Serial.print(" | Source: ");
  Serial.println(scan_internal ? "Internal Resistor" : "External");
  Serial.print("  Phase Range: ");
  Serial.print(scanPhaseRangeLabel(scan_phase_range));
  Serial.print(" | Internal AHPF: ");
  Serial.print(scanAhpfOverrideLabel(scan_internal_bist_ahpf));
  Serial.print(" | Settle: ");
  Serial.print(scan_settle_samples);
  Serial.print(" | Current Settle: ");
  Serial.println(scan_current_change_settle_samples);
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("Calibration Settings:");
  Serial.print("  Requested Resistor: ");
  Serial.print(cal_resistance);
  Serial.print(" Ohm | Requested Modulation: ");
  if (cal_modulation == 0U) {
    Serial.print("DC only");
  } else {
    Serial.print(requestedCalibrationModulationOhm(cal_resistance, cal_modulation), 1);
    Serial.print(" Ohm step");
  }
  Serial.print(" (selector ");
  Serial.print(cal_modulation);
  Serial.print(") | Requested Mod Frequency: ");
  Serial.print(requestedCalibrationModulationFrequencyHz(cal_mod_freq), 4);
  Serial.print(" Hz (selector ");
  Serial.print(cal_mod_freq);
  Serial.println(")");
  if (current_mode == MODE_BIOZ_INTERNAL_CAL) {
    Serial.print("  Applied BIST: RNOM ");
    Serial.print(BIOZ_test_rnom, 1);
    Serial.print(" Ohm, RMOD ");
    Serial.print(BIOZ_test_rmod, 1);
    Serial.print(" Ohm, RMOD freq ");
    Serial.print(BIOZ_test_frequency, 4);
    Serial.println(" Hz");
  }
  Serial.println("================================================================================");
}

// Apply current settings to AFE (setup based on mode)
void applySettings() {
  if (measurement_running) {
    Serial.println("Stopping measurement before applying new settings...");
    afe.stop();
    measurement_running = false;
    delay(100);
  }

  Serial.print("Applying settings for mode: ");
  Serial.println(getModeName(current_mode));

  switch (current_mode) {
    case MODE_ECG:
      afe.setupECG(ecg_speed, ecg_gain, ecg_threeleads);
      Serial.println("ECG mode configured");
      printAppliedEcgSummary();
      break;

    case MODE_BIOZ:
      afe.setupBIOZ(bioz_speed, bioz_gain, bioz_ahpf, bioz_dlpf, bioz_dhpf,
                    bioz_frequency, bioz_current, bioz_phase,
                    bioz_leadbias, bioz_leadsoffdetect, bioz_fourleads);
      Serial.println("BIOZ mode configured");
      printAppliedBiozSummary();
      break;

    case MODE_ECG_BIOZ:
      afe.setupECGandBIOZ(ecg_speed, ecg_gain, ecg_threeleads,
                          bioz_speed, bioz_gain, bioz_dlpf, bioz_dhpf,
                          bioz_frequency, bioz_current, bioz_phase,
                          bioz_leadbias, bioz_leadsoffdetect, bioz_fourleads);
      Serial.println("ECG+BIOZ mode configured");
      printAppliedEcgSummary();
      printAppliedBiozSummary();
      break;

    case MODE_ECG_CAL:
      afe.setupECGSignalCalibration(ecg_speed, ecg_gain);
      Serial.println("ECG calibration mode configured");
      printAppliedEcgSummary();
      break;

    case MODE_BIOZ_CAL:
      afe.setupBIOZSignalCalibration(bioz_speed, bioz_gain);
      afe.setTestSignal(BIOZ_CAL_ROUTE_ECG, BIOZ_CAL_ROUTE_BIOZ, BIOZ_CAL_UNIPOLAR,
                        BIOZ_CAL_0P5_MV, BIOZ_CAL_FCAL_1HZ, BIOZ_CAL_DUTY_PERCENT);
      afe.setBIOZfilter(BIOZ_CAL_AHPF_BYPASS, BIOZ_CAL_DLPF_BYPASS, BIOZ_CAL_DHPF_BYPASS);
      Serial.println("BIOZ calibration mode configured: raw bipolar 0.5mV VCAL at ~1Hz, filters bypassed");
      printAppliedBiozSummary();
      break;

    case MODE_BIOZ_INTERNAL_CAL:
      afe.setupBIOZImpedanceCalibration(bioz_speed, bioz_gain, bioz_ahpf, bioz_dlpf, bioz_dhpf,
                                         bioz_frequency, bioz_current, bioz_phase,
                                         cal_resistance, cal_modulation, cal_mod_freq);
      Serial.println("BIOZ internal calibration mode configured");
      printAppliedBiozSummary();
      Serial.print("  Applied BIST: RNOM ");
      Serial.print(BIOZ_test_rnom, 1);
      Serial.print(" Ohm, RMOD ");
      Serial.print(BIOZ_test_rmod, 1);
      Serial.print(" Ohm, RMOD freq ");
      Serial.print(BIOZ_test_frequency, 4);
      Serial.println(" Hz");
      break;

    case MODE_BIOZ_EXTERNAL_CAL:
      afe.setupBIOZExternalImpedanceCalibration(bioz_frequency, bioz_phase);
      Serial.println("BIOZ external calibration mode configured");
      printAppliedBiozSummary();
      break;

    case MODE_BIOZ_SCAN:
      {
        BIOZScanConfig config;
        config.avg = scan_avg;
        config.fast = scan_fast;
        config.fourleads = bioz_fourleads;
        config.freq_end_index = scan_fullrange ? (MAX30001_BIOZ_NUM_FREQUENCIES - 1U) : 7U;
        config.phase_range = scan_phase_range;
        config.use_internal_resistor = scan_internal;
        config.internal_resistor_ohm = static_cast<uint16_t>(cal_resistance);
        config.internal_bist_ahpf = scan_internal_bist_ahpf;
        config.initial_current_nA = bioz_current;
        config.settle_samples = scan_settle_samples;
        config.current_change_settle_samples = scan_current_change_settle_samples;
        afe.setupBIOZScan(config);
        Serial.print("BIOZ scan mode configured (");
        Serial.print(scan_internal ? "internal resistor" : "external electrodes");
        Serial.println(")");
        Serial.print("  Requested scan start current: ");
        Serial.print(config.initial_current_nA);
        Serial.print(" nA | phase range: ");
        Serial.print(scanPhaseRangeLabel(config.phase_range));
        Serial.print(" | internal AHPF: ");
        Serial.print(scanAhpfOverrideLabel(config.internal_bist_ahpf));
        Serial.print(" | settle: ");
        Serial.print(config.settle_samples);
        Serial.print(" samples | current-change settle: ");
        Serial.print(config.current_change_settle_samples);
        Serial.println(" samples");
      }
      break;

    case MODE_IDLE:
      afe.enableAFE(false, false, false);
      Serial.println("IDLE mode - AFE disabled");
      break;

    default:
      Serial.println("Unknown mode");
      break;
  }
}

/******************************************************************************************************/
// Command Handlers
/******************************************************************************************************/

void handleGeneralCommand(char cmd) {
  switch (cmd) {
    case '?':  // Help
      helpMenu();
      break;

    case 's':  // Show settings
      showSettings();
      break;

    case 'h':  // Health check
      {
        Serial.println("Running health check...");
        HealthCheckResult result = afe.healthCheck();
        const bool info_ok = (result.info_reg != 0U);
        const bool status_ok = !result.fault_present;
        Serial.print("SPI OK: ");
        Serial.println(result.spi_ok ? "YES" : "NO");
        Serial.print("INFO OK: ");
        Serial.println(info_ok ? "YES" : "NO");
        Serial.print("STATUS OK: ");
        Serial.println(status_ok ? "YES" : "NO");
        Serial.print("INFO REG: 0x");
        Serial.println(result.info_reg, HEX);
        Serial.print("STATUS REG: 0x");
        Serial.println(result.status_reg, HEX);
        Serial.print("PLL Unlocked: ");
        Serial.println(result.pll_unlocked ? "YES" : "NO");
        Serial.print("Fault Present: ");
        Serial.println(result.fault_present ? "YES" : "NO");
        Serial.print("Overall: ");
        Serial.println(result.spi_ok && info_ok && status_ok ? "PASS" : "FAIL");
      }
      break;

    case 'i':  // Print device info
      afe.readInfo();
      afe.printInfo();
      break;

    case 'r':  // Print all registers
      afe.readAllRegisters();
      afe.printAllRegisters();
      break;

    case 't':  // Print status
      afe.readStatusRegisters();
      afe.printStatus();
      break;

    case 'p':  // Print config registers
      afe.printConfig();
      break;

    case '(':  // Save config
      afe.saveConfig();
      Serial.println("Configuration saved to snapshot");
      break;

    case ')':  // Restore config
      afe.restoreConfig();
      Serial.println("Configuration restored from snapshot");
      break;

    case 'w':  // Software reset
      Serial.println("Performing software reset...");
      afe.swReset();
      delay(500);
      Serial.println("Reset complete. Reinitialize before use.");
      measurement_running = false;
      current_mode = MODE_IDLE;
      break;

    case 'y':  // Synchronize
      Serial.println("Synchronizing...");
      afe.synch();
      Serial.println("Sync complete");
      break;

    case 'k':  // Clear latched status flags
      afe.clearLatchedStatusFlags();
      Serial.println("Latched status flags cleared");
      break;

    case 'f':  // FIFO reset
      afe.FIFOReset();
      Serial.println("FIFO reset");
      break;

    case 'c':  // Reset sample counter
      sample_count = 0;
      last_sample_count = 0;
      last_time = millis();
      Serial.println("Sample counter reset");
      break;

    case 'z':  // Toggle data display
      data_reporting = !data_reporting;
      Serial.print("Continuous data display: ");
      Serial.println(data_reporting ? "ON" : "OFF");
      break;

    case 'a':  // Apply settings
      applySettings();
      break;

    case '.':  // Toggle start/stop
      if (measurement_running) {
        afe.stop();
        measurement_running = false;
        Serial.println("Measurement stopped");
      } else {
        if (current_mode == MODE_IDLE) {
          Serial.println("Cannot start - set a mode first (m1-m8)");
        } else {
          afe.start();
          measurement_running = true;
          sample_count = 0;
          last_sample_count = 0;
          last_time = millis();
          Serial.println("Measurement started");
        }
      }
      break;

    case '>':  // Start
      if (current_mode == MODE_IDLE) {
        Serial.println("Cannot start - set a mode first (m1-m8)");
      } else if (!measurement_running) {
        afe.start();
        measurement_running = true;
        sample_count = 0;
        last_sample_count = 0;
        last_time = millis();
        Serial.println("Measurement started");
      } else {
        Serial.println("Already running");
      }
      break;

    case '<':  // Stop
      if (measurement_running) {
        afe.stop();
        measurement_running = false;
        Serial.println("Measurement stopped");
      } else {
        Serial.println("Not running");
      }
      break;

    default:
      Serial.println("Unknown command");
      break;
  }
}

void handleParameterCommand(const char* command) {
  char cmd1 = command[0];
  char cmd2 = (strlen(command) > 1) ? command[1] : '\0';
  long value = (strlen(command) > 2) ? strtol(&command[2], NULL, 10) : 0;

  // Log level commands
  if (cmd1 == 'l') {
    currentLogLevel = value;
    Serial.print("Log level set to: ");
    Serial.println(value);
    return;
  }

  // Mode commands
  if (cmd1 == 'm') {
    if (measurement_running) {
      afe.stop();
      measurement_running = false;
      Serial.println("Stopped previous measurement");
    }

    switch (cmd2) {
      case '1': current_mode = MODE_ECG; break;
      case '2': current_mode = MODE_BIOZ; break;
      case '3': current_mode = MODE_ECG_BIOZ; break;
      case '4': current_mode = MODE_ECG_CAL; break;
      case '5': current_mode = MODE_BIOZ_CAL; break;
      case '6': current_mode = MODE_BIOZ_INTERNAL_CAL; break;
      case '7': current_mode = MODE_BIOZ_EXTERNAL_CAL; break;
      case '8': current_mode = MODE_BIOZ_SCAN; break;
      default:
        Serial.println("Invalid mode number (1-8)");
        return;
    }
    Serial.print("Mode changed to: ");
    Serial.println(getModeName(current_mode));
    applySettings();
    return;
  }

  // ECG commands
  if (cmd1 == 'E') {
    switch (cmd2) {
      case 's':  // ECG speed
        if (value >= 0 && value <= 2) {
          ecg_speed = value;
          Serial.print("ECG speed requested: ~");
          Serial.print(requestedEcgSamplingRateSps(ecg_speed), 0);
          Serial.print(" sps (selector ");
          Serial.print(ecg_speed);
          Serial.println(")");
        } else {
          Serial.println("ECG speed must be 0-2");
        }
        break;

      case 'g':  // ECG gain
        if (value >= 0 && value <= 3) {
          ecg_gain = value;
          Serial.print("ECG gain requested: ");
          Serial.print(20 << ecg_gain);
          Serial.print(" V/V (selector ");
          Serial.print(ecg_gain);
          Serial.println(")");
        } else {
          Serial.println("ECG gain must be 0-3");
        }
        break;

      case 'l':  // ECG leads
        if (value == 2 || value == 3) {
          ecg_threeleads = (value == 3);
          Serial.print("ECG leads requested: ");
          Serial.println(ecg_threeleads ? "3-lead" : "2-lead");
        } else {
          Serial.println("ECG leads must be 2 or 3");
        }
        break;

      case 'r':  // ECG R-to-R
        if (value == 0 || value == 1) {
          ecg_rtor = (value == 1);
          Serial.print("ECG R-to-R requested: ");
          Serial.println(onOffLabel(ecg_rtor));
        } else {
          Serial.println("ECG R-to-R must be 0 or 1");
        }
        break;

      default:
        Serial.println("Unknown ECG command");
        break;
    }
    return;
  }

  // BIOZ commands
  if (cmd1 == 'B') {
    switch (cmd2) {
      case 's':  // BIOZ speed
        if (value >= 0 && value <= 1) {
          bioz_speed = value;
          Serial.print("BIOZ speed requested: ~");
          Serial.print(requestedBiozSamplingRateSps(bioz_speed), 0);
          Serial.print(" sps (selector ");
          Serial.print(bioz_speed);
          Serial.println(")");
        } else {
          Serial.println("BIOZ speed must be 0-1");
        }
        break;

      case 'g':  // BIOZ gain
        if (value >= 0 && value <= 3) {
          bioz_gain = value;
          Serial.print("BIOZ gain requested: ");
          Serial.print(10 * (1 << bioz_gain));
          Serial.print(" V/V (selector ");
          Serial.print(bioz_gain);
          Serial.println(")");
        } else {
          Serial.println("BIOZ gain must be 0-3");
        }
        break;

      case 'a':  // BIOZ analog HPF
        if (value >= 0 && value <= 7) {
          bioz_ahpf = value;
          Serial.print("BIOZ analog HPF requested: ");
          printHzOrBypass(requestedBiozAhpfHz(bioz_ahpf), 0U);
          Serial.print(" (selector ");
          Serial.print(bioz_ahpf);
          Serial.println(")");
        } else {
          Serial.println("BIOZ analog HPF must be 0-7");
        }
        break;

      case 'd':  // BIOZ digital LPF
        if (value >= 0 && value <= 3) {
          bioz_dlpf = value;
          Serial.print("BIOZ digital LPF requested: ");
          printHzOrBypass(requestedBiozDlpfHz(bioz_dlpf), 0U);
          Serial.print(" (selector ");
          Serial.print(bioz_dlpf);
          Serial.println(")");
        } else {
          Serial.println("BIOZ digital LPF must be 0-3");
        }
        break;

      case 'h':  // BIOZ digital HPF
        if (value >= 0 && value <= 3) {
          bioz_dhpf = value;
          Serial.print("BIOZ digital HPF requested: ");
          printHzOrBypass(requestedBiozDhpfHz(bioz_dhpf), 2U);
          Serial.print(" (selector ");
          Serial.print(bioz_dhpf);
          Serial.println(")");
        } else {
          Serial.println("BIOZ digital HPF must be 0-3");
        }
        break;

      case 'f':  // BIOZ frequency
        if (value >= 125 && value <= 128000) {
          bioz_frequency = value;
          Serial.print("BIOZ frequency requested: ");
          Serial.print(value);
          Serial.println(" Hz");
        } else {
          Serial.println("BIOZ frequency must be 125-128000 Hz");
        }
        break;

      case 'c':  // BIOZ current
        if (value >= 0 && value <= 96000) {
          bioz_current = value;
          Serial.print("BIOZ current requested: ");
          Serial.print(value);
          Serial.println(" nA");
        } else {
          Serial.println("BIOZ current must be 0-96000 nA");
        }
        break;

      case 'p':  // BIOZ phase
        bioz_phase = value;
        Serial.print("BIOZ phase requested: ");
        Serial.print(value);
        Serial.println(" deg");
        break;

      case 'l':  // BIOZ lead bias
        if (value == 0 || value == 1) {
          bioz_leadbias = (value == 1);
          Serial.print("BIOZ lead bias requested: ");
          Serial.println(onOffLabel(bioz_leadbias));
        } else {
          Serial.println("BIOZ lead bias must be 0 or 1");
        }
        break;

      case 'o':  // BIOZ lead-off detect
        if (value == 0 || value == 1) {
          bioz_leadsoffdetect = (value == 1);
          Serial.print("BIOZ lead-off detect requested: ");
          Serial.println(onOffLabel(bioz_leadsoffdetect));
        } else {
          Serial.println("BIOZ lead-off must be 0 or 1");
        }
        break;

      case 'w':  // BIOZ wires
        if (value == 2 || value == 4) {
          bioz_fourleads = (value == 4);
          Serial.print("BIOZ wiring requested: ");
          Serial.println(bioz_fourleads ? "4-wire" : "2-wire");
        } else {
          Serial.println("BIOZ wires must be 2 or 4");
        }
        break;

      default:
        Serial.println("Unknown BIOZ command");
        break;
    }
    return;
  }

  // Scan commands
  if (cmd1 == 'S') {
    switch (cmd2) {
      case 'a':  // Scan averages
        if (value >= 1 && value <= 8) {
          scan_avg = value;
          Serial.print("Scan averages requested: ");
          Serial.println(value);
        } else {
          Serial.println("Scan averages must be 1-8");
        }
        break;

      case 'f':  // Scan fast mode
        if (value == 0 || value == 1) {
          scan_fast = (value == 1);
          Serial.print("Scan speed requested: ");
          Serial.println(scan_fast ? "~64 sps BIOZ" : "~32 sps BIOZ");
        } else {
          Serial.println("Scan fast mode must be 0 or 1");
        }
        break;

      case 'r':  // Scan full range
        if (value == 0 || value == 1) {
          scan_fullrange = (value == 1);
          Serial.print("Scan range requested: ");
          Serial.println(scan_fullrange ? "128 kHz down to 125 Hz" : "128 kHz down to 1 kHz");
        } else {
          Serial.println("Scan full range must be 0 or 1");
        }
        break;

      case 'i':  // Scan internal resistor
        if (value == 0 || value == 1) {
          scan_internal = (value == 1);
          Serial.print("Scan source requested: ");
          Serial.println(scan_internal ? "INTERNAL RESISTOR" : "EXTERNAL ELECTRODES");
        } else {
          Serial.println("Scan source must be 0 or 1");
        }
        break;

      case 'p':  // Scan phase range
        if (value == 0 || value == 1) {
          scan_phase_range = (value == 1) ? BIOZ_SCAN_PHASE_REDUCED : BIOZ_SCAN_PHASE_FULL;
          Serial.print("Scan phase range requested: ");
          Serial.println(scanPhaseRangeLabel(scan_phase_range));
        } else {
          Serial.println("Scan phase range must be 0 (full) or 1 (reduced)");
        }
        break;

      case 'h':  // Scan internal BIST AHPF override
        if ((value >= 0 && value <= 7) || value == 255) {
          scan_internal_bist_ahpf = static_cast<uint8_t>(value);
          Serial.print("Scan internal AHPF requested: ");
          Serial.println(scanAhpfOverrideLabel(scan_internal_bist_ahpf));
        } else {
          Serial.println("Scan internal AHPF must be 255 (dynamic) or 0-7");
        }
        break;

      case 't':  // Scan settle samples
        if (value >= 1 && value <= 64) {
          scan_settle_samples = static_cast<uint8_t>(value);
          if (scan_current_change_settle_samples < scan_settle_samples) {
            scan_current_change_settle_samples = scan_settle_samples;
          }
          Serial.print("Scan settle requested: ");
          Serial.print(scan_settle_samples);
          Serial.println(" samples");
        } else {
          Serial.println("Scan settle must be 1-64 samples");
        }
        break;

      case 'c':  // Scan current-change settle samples
        if (value >= 1 && value <= 64) {
          scan_current_change_settle_samples = static_cast<uint8_t>(value);
          if (scan_current_change_settle_samples < scan_settle_samples) {
            scan_current_change_settle_samples = scan_settle_samples;
          }
          Serial.print("Scan current-change settle requested: ");
          Serial.print(scan_current_change_settle_samples);
          Serial.println(" samples");
        } else {
          Serial.println("Scan current-change settle must be 1-64 samples");
        }
        break;

      default:
        Serial.println("Unknown Scan command");
        break;
    }
    return;
  }

  // Calibration commands
  if (cmd1 == 'C') {
    switch (cmd2) {
      case 'r':  // Calibration resistor
        if (value > 0 && value <= 100000) {
          cal_resistance = value;
          Serial.print("Calibration resistor requested: ");
          Serial.print(value);
          Serial.println(" Ohm");
        } else {
          Serial.println("Calibration resistor must be 1-100000 Ohm");
        }
        break;

      case 'm':  // Calibration modulation
        if (value >= 0 && value <= 3) {
          cal_modulation = value;
          Serial.print("Calibration modulation requested: ");
          if (cal_modulation == 0U) {
            Serial.println("DC only");
          } else {
            Serial.print(requestedCalibrationModulationOhm(cal_resistance, cal_modulation), 1);
            Serial.print(" Ohm step (selector ");
            Serial.print(cal_modulation);
            Serial.println(")");
          }
        } else {
          Serial.println("Calibration modulation must be 0-3");
        }
        break;

      case 'f':  // Calibration modulation frequency
        if (value >= 0 && value <= 4) {
          cal_mod_freq = value;
          Serial.print("Calibration modulation frequency requested: ");
          Serial.print(requestedCalibrationModulationFrequencyHz(cal_mod_freq), 4);
          Serial.print(" Hz (selector ");
          Serial.print(cal_mod_freq);
          Serial.println(")");
        } else {
          Serial.println("Calibration mod frequency must be 0-4");
        }
        break;

      default:
        Serial.println("Unknown Calibration command");
        break;
    }
    return;
  }

  Serial.println("Unknown command");
}

void processCommand() {
  if (serial_input_buff[0] == '\0') return;  // Empty command

  // Single-character commands
  if (strlen(serial_input_buff) == 1) {
    handleGeneralCommand(serial_input_buff[0]);
  }
  // Parameter commands
  else {
    handleParameterCommand(serial_input_buff);
  }
}

/******************************************************************************************************/
// Data Display
/******************************************************************************************************/

void printSpectrum(const ImpedanceSpectrum& spectrum) {
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

void displayData() {
  float value = 0.0f;
  bool had_data = false;

  // ECG Data
  while (ECG_data.available() > 0) {
    ECG_data.pop(value);
    if (data_reporting) {
      Serial.print("ECG [mV]: ");
      Serial.println(value, 3);
    }
    sample_count++;
    had_data = true;
  }

  // BIOZ Data
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
    if (data_reporting) {
      if (current_mode == MODE_BIOZ_CAL) {
        Serial.print("BIOZ_Cal_[raw]: ");
        Serial.println(value, 0);
      } else {
        Serial.print("BIOZ [Ohm]: ");
        Serial.println(value, 2);
      }
    }
    sample_count++;
    had_data = true;
  }

  // RTOR Data (R-to-R intervals)
  while (RTOR_data.available() > 0) {
    RTOR_data.pop(value);
    if (data_reporting) {
      Serial.print("RR [ms]: ");
      Serial.print(value, 1);
      if (value > 0.0f) {
        Serial.print(" | HR [bpm]: ");
        Serial.println(60000.0f / value, 1);
      } else {
        Serial.println();
      }
    }
    had_data = true;
  }

  // Periodic sampling rate display (every 5 seconds)
  if (had_data) {
    current_time = millis();
    if (current_time - last_time >= 5000) {
      uint32_t sample_delta = sample_count - last_sample_count;
      uint32_t time_delta = current_time - last_time;
      sampling_rate = (sample_delta * 1000.0f) / time_delta;
      
      if (data_reporting) {
        Serial.print("Sampling rate: ");
        Serial.print(sampling_rate, 1);
        Serial.print(" sps | Samples: ");
        Serial.println(sample_count);
      }
      
      last_sample_count = sample_count;
      last_time = current_time;
    }
  }

  while (BIOZ_spectrum.available() > 0) {
    ImpedanceSpectrum spectrum;
    if (BIOZ_spectrum.pop(spectrum) != 1U) {
      break;
    }
    Serial.println("BIOZ scan complete");
    printSpectrum(spectrum);
    measurement_running = false;
  }
}

/******************************************************************************************************/
// Setup
/******************************************************************************************************/

void setup() {
  currentLogLevel = LOG_LEVEL;

  // Serial Setup
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1000);
  serialTrigger(waitmsg, 5000);

  LOGI("MAX30001G Interactive Test Program Starting");
  LOGI("===========================================");

  // AFE Initialization
  afe.begin();
  LOGI("MAX30001G initialized");

  // Read device info
  afe.readInfo();
  afe.printInfo();

  // Health check
  LOGI("Running health check...");
  HealthCheckResult result = afe.healthCheck();
  const bool info_ok = (result.info_reg != 0U);
  const bool status_ok = !result.fault_present;
  if (result.spi_ok && info_ok && status_ok) {
    LOGI("Health check: PASS");
  } else {
    LOGE("Health check: FAIL");
  }

  Serial.println();
  Serial.println("Ready for commands. Type '?' for help.");
  Serial.println();
}

/******************************************************************************************************/
// Main Loop
/******************************************************************************************************/

void loop() {
  // Serial command handling
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_index > 0) {
        serial_input_buff[serial_index] = '\0';
        serial_command_complete = true;
        serial_index = 0;
      }
    } else if (c >= 32 && c <= 126) {  // Printable ASCII
      if (serial_index < sizeof(serial_input_buff) - 1) {
        serial_input_buff[serial_index++] = c;
      }
    }
  }

  // Process command if complete
  if (serial_command_complete) {
    processCommand();
    serial_command_complete = false;
  }

  // Update AFE and display data if running
  if (measurement_running) {
    afe.update(current_mode == MODE_BIOZ_CAL);
    displayData();
  }

  delay(1);  // Small delay to prevent overwhelming the system
}
