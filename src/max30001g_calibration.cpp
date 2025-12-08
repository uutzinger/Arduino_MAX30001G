/******************************************************************************************************/
// Calibration
/******************************************************************************************************/

// Embedded system specific
#include "logger.h"
#include "max30001g_globals.h"
#include "max30001g_comm.h"        // SPI communication
#include "max30001g_defs.h"        // Register definitions
#include "max30001g_calibration.h"

void MAX30001G::setDefaultNoCalibration(void) {
  /* 
   * Do not enable calibration signal
   */
  
  setCalibration(
    false,      // Enable ECG calibration
    false,      // Disable BIOZ calibration
    true,       // Unipolar mode
    true,       // ±0.5 mV calibration voltage
    0b100,      // Frequency: 1 Hz
    50          // 50% duty cycle
  );
}
    
void MAX30001G::setDefaultECGCalibration(void) {
  /*
    Sets default values for ECG calibration.
    - enable: true (calibration enabled)
    - unipolar: true (Unipolar mode)
    - cal_vmag: true (±0.5 mV)
    - freq: 0b100 (1 Hz) (default)
    - dutycycle: 50% (default)
  */

  setCalibration(
    true,       // Enable ECG calibration
    false,      // Disable BIOZ calibration
    true,       // Unipolar mode
    true,       // ±0.5 mV calibration voltage
    0b100,      // Frequency: 1 Hz (FMSTR/2^15)
    50          // 50% duty cycle
  );
}

void MAX30001G::setDefaultBIOZCalibration(void) {
  /*
    Sets default values for BIOZ calibration.
    - enable: true (calibration enabled)
    - unipolar: true (Unipolar mode)
    - cal_vmag: true (±0.5 mV)
    - freq: 0b100 (1 Hz) (default)
    - dutycycle: 50% (default)
  */

  setCalibration(
    false,      // Disable ECG calibration
    true,       // Enable BIOZ calibration
    true,       // Unipolar mode
    true,       // ±0.5 mV calibration vothighltage
    0b100,      // Frequency: 1 Hz (FMSTR/2^15)
    50          // 50% duty cycle
  );
}
    
void MAX30001G::setCalibration(bool enableECGCal, bool enableBIOZCal, bool unipolar, bool cal_vmag, uint8_t freq, uint8_t dutycycle) {
/*
  Configures ECG and BIOZ voltage calibration settings.

  This function disconnects ECGP and ECGN from the subject and applies the calibration settings.
  
  Parameters:
    - enableECGCal: Enable or disable ECG calibration
    - enableBIOZCal: Enable or disable BIOZ calibration
    Signal Settings:
    - unipolar: Set to true for unipolar, false for bipolar test signals
    - cal_vmag: Set the calibration voltage magnitude: true for ±0.5 mV, false for ±0.25 mV (CAL_VMAG).
    - freq: Calibration frequency (value = 0..7) will result in FMSTR / (2^(7+value*2)) Hz or approx 250..0.01 Hz
    - dutycycle: Duty cycle percentage (default 50%)

  ECG Calibration
  - ECGN connect internally to calibration signal
  - ECGP connect internally to calibration signal
  - Enable/Disable
  - Unipolar/Bipolar CAL_VMODE
  - Voltage +/-0.25 pr 0.5 mV CAL_VMAG
  - Freqeuncy 0.015625 to 256 Hz
  - Dutycyle 1-99%, 50% is default 

  BIOZ Calibration
  - BIOZN
  - BIOZP

  Registers involved:

  cnfg_emux.bit.openp
    Open ECGP input switch
    0 = Connect ECGP to AFE
    1 = Disconnect ECGP from AFE, e.g. for calibration

  cnfg_emux.bit.openn
    Open ECGN input switch
    0 = Connect ECGN to AFE
    1 = Disconnect ECGN from AFE, e.g. for calibration

  cnfg_emux.bit.ecg_calp_sel
    ECGP Calibration Selection
      00 = No calibration signal applied
      01 = Input is connected to VMID
      10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
      11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)

  cnfg_emux.bit.ecg_caln_sel
    ECGN Calibration Selection
      00 = No calibration signal applied
      01 = Input is connected to VMID
      10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
      11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)

  // Enable or disable calibration
  cnfg_cal.bit.vcal = enable ? 1 : 0;

  cnfg_cal.bit.vmode = unipolar ? 0 : 1;   // Set calibration mode: 0 = Unipolar, 1 = Bipolar
  cnfg_cal.bit.vmag = cal_vmag ? 1 : 0;    // Set voltage magnitude: 0 = ±0.25 mV, 1 = ±0.5 mV

  cnfg_cal.bit.fcal = freq & 0x07;         // Set calibration frequency (0.015625 to 256 Hz)
    Calibration Source Frequency Selection (FCAL) 
    000 = FMSTR /128   (     256, 250,        or 249.75Hz)
    001 = FMSTR /512   (      64,  62.5,      or  62.4375   Hz)
    010 = FMSTR /2048  (      16,  15.625,    or  15.609375 Hz)
    011 = FMSTR /8192  (       4,   3.90625,  or   3.902344 Hz)
    100 = FMSTR /2^15  (       1,   0.976563, or   0.975586 Hz)
    101 = FMSTR /2^17  (    0.25,   0.24414,  or   0.243896 Hz)
    110 = FMSTR /2^19  (  0.0625,   0.061035, or   0.060974 Hz)
    111 = FMSTR /2^21  (0.015625,   0.015259, or   0.015244 Hz)
    Actual frequencies are determined by FMSTR selection (see CNFG_GEN for details),
    frequencies in parenthesis are based on 32,768, 32,000, or 31,968Hz clocks (FMSTR[1:0] = 00). TCAL = 1/FCAL

  cnfg_cal.bit.fifty = dutycycle==50 ? 1 : 0;  // Set duty cycle: 1 = 50%, 0 = Custom time high duration

  cnfg_cal.bit.thigh (11 bit)
    Calibration Source Time High Selection
    If FIFTY = 1, tHIGH = 50% (and THIGH[10:0] are ignored), 
    otherwise THIGH = THIGH[10:0] x CAL_resolution
    CAL_RES is determined by FMSTR selection (see CNFG_GEN for details);
    for example, if FMSTR[1:0] = 00, CAL_resolution = 30.52µs

  */

  // Read current settings
  cnfg_emux.all = readRegister24(MAX30001_CNFG_EMUX);
  cnfg_bmux.all = readRegister24(MAX30001_CNFG_BMUX);
  cnfg_cal.all  = readRegister24(MAX30001_CNFG_CAL);

  if (enableECGCal) {
    // Disconnect ECGP and ECGN by setting the openp and openn bits
    cnfg_emux.bit.openp = 1; // Open ECGP input switch
    cnfg_emux.bit.openn = 1; // Open ECGN input switch
    // Connect internally
    cnfg_emux.bit.ecg_calp_sel = 0b01; // CALP
    cnfg_emux.bit.ecg_caln_sel = 0b11; // CALN 
  } else {
    // Disconnect calibration signal internally
    cnfg_emux.bit.ecg_calp_sel = 0b00; // CALP
    cnfg_emux.bit.ecg_caln_sel = 0b00; // CALN       
    // Connect ECGP and ECGN by setting the openp and openn bits
    cnfg_emux.bit.openp = 0; // Connect ECGP input
    cnfg_emux.bit.openn = 0; // Connect ECGN input
  }

  if (enableBIOZCal) {
    // Disconnect BIOZP and BIOZN by setting the openp and openn bits
    cnfg_bmux.bit.openp = 1; // Open BIOZP input switch
    cnfg_bmux.bit.openn = 1; // Open BIOZN input switch
    // Connect internally
    cnfg_bmux.bit.bioz_calp_sel = 0b01; // CALP
    cnfg_bmux.bit.bioz_caln_sel = 0b11; // CALN 
  } else {
    // Disable BIOZ Calibration, close the switches
    cnfg_bmux.bit.bioz_calp_sel = 0b00; // No calibration signal applied
    cnfg_bmux.bit.bioz_caln_sel = 0b00;
    cnfg_bmux.bit.openp = 0; // Close BIOZP switch
    cnfg_bmux.bit.openn = 0; // Close BIOZN switch      
  }

  if ( ECGenable || BIOZenable ) {
    // Apply calibration settings
    cnfg_cal.bit.vmode = unipolar ? 0 : 1;   // Set calibration mode: 0 = Unipolar, 1 = Bipolar
    cnfg_cal.bit.vmag = cal_vmag ? 1 : 0;    // Set voltage magnitude: 0 = ±0.25 mV, 1 = ±0.5 mV
    cnfg_cal.bit.fcal = freq & 0x07;         // Set calibration frequency (0.015625 to 256 Hz)

    // update gloabl variable CAL_fcal (calibration frequency in Hz)
    updateGlobalCAL_fcal();

    if (dutycyle == 50) {
      cnfg_cal.bit.fifty = 1; // 50%
    } else {
      cnfg_cal.bit.fifty = 0; 
      // Calulcate thigh value
      //  - dutycyle is in percent e.g. 50 specified by user
      //  - CAL_resultion is in micro seconds, system setting
      //  - CAL_fcal is in Hz, system setting
      //  - Period = 1/CAL_fcal is in seconds. system setting
      //  - time high = duty cycle * 1/CAL_fcal in seconds, computed
      // 
      // thigh register is in resolution units: 
      // thigh = time high in seconds * 1,000,000 / CAL_resolution 
      //       = dutycycle / CAL_fcal * 1,000,000 / CAL_resoltuion
      //       = (((dutycyle / 100.0) * 1,000,000 / CAL_resolution) / CAL_fcal );
      // which is:

      cnfg_cal.bit.thigh = static_cast<uint16_t>((float(dutycyle) * 10000.0) / CAL_fcal / CAL_resolution);
    }

    cnfg_cal.bit.vcal = 1;

  } else {
    
    LOGD("Both ECG and BIOZ calibration are disabled. No calibration will be applied.");
    cnfg_cal.bit.vcal = 0;
  } 

  // Write back the configuration
  writeRegister(MAX30001_CNFG_EMUX, cnfg_emux.all);
  writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
  writeRegister(MAX30001_CNFG_CAL, cnfg_cal.all);

}

#endif
