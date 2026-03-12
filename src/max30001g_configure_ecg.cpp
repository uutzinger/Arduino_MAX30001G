/******************************************************************************************************/
// Configure ECG
/******************************************************************************************************/
#include <Arduino.h>
#include "logger.h"      // Logging 
#include "max30001g.h"

void MAX30001G::setECGSamplingRate(uint8_t speed_select) {
    /*
    Set ECG sampling rate selector for the AFE:
      0 = low, 1 = medium, 2 = high

    Actual sample rate depends on FMSTR (Table 34):
      FMSTR 00/01: rate bits 00/01/10 = high/medium/low
      FMSTR 10/11: only rate bits 10 are supported

    Based on FMSTR, adjusts global variable ECG_samplingRate
    */
  
    cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);
    cnfg_ecg.all  = readRegister24(MAX30001_CNFG_ECG);

    if ((cnfg_gen.bit.fmstr == 0b10) || (cnfg_gen.bit.fmstr == 0b11)) {
      cnfg_ecg.bit.rate = 0b10; // only supported ECG_RATE at these FMSTR settings
      if (speed_select != 0) {
        LOGW("FMSTR=%u only supports ECG_RATE=10; forcing low selector.", cnfg_gen.bit.fmstr);
      }
    } else {
      switch (speed_select) {
        case 0:
          cnfg_ecg.bit.rate = 0b10; // low
          break;
        case 1:
          cnfg_ecg.bit.rate = 0b01; // medium
          break;
        case 2:
          cnfg_ecg.bit.rate = 0b00; // high
          break;
        default:
          cnfg_ecg.bit.rate = 0b10; // low default
          LOGE("Invalid ECG speed selector: %u. Using low.", speed_select);
          break;
      }
    }
  
    writeRegister(MAX30001_CNFG_ECG, cnfg_ecg.all);
    delay(100);
    updateGlobalECG_samplingRate();
  
  }
  
  void MAX30001G::setECGfilter(uint8_t lpf, uint8_t hpf) {
  /*
    Set the ECG digital lpf for the AFE  
      0 bypass, no filter applied
      1 low, approx 40Hz
      2 medium, approx 100Hz
      3 high, approx 150Hz
  
    Set the ECG digital hpf for the AFE  
      0 bypass (DC)
      1 0.5Hz (engage)

    Hardware high pass filter
    In addition there is a fixed HPF with an external capacitor on C_HPF
    0.1, 1.0, 10uF for 5, 0.5, 0.05Hz corner frequency
    5Hz results in lowest signal quality but best motion artifact suppression

    MediBrick uses 10uF

  */
  
    cnfg_ecg.all = readRegister24(MAX30001_CNFG_ECG);
  
    // Digital High Pass
    if (hpf == 0) {
      ECG_hpf = 0.0f;
      cnfg_ecg.bit.dhpf = 0; // bypass
    } else {
      ECG_hpf = 0.5f;
      cnfg_ecg.bit.dhpf = 1; // 0.5Hz
      if (hpf > 1) {
        LOGE("Invalid ECG HPF selector: %u. Using 0.5Hz.", hpf);
      }
    }
  
    // Digital Low Pass
    switch (lpf) {
      case 0:
        cnfg_ecg.bit.dlpf = 0b00; // bypass
        break;
      case 1:
        cnfg_ecg.bit.dlpf = 0b01; // low
        break;
      case 2:
        cnfg_ecg.bit.dlpf = 0b10; // medium
        break;
      case 3:
        cnfg_ecg.bit.dlpf = 0b11; // high
        break;
      default:
        cnfg_ecg.bit.dlpf = 0b01; // low
        LOGE("Invalid ECG LPF selector: %u. Using 40Hz class.", lpf);
        break;
    }
    writeRegister(MAX30001_CNFG_ECG, cnfg_ecg.all);
    delay(100);
  
    updateGlobalECG_lpf();
  
  }
  
  void MAX30001G::setECGgain(uint8_t gain) {
  /*
    Set the ECG gain for the AFE
      0 20V/V
      1 40V/V
      2 80V/V **
      3 160V/V
  
      max DC differential is +/- 650mV
      max AC differential is +/-  32mV
      usually ECG is smaller than  3mV
  
      V_ECG = ADC * V_REF / (2^17 * ECG_GAIN)
      V_REF is typ 1000mV
  
  */
  
    if (gain > 3) {
      LOGE("Invalid ECG gain selector: %u. Valid range is 0..3.", gain);
      return;
    }

    cnfg_ecg.all = readRegister24(MAX30001_CNFG_ECG);
    cnfg_ecg.bit.gain = gain;
    writeRegister(MAX30001_CNFG_ECG, cnfg_ecg.all);

    switch (gain){
      case 0:
        ECG_gain = 20;
        break;
      case 1:
        ECG_gain = 40;
        break;
      case 2:
        ECG_gain = 80;
        break;
      case 3:
        ECG_gain = 160;
        break;
      default:
        ECG_gain = 20;
        break;
    }
  }
  
  void MAX30001G::setECGLeadPolarity(bool inverted, bool open) {
    /*
    inverted 0: regular
    inverted 1: swap ECG_N and ECG_P connection to AFE
  
    open 0: P,N are connected to AFE
    open 1: P,N are isolated, use for internal calibration
    */
  
    cnfg_emux.all  = readRegister24(MAX30001_CNFG_EMUX);
    
    cnfg_emux.bit.pol = inverted ? 1U : 0U;
    cnfg_emux.bit.openp = open ? 1U : 0U;
    cnfg_emux.bit.openn = open ? 1U : 0U;
  
    writeRegister(MAX30001_CNFG_EMUX, cnfg_emux.all);
   
  }

  /****************************************************************

  Input INA has ability to automatically, manually or fast recover after excessive overdrive
  (after defib, pacing or electro surgery)
      
  ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass):
  00 = Normal Mode (Fast Recovery Mode Disabled)
  01 = Manual Fast Recovery Mode Enable (remains active until disabled)
  10 = Automatic Fast Recovery Mode Enable (Fast Recovery automatically activated when/while ECG outputs are saturated, using FAST_TH).
  11 = Reserved. Do not use.

  Automatic Fast Recovery Threshold:
  ----------------------------------
  If fast is enabled (10) and the output of an ECG measurement exceeds the symmetric thresholds defined by 2048*FAST_TH for more than 125ms, 
  the Fast Recovery mode will be automatically engaged and remain active for 500ms.

  Threshold Voltage * ECG_gain == FAST_TH x 2048 × V_ref / ADC_FULLSCALE  
  Threshold voltage is in mV.

  Example:
    ADC_FULLSCALE = 2^17 = 131072
    ​V_ref = 1000mV
    ECG_GAIN = 20V/V
    FAST_TH = 0x3F (default, 63 decimal) results in trigger when ADC value >0x1F800 or <0x20800 in 2's complement

    Threshold Voltage: 63*1000[mV]*2048/131072/20[V/V] = +/-49.22mV 

  ****************************************************************/

  void MAX30001G::setECGAutoRecovery(int threshold_voltage) {
    /*
      threshold_voltage is ECG input threshold in mV.
      FAST_TH mapping from datasheet Table 20:
      2048*FAST_TH ADC counts correspond to:
      threshold_voltage * ECG_gain / V_ref = (2048*FAST_TH) / 2^17
    */

    if (ECG_gain <= 0 || V_ref <= 0) {
      LOGE("ECG auto recovery requires ECG_gain and V_ref to be initialized.");
      return;
    }

    if (threshold_voltage < 0) {
      threshold_voltage = 0;
    }

    // Ensure boundary at full-scale input.
    if ((threshold_voltage * ECG_gain) > V_ref) {
      threshold_voltage = V_ref / ECG_gain;
    }
  
    // Calculate FAST_TH value.
    uint8_t fast_th = static_cast<uint8_t>((threshold_voltage * ECG_gain * 131072L) / (2048L * V_ref));
  
    // FAST_TH is 6-bit.
    if (fast_th > 0x3F) {
      fast_th = 0x3F;
    }
  
    mngr_dyn.all  = readRegister24(MAX30001_MNGR_DYN);
    mngr_dyn.bit.fast = 0b10;       // auto
    mngr_dyn.bit.fast_th = fast_th; // threshold
    writeRegister(MAX30001_MNGR_DYN, mngr_dyn.all);
  
  }
  
  void MAX30001G::setECGNormalRecovery() {
    // ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass):
    // 00 = Normal Mode (Fast Recovery Mode Disabled)
    mngr_dyn.all  = readRegister24(MAX30001_MNGR_DYN);
    mngr_dyn.bit.fast = 0b00; // normal
    writeRegister(MAX30001_MNGR_DYN, mngr_dyn.all);
  
  }

  /*
  Input INA has ability to be forcefully set to recover after excessive overdrive
  1. Set Manual Recovery
  2. Stop Manual Recovery
  If we  manually enter recovery we need to manually leave it, otherwise AFE will not be able to measure.
  */

  void MAX30001G::startECGManualRecovery() {
    /* 
    Start the manual recovery.
    */
    mngr_dyn.all  = readRegister24(MAX30001_MNGR_DYN);
    mngr_dyn.bit.fast = 0b01; // start manual revcovery
    writeRegister(MAX30001_MNGR_DYN, mngr_dyn.all);
  }
  
  void MAX30001G::stopECGManualRecovery() {
    /* 
    Stop the manual recovery.
    */  
    mngr_dyn.all  = readRegister24(MAX30001_MNGR_DYN);
    mngr_dyn.bit.fast = 0b00;  // Resets to normal mode
    writeRegister(MAX30001_MNGR_DYN, mngr_dyn.all);
  }
  
