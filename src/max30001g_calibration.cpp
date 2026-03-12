/******************************************************************************************************/
// Calibration
/******************************************************************************************************/

#include <math.h>

#include "logger.h"
#include "max30001g.h"

namespace {

uint8_t clampDutyCyclePercent(uint8_t dutycycle) {
  if (dutycycle < 1U) {
    return 1U;
  }
  if (dutycycle > 99U) {
    return 99U;
  }
  return dutycycle;
}

} // namespace

void MAX30001G::setDefaultNoTestSignal(void) {
  /*
    Disable ECG/BIOZ voltage calibration source (VCAL).
  */
  setTestSignal(
    false, // set enableECGCal to false
    false, // set enableBIOZCal to false
    true,  // set unipolar (ignored when disabled)
    true,  // 0.5mV (ignored when disabled)
    0b100, // ~1Hz (ignored when disabled)
    50     // 50% (ignored when disabled)
  );
}

void MAX30001G::setDefaultECGTestSignal(void) {
  /*
    ECG VCAL default:
    - enabled
    - unipolar
    - 0.5mV
    - FCAL=100 (~1Hz)
    - 50% duty
  */
  setTestSignal(
    true,  // enable ECG calibration 
    false, // disable BIOZ calibration
    true,  // enable unipolar
    true,  // 0.5mV
    0b100, // ~1Hz
    50);   // 50% duty cycle
}

void MAX30001G::setDefaultBIOZTestSignal(void) {
  /*
    BIOZ VCAL default:
    - enabled
    - unipolar
    - 0.5mV
    - FCAL=100 (~1Hz)
    - 50% duty
  */
  setTestSignal(
    false, // disable ECG calibration
    true,  // enable BIOZ calibration
    true,  // enable unipolar
    true,  // 0.5mV
    0b100, // ~1Hz
    50     // 50% duty cycle
  );
}

void MAX30001G::setDefaultECGandBIOZTestSignal(void) {
  /*
    BIOZ VCAL default:
    - enabled
    - unipolar
    - 0.5mV
    - FCAL=100 (~1Hz)
    - 50% duty
  */
  setTestSignal(
    true,  // enable ECG calibration
    true,  // enable BIOZ calibration
    true,  // enable unipolar
    true,  // 0.5mV
    0b100, // ~1Hz
    50     // 50% duty cycle
  );
}

//////////////////////////////////////////////////////////////////

void MAX30001G::setTestSignal(
  bool enableECGCalSignal,
  bool enableBIOZCalSignal,
  bool unipolar,
  bool cal_vmag,
  uint8_t freq,
  uint8_t dutycycle
) {
  /*
    Configure internal voltage calibration source (CNFG_CAL) and route it to ECG/BIOZ muxes:
      ECG mux:  CNFG_EMUX CALP/CALN selectors
      BIOZ mux: CNFG_BMUX CALP/CALN selectors

    VCAL routing selections:
      00 = none
      10 = VCALP
      11 = VCALN

    Note:
    - If VCAL is enabled, BIOZ impedance test modes (EN_BIST/EN_BISTR) are disabled.
    - Custom duty cycle uses CAL_THIGH and CAL_RES.

  This function disconnects ECGP and ECGN from the subject and applies the calibration settings.
  
  Parameters:
    - enableECGCalSignal:  Enable or disable ECG calibration signal
    - enableBIOZCalSignal: Enable or disable BIOZ calibration signal
     (both signals can be enabled at the same time, they share the same frequency/duty settings)
    Signal Settings:
    - unipolar:      Set to true for unipolar, false for bipolar test signals
    - cal_vmag:      Set the calibration voltage magnitude: true for ±0.5 mV, false for ±0.25 mV (CAL_VMAG).
    - freq:          Calibration frequency (value = 0..7) will result in FMSTR / (2^(7+value*2)) Hz or approx 250..0.01 Hz
    - dutycycle:     Duty cycle percentage (default 50%)

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

  Enable or disable calibration:

  cnfg_cal.bit.vcal = enable ? 1 : 0;

  Unipolar or bipolar mode:

  cnfg_cal.bit.vmode = unipolar ? 0 : 1;   // Set calibration mode: 0 = Unipolar, 1 = Bipolar

  Mangitude of the calibration voltage:

  cnfg_cal.bit.vmag = cal_vmag ? 1 : 0;    // Set voltage magnitude: 0 = ±0.25 mV, 1 = ±0.5 mV

  Calibration frequency:

  cnfg_cal.bit.fcal = freq & 0x07;         // Set calibration frequency (0.015625 to 256 Hz)
    Calibration Source Frequency Selection (FCAL) 
    000 = FMSTR /128   (     256, 250,        or 249.75Hz)
    001 = FMSTR /512   (      64,  62.5,      or  62.4375   Hz)
    010 = FMSTR /2048  (      16,  setupECG15.625,    or  15.609375 Hz)
    011 = FMSTR /8192  (       4,   3.90625,  or   3.902344 Hz)
    100 = FMSTR /2^15  (       1,   0.976563, or   0.975586 Hz)
    101 = FMSTR /2^17  (    0.25,   0.24414,  or   0.243896 Hz)
    110 = FMSTR /2^19  (  0.0625,   0.061035, or   0.060974 Hz)
    111 = FMSTR /2^21  (0.015625,   0.015259, or   0.015244 Hz)
    Actual frequencies are determined by FMSTR selection (see CNFG_GEN for details),
    frequencies in parenthesis are based on 32,768, 32,000, or 31,968Hz clocks (FMSTR[1:0] = 00). TCAL = 1/FCAL

  Calibration signal duty cycle:

    cnfg_cal.bit.fifty = dutycycle==50 ? 1 : 0;  // Set duty cycle: 1 = 50%, 0 = Custom time high duration

    cnfg_cal.bit.thigh (11 bit)
      Calibration Source Time High Selection
      If FIFTY = 1, tHIGH = 50% (and THIGH[10:0] are ignored), 
      otherwise THIGH = THIGH[10:0] x CAL_resolution
      CAL_RES is determined by FMSTR selection (see CNFG_GEN for details);
      for example, if FMSTR[1:0] = 00, CAL_resolution = 30.52µs


  */

  cnfg_emux.all    = readRegister24(MAX30001_CNFG_EMUX);
  cnfg_bmux.all    = readRegister24(MAX30001_CNFG_BMUX);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);
  cnfg_cal.all     = readRegister24(MAX30001_CNFG_CAL);

  // ECG calibration routing
  if (enableECGCalSignal) {
    cnfg_emux.bit.openp = 1U; // Disconnect ECGP from AFE to route calibration signal
    cnfg_emux.bit.openn = 1U; // Disconnect ECGN from AFE to route calibration signal
    cnfg_emux.bit.calp_sel = 0b10; // VCALP interncal connection
    cnfg_emux.bit.caln_sel = 0b11; // VCALN internal connection
  } else {
    cnfg_emux.bit.calp_sel = 0b00; // none
    cnfg_emux.bit.caln_sel = 0b00; // none
    cnfg_emux.bit.openp = 0U; // Close CAL input 
    cnfg_emux.bit.openn = 0U; // Close CAL input 
  }

  // BIOZ calibration routing
  if (enableBIOZCalSignal) {
    cnfg_bmux.bit.openp = 1U; // Open BIOZP input switch to route calibration signal
    cnfg_bmux.bit.openn = 1U; // Open BIOZN input switch to route calibration signal
    cnfg_bmux.bit.calp_sel = 0b10; // VCALP
    cnfg_bmux.bit.caln_sel = 0b11; // VCALN
  } else {
    cnfg_bmux.bit.calp_sel = 0b00; // none
    cnfg_bmux.bit.caln_sel = 0b00; // none
    cnfg_bmux.bit.openp = 0U; // Close BIOZP input switch to connect to AFE
    cnfg_bmux.bit.openn = 0U; // Close BIOZN input switch to connect to AFE
  }

  const bool enableAnyCalibration = (enableECGCalSignal || enableBIOZCalSignal);
  if (enableAnyCalibration) {
    // VCAL and impedance-test modes are mutually exclusive.
    if ((cnfg_bmux.bit.en_bist == 1U) || (cnfg_bioz_lc.bit.en_bistr == 1U)) {
      LOGW("Enabling VCAL disables BIOZ impedance-test modes.");
    }
    cnfg_bmux.bit.en_bist = 0U;
    cnfg_bioz_lc.bit.en_bistr = 0U;
    cnfg_bmux.bit.rmod = 0b100; // unmodulated state

    cnfg_cal.bit.vmode = unipolar ? 0U : 1U;
    cnfg_cal.bit.vmag = cal_vmag ? 1U : 0U;
    cnfg_cal.bit.fcal = static_cast<uint8_t>(freq & 0x07U);

    updateGlobalCAL_fcal(); // update global variable calibration frequency

    const uint8_t duty = clampDutyCyclePercent(dutycycle);
    if (duty == 50U) {
      cnfg_cal.bit.fifty = 1U;
      cnfg_cal.bit.thigh = 0U;
    } else {
      cnfg_cal.bit.fifty = 0U;

      if ((CAL_fcal <= 0.0f) || (CAL_resolution <= 0.0f)) {
        LOGW("Invalid CAL_fcal/CAL_resolution for custom duty cycle. Falling back to 50%%.");
        cnfg_cal.bit.fifty = 1U;
        cnfg_cal.bit.thigh = 0U;
      } else {
        const float tcal_us = 1000000.0f / CAL_fcal;
        const float thigh_us = (static_cast<float>(duty) / 100.0f) * tcal_us;
        uint32_t thigh_counts = static_cast<uint32_t>((thigh_us / CAL_resolution) + 0.5f);
        if (thigh_counts > 0x07FFU) {
          LOGW("Requested duty cycle maps to THIGH=%lu > 2047; clamping.", static_cast<unsigned long>(thigh_counts));
          thigh_counts = 0x07FFU;
        }
        cnfg_cal.bit.thigh = static_cast<uint16_t>(thigh_counts);
      }
    }

    cnfg_cal.bit.vcal = 1U;
  } else {
    cnfg_cal.bit.vcal = 0U;
    cnfg_cal.bit.fifty = 1U;
    cnfg_cal.bit.thigh = 0U;
  }

  writeRegister(MAX30001_CNFG_EMUX, cnfg_emux.all);
  writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
  writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);
  writeRegister(MAX30001_CNFG_CAL, cnfg_cal.all);

  updateGlobalCAL_fcal();
  updateGlobalBIOZ_test_impedance();
}

/******************************************************************************************************/
// BIOZ Impedance Test
/******************************************************************************************************/

void MAX30001G::setDefaultNoBIOZTestImpedance() {
  setBIOZTestImpedance(
    false, // disable impedance test
    false, // useHighResistance
    false, // enableModulation
    0b000, // rnom (5kOhm when enabled)
    0b000, // rmod (~2.96Ohm modulation when enabled)
    0b01   // modFreq (~1Hz)
  );
}

void MAX30001G::setDefaultBIOZTestImpedance() {
  setBIOZTestImpedance(
    true,  // enable
    false, // useHighResistance (use low-R RMOD BIST)
    true,  // enableModulation
    0b000, // RNOM: 5kOhm
    0b000, // RMOD value 0 (~2.96Ohm modulation)
    0b01   // modFreq: ~1Hz
  );
}

void MAX30001G::setBIOZTestImpedance(
  bool enable,
  bool useHighResistance,
  bool enableModulation,
  uint8_t rnomValue,
  uint8_t rmodValue,
  uint8_t modFreq
) {
  /*
    Internal BIOZ impedance-test options:

    1) High-resistance programmable load (CNFG_BIOZ_LC.EN_BISTR/BISTR):
       27k, 108k, 487k, 1029k.

    2) Low-resistance RMOD BIST (CNFG_BMUX.EN_BIST/RNOM/RMOD/FBIST):
       RNOM: 5k..625ohm, optional modulation.

    Datasheet requirements:
    - EN_BIST is available only when CNFG_CAL.EN_VCAL=0.
    - For RMOD BIST, OPENP/OPENN should be open to avoid body interference.

    - enable 
        true to enable internal impedance test
        false to disable internal impedance test
    - useHighResistance: 
        true to use high-resistance load (27 kΩ to 1029 kΩ), 
        false to use low-resistance modulated load.
    - resistance rnomValue: Nominal resistance setting (0 to 7). 
        high resistance mode
          0 00 =   27 kΩ
          1 01 =  108 kΩ
          2 10 =  487 kΩ
          3 11 = 1029 kΩ
        low resistance mode
          0 000 =    5 k
          1 001 =    2.5k
          2 010 =    1.667k
          3 011 =    1.25k
          4 100 =    1k
          5 101 =    0.833k
          6 110 =    0.714k
          7 111 =    0.625k
    - enableModulation: 
        true to enable modulated resistance
        false to disable modulated resistance
        Resistance will switch between RNOM and (RNOM - RMOD) at the selected modulation rate. 
    - rmodValue: 
        Modulated resistance setting (0 to 3)
          0      1     2     3     4     5    6    7 low resitance mode
        0 2960.7 704.4 329.1 185.1 118.5 82.3 60.5 46.3 mΩ
        1  980.6 245.2 109.0  61.3  39.2 27.2 20.0 15.3 mΩ
        2  247.5  61.9  27.5  none  none none none none mΩ
        3   none 
    - modFreq: 
        resistance modulation frequency (0 to 3).
        0 4Hz
        1 1Hz
        2 0.25Hz
        3 0.0625Hz

    cnfg_bioz_lc.bit.en_bistr
      Enable High-Resistance Programmable Load Value
        0 = Disable high-resistance programmable load
        1 = Enable high-resistance programmable load
        
    cnfg_bioz_lc.bit.bistr
      Select High-Resistance Programmable Load Value
        00 = 27 kΩ
        01 = 108 kΩ
        10 = 487 kΩ
        11 = 1029 kΩ

    cnfg_bmux.bit.en_bist
      BioZ Modulated Resistance Built-In-Self-Test (RMOD BIST) Mode Enable
        0 = RMOD BIST Disabled
        1 = RMOD BIST Enabled
      Note: Available only when CNFG_CAL -> EN_VCAL = 0 (Calibration Source (VCALP and VCALN) disabled)
      To avoid body interference, the BIP/N switches should be open in this mode.
      When enabled, the DRVP/N isolation switches are opened and the DRVP/N-to-BIP/N internal switches are engaged. 
      Also, the lead bias resistors are applied to the BioZ inputs in 200MΩ mode.

    cnfg_bmux.bit.fbist
      BioZ RMOD BIST Frequency Selection. Not applicable for the high-resistance (27kΩ to 1029kΩ) values.
      Calibration Source Frequency Selection (FCAL)
      00 = fMSTR/213 (Approximately 4 Hz)
      01 = fMSTR/215 (Approximately 1 Hz)
      10 = fMSTR/217 (Approximately 1/4 Hz) 
      11 = fMSTR/219 (Approximately 1/16 Hz)
      Actual frequencies are determined by FMSTR selection (see CNFG_GEN for details),
       approximate frequencies are based on a 32,768 Hz clock (FMSTR[1:0]=00). All selections
       use 50% duty cycle

    cnfg_bmux.bit.rmod, cnfg_bmux.bit.rnorm
      BioZ RMOD BIST Nominal Resistance Selection. For higher resistance values, see BIOZ_CNFG_LC.
      See RMOD BIST Settings Table for details.


      BMUX_RNOM[2:0]  BMUX_RMOD[2:0]  NOMINAL RESISTANCE (Ω)  MODULATED RESISTANCE (mΩ)
      000             000             5000                    2960
                      001                                     980.5
                      010                                     247.5
                      1xx                                     none
      001             000             2500                    740.4
                      001                                     245.2
                      010                                     61.9
                      1xx                                     none
      010             000             1667                    329.1
                      001                                     109.0
                      010                                     27.5
                      1xx                                     none
      011             000             1250                    185.1
                      001                                     61.3
                      010                                     none
                      1xx                                     none
      100             000             1000                    118.5
                      001                                     39.2
                      010                                     none
                      1xx                                     none
      101             000              833                    82.3
                      001                                     27.2
                      010                                     none
                      1xx                                     none
      110             000              714                    60.5
                      001                                     20.0
                      010                                     none
                      1xx                                     none
      111             000              625                    46.3
                      001                                     15.3
                      010                                     none
                      1xx                                     none


    cnfg_cal.bit.en_vcal
      Calibration Source (VCALP and VCALN) Enable
      0 = Calibration sources disabled (needed for internal BIOZ resistance test)
      1 = Calibration sources enabled


  */

  cnfg_bmux.all    = readRegister24(MAX30001_CNFG_BMUX);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);
  cnfg_cal.all     = readRegister24(MAX30001_CNFG_CAL);

  // Do not route VCAL while using impedance test.
  cnfg_bmux.bit.calp_sel = 0U;
  cnfg_bmux.bit.caln_sel = 0U;

  if (!enable) {
    cnfg_bioz_lc.bit.en_bistr = 0U;
    cnfg_bmux.bit.en_bist = 0U;
    cnfg_bmux.bit.rmod = 0b100; // explicit unmodulated state

    writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
    writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);
    writeRegister(MAX30001_CNFG_CAL, cnfg_cal.all);

    BIOZ_test_rnom = 0.0f;
    BIOZ_test_rmod = 0.0f;
    BIOZ_test_frequency = 0.0f;
    updateGlobalRCAL_freq();
    LOGI("BIOZ impedance test disabled.");
    return;
  }

  // RMOD BIST requires VCAL disabled.
  cnfg_cal.bit.vcal = 0U;

  if (useHighResistance) {
    if (rnomValue > 3U) {
      LOGW("High-resistance selector %u invalid; clamping to 3.", rnomValue);
      rnomValue = 3U;
    }

    cnfg_bioz_lc.bit.en_bistr = 1U;
    cnfg_bioz_lc.bit.bistr = rnomValue;

    // High-resistance mode does not use RMOD modulation.
    cnfg_bmux.bit.en_bist = 0U;
    cnfg_bmux.bit.rmod = 0b100; // unmodulated
    if (enableModulation) {
      LOGW("High-resistance test does not support RMOD modulation. Ignoring enableModulation.");
    }
  } else {
    // Low-resistance RMOD BIST mode.
    cnfg_bioz_lc.bit.en_bistr = 0U;
    cnfg_bmux.bit.en_bist = 1U;
    cnfg_bmux.bit.rnom = static_cast<uint8_t>(rnomValue & 0x07U);
    cnfg_bmux.bit.fbist = static_cast<uint8_t>(modFreq & 0x03U);

    // Datasheet recommendation for RMOD BIST.
    cnfg_bmux.bit.openp = 1U;
    cnfg_bmux.bit.openn = 1U;

    if (enableModulation) {
      // Table 38: RMOD=010 valid only for RNOM 000..010.
      const uint8_t max_rmod = (cnfg_bmux.bit.rnom <= 0b010U) ? 0b010U : 0b001U;
      if (rmodValue > max_rmod) {
        LOGW("RMOD=%u not supported for RNOM=%u; clamping to %u.",
             rmodValue, cnfg_bmux.bit.rnom, max_rmod);
        rmodValue = max_rmod;
      }
      cnfg_bmux.bit.rmod = static_cast<uint8_t>(rmodValue & 0x07U);
    } else {
      cnfg_bmux.bit.rmod = 0b100; // 1xx -> unmodulated (datasheet)
    }
  }

  writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
  writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);
  writeRegister(MAX30001_CNFG_CAL, cnfg_cal.all);

  updateGlobalBIOZ_test_impedance();
  updateGlobalRCAL_freq();
  LOGI("BIOZ impedance test configured.");
}
