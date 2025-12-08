/******************************************************************************************************/
// Configure BIOZ
/******************************************************************************************************/
#include "logger.h"      // Logging 
#include "max30001g_globals.h"
#include "max30001g_comm.h"        // SPI communication
#include "max30001g_defs.h"        // Register definitions
#include "max30001g_configure_bioz.h"


void MAX30001G::setBIOZSamplingRate(uint8_t BIOZ) {
  /*
  Set the BIOZ sampling rate for the AFE, 
  0 =  low  25-32 * default
  1 = high  50-64
  Based on FMSTR, sets global variable BIOZ_samplingRate
  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);

  // Set the BIOZ sampling rate based on the input
  if (BIOZ == 0) {
    cnfg_bioz.bit.rate = 0b1; // low
  } else if (BIOZ == 1) {
    cnfg_bioz.bit.rate = 0b0; // high
  } else {
    LOGE("Invalid BIOZ sampling rate input. Only 0 or 1 is allowed.");
    return; // Early return for invalid input
  }  

  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

  updateGlobalBIOZ_samplingRate();

  LOGD("BIOZ Sampling Rate Set: %.2f Hz", BIOZ_samplingRate);

}

void MAX30001G::setBIOZgain(uint8_t gain, bool lowNoise) {
/*
  Set the BIOZ gain for the AFE
    0 = 10V/V * default
    1 = 20V/V
    2 = 40V/V
    3 = 80V/V

  Set Low Noise or Low Power mode
    0 = Low Power * default
    1 = Low Noise
*/

  // Read the BIOZ configuration register
  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);

  if (gain <= 3) {
    
    // Set the gain value
    cnfg_bioz.bit.gain = gain;
    
    // Set the global variable based on the selected gain
    switch (gain) {
      case 0:
        BIOZ_gain = 10;  // 10V/V
        break;
      case 1:
        BIOZ_gain = 20;  // 20V/V
        break;
      case 2:
        BIOZ_gain = 40;  // 40V/V
        break;
      case 3:
        BIOZ_gain = 80;  // 80V/V
        break;
    }

    LOGI("BIOZ gain set to %u V/V", BIOZ_gain);
  } else {
    LOGE("Invalid BIOZ gain value: %u. Valid range is 0-3.", gain);
  }

  if (lowNoise) {    
    // Set the low noise value
    cnfg_bioz.bit.ln_bioz = 1;
    LOGI("BIOZ set to Low Noise mode.");
  } else {
    // Set the low noise value
    cnfg_bioz.bit.ln = 0;
    LOGI("BIOZ set to Low Power mode.");
  }

  // Write the updated register back
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

}

void MAX30001G::setBIOZModulationFrequencyByFrequency(uint16_t frequency) {

/*
  Set the BIOZ modulation frequency.

  Selecting numer in this range results in following frequency:
   Range          Frequency         Currents
         > 104000 approx 128kHz     55 - 96000
   60000 - 104000 approx 80kHz      55 - 96000
   29000 - 60000  approx 40kHz      55 - 96000
   13000 - 29000  approx 18kHz      55 - 96000
    6000 - 13000  approx  8kHz      55 - 80000
    3000 - 6000   approx  4kHz      55 - 32000
    1500 - 3000   approx  2kHz      55 - 16000
     750 - 1500   approx  1kHz      55 -  8000
     375 - 750    approx  0.5 kHz   55 -  8000
     187 - 375    approx  0.25 kHz  55 -  8000
      >0 - 187    approx  0.125 kHz 55 -  8000
       0          OFF               OFF         this is not allowed

  After setting the the frequency, the current magnitude will need to be set.

  BIOZ CURRENT GENERATOR MODULATION FREQUENCY (Hz) AND ALLOWED CURRENT MAGINTUDE
  ------------------------------------------------------------------------------
  BIOZ_FCGEN[3:0]  FMSTR[1:0] = 00 FMSTR[1:0] = 01  FMSTR[1:0] = 10  FMSTR[1:0] = 11   CMAG allowed
                   fMSTR= 32,768Hz fMSTR = 32,000Hz fMSTR = 32,000Hz fMSTR = 31,968Hz  
  0000             131,072         128,000          128,000         127,872            all
  0001              81,920          80,000           80,000          81,920            all
  0010              40,960          40,000           40,000          40,960            all
  0011              18,204          17,780           17,780          18,204            all
  0100               8,192           8,000            8,000           7,992            not 111
  0101               4,096           4,000            4,000           3,996            000 to 011
  0110               2,048           2,000            2,000           1,998            000 to 010
  0111               1,024           1,000            1,000             999            000, 001
  1000                 512             500              500             500            000, 001
  1001                 256             250              250             250            000, 001
  101x,11xx            128             125              125             125            000, 001

  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);

  // Helper lambda to assign BIOZ_frequency based on FMSTR
  auto setBIOZFrequency = [&](uint32_t f32768, uint32_t f32000_1, uint32_t f32000_2, uint32_t f31968) {
    switch (cnfg_gen.bit.fmstr) {
      case 0b00: BIOZ_frequency = f32768;   break;
      case 0b01: BIOZ_frequency = f32000_1; break;
      case 0b10: BIOZ_frequency = f32000_2; break;
      case 0b11: BIOZ_frequency = f31968;   break;
      default: BIOZ_frequency = 0; break;
    }
  };

  if (frequency >= 104000) {
    cnfg_bioz.bit.fcgen = 0b0000;
    setBIOZFrequency(131072, 128000, 128000, 127872);
  } else if (frequency > 60000) {
    cnfg_bioz.bit.fcgen = 0b0001;
    setBIOZFrequency( 81920,  80000,  80000,  81920);
  } else if (frequency > 29000) {
    cnfg_bioz.bit.fcgen = 0b0010;
    setBIOZFrequency( 40960,  40000,  40000,  40960);
  } else if (frequency > 13000) {
    cnfg_bioz.bit.fcgen = 0b0011;
    setBIOZFrequency( 18204,  17780,  17780,  18204);
  } else if (frequency > 6000) {
    cnfg_bioz.bit.fcgen = 0b0100;
    setBIOZFrequency(  8192,   8000,   8000,   7992);
  } else if (frequency > 3000) {
    cnfg_bioz.bit.fcgen = 0b0101;
    setBIOZFrequency(  4096,   4000,   4000,   3996);
  } else if (frequency > 1500) {
    cnfg_bioz.bit.fcgen = 0b0110;
    setBIOZFrequency(  2048,   2000,   2000,   1998);
  } else if (frequency > 750) {
    cnfg_bioz.bit.fcgen = 0b0111;
    setBIOZFrequency(  1024,   1000,   1000,    999);
  } else if (frequency > 375) {
    cnfg_bioz.bit.fcgen = 0b1000;
    setBIOZFrequency(   512,    500,    500,    500);
  } else if (frequency > 187) {
    cnfg_bioz.bit.fcgen = 0b1001;
    setBIOZFrequency(   256,    250,    250,    250);
  } else if (frequency > 0) {
    cnfg_bioz.bit.fcgen = 0b1010;
    setBIOZFrequency(   128,    125,    125,    125);
  } else {
    // Frequency off or invalid
    LOGE("Invalid BIOZ frequency: %u. Frequency must be greater than 0.", frequency);
    return;
  }

  // Write to the register
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  LOGI("BIOZ frequency set to %5u [Hz]", BIOZ_frequency);

}

void MAX30001G::setBIOZModulationFrequencybyIndex(uint8_t frequency_selector) {

/*
  Set the BIOZ modulation frequency using a selector index.
  // BIOZ CURRENT GENERATOR MODULATION FREQUENCY (Hz)
  // BIOZ_FCGEN[3:0]  FMSTR[1:0] = 00 FMSTR[1:0] = 01  FMSTR[1:0] = 10  FMSTR[1:0] = 11   CMAG allowed
  //                  fMSTR= 32,768Hz fMSTR = 32,000Hz fMSTR = 32,000Hz fMSTR = 31,968Hz  
  // 0000             131,072         128,000          128,000         127,872            all
  // 0001              81,920          80,000           80,000          81,920            all
  // 0010              40,960          40,000           40,000          40,960            all
  // 0011              18,204          17,780           17,780          18,204            all
  // 0100               8,192           8,000            8,000           7,992            not 111
  // 0101               4,096           4,000            4,000           3,996            000 to 011
  // 0110               2,048           2,000            2,000           1,998            000 to 010
  // 0111               1,024           1,000            1,000             999            000, 001
  // 1000                 512             500              500             500            000, 001
  // 1001                 256             250              250             250            000, 001
  // 101x,11xx            128             125              125             125            000, 001
*/

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);

  if (frequency_selector <= 0b1010) {
    cnfg_bioz.bit.fcgen = frequency_selector;

    // Helper lambda to assign BIOZ_frequency based on FMSTR
    auto setBIOZFrequency = [&](uint32_t f32768, uint32_t f32000_1, uint32_t f32000_2, uint32_t f31968) {
      switch (cnfg_gen.bit.fmstr) {
        case 0b00: BIOZ_frequency = f32768; break;
        case 0b01: BIOZ_frequency = f32000_1; break;
        case 0b10: BIOZ_frequency = f32000_2; break;
        case 0b11: BIOZ_frequency = f31968; break;
        default:   BIOZ_frequency = 0; break;
      }
    };

    switch (frequency_selector) {
      case 0b0000: setBIOZFrequency(131072, 128000, 128000, 127872); break;
      case 0b0001: setBIOZFrequency( 81920,  80000,  80000,  81920); break;
      case 0b0010: setBIOZFrequency( 40960,  40000,  40000,  40960); break;
      case 0b0011: setBIOZFrequency( 18204,  17780,  17780,  18204); break;
      case 0b0100: setBIOZFrequency(  8192,   8000,   8000,   7992); break;
      case 0b0101: setBIOZFrequency(  4096,   4000,   4000,   3996); break;
      case 0b0110: setBIOZFrequency(  2048,   2000,   2000,   1998); break;
      case 0b0111: setBIOZFrequency(  1024,   1000,   1000,    999); break;
      case 0b1000: setBIOZFrequency(   512,    500,    500,    500); break;
      case 0b1001: setBIOZFrequency(   256,    250,    250,    250); break;
      case 0b1010: setBIOZFrequency(   128,    125,    125,    125); break;
      default: 
        LOGE("Invalid frequency selector: %u", frequency_selector);
        return;
    }

    writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
    LOGI("BIOZ frequency set to %5u [Hz]", BIOZ_frequency);
  } else {
    LOGE("Invalid frequency selector: %u", frequency_selector);
  }
}

void MAX30001G::setBIOZmag(uint32_t current){
/*
  Setting current magnitude
    range is 50 to 96000 nA
  This adjusts current to be in the range allowed by modulation frequency
  8 microA can be used at all frequencies
  This also programs the BIOZ common mode current feeback resistor
  Before using this function, modualtion frequency needs to be set
  High current gives better readings for measurement on high resitance samples

  High Current is 8000nA and larger
*/
  
  cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);

  // high current 
  // --------------------------------------------------------------------
  //   cmag >= 001
  //   and cnfg_bioz_lc.bit.hi_lob == 1 
  //
  //BioZ Current Generator Magnitude: cnfg_bioz.cgmag
  // 000 = Off (DRVP and DRVN floating, Current Generators Off)
  // 001 = 8µA (also use this setting when BIOZ_HI_LOB = 0)
  // 010 = 16µA
  // 011 = 32µA
  // 100 = 48µA
  // 101 = 64µA
  // 110 = 80µA
  // 111 = 96µA

  // low current
  // ---------------------------------------------------------------------
  //   cmag == 001 
  //   and cnfg_bioz_lc.bit.hi_lob == 0 
  //   cnfg_bioz_lc.bit.lc2x == 0 55-1100nA
  //   cnfg_bioz_lc.bit.lc2x == 1 110-1100nA
  //   set the common mode resistance as recommended in BIOZ_CMRES.
  //
  // BIOZ Low Current Generator Magnitude: cnfg_bioz_lc.bit.cmag_lc
  //         LC2X = 0 LC2X = 1
  // 0000    0        0
  // 0001    55nA     110nA
  // 0010    110nA    220nA
  // 0011    220nA    440nA
  // 0100    330nA    660nA
  // 0101    440nA    880nA
  // 0110    550nA    1100nA

  // Make sure requested current is achievable with current frequency 
  // ----------------------------------------------------------------
  // BIOZ CURRENT GENERATOR MODULATION FREQUENCY (Hz)                                     CNFG_BIOZ
  // BIOZ_FCGEN[3:0]  FMSTR[1:0] = 00 FMSTR[1:0] = 01  FMSTR[1:0] = 10  FMSTR[1:0] = 11   CGMAG allowed
  //                  fMSTR= 32,768Hz fMSTR = 32,000Hz fMSTR = 32,000Hz fMSTR = 31,968Hz  
  // 0000             131,072         128,000          128,0001        127,872            all
  // 0001              81,920          80,000           80,000          81,920            all
  // 0010              40,960          40,000           40,000          40,960            all
  // 0011              18,204          17,780           17,780          18,204            all
  // 0100               8,192           8,000            8,000           7,992            not 111
  // 0101               4,096           4,000            4,000           3,996            000 to 011
  // 0110               2,048           2,000            2,000           1,998            000 to 010
  // 0111               1,024           1,000            1,000             999            000, 001
  // 1000                 512             500              500             500            000, 001
  // 1001                 256             250              250             250            000, 001
  // 101x,11xx            128             125              125             125            000, 001

  // Limit current based on the modulation frequency settings (fcgen)
  if (cnfg_bioz.bit.fcgen >= 0b0111) {
    if (current > 8000) {
      current = 8000;
      LOGW("Current cannot exceed 8000 nA at this frequency.");
    }
  } else if (cnfg_bioz.bit.fcgen == 0b0110) {
    if (current > 16000) {
      current = 16000;
      LOGW("Current cannot exceed 16000 nA at this frequency.");
    }
  } else if (cnfg_bioz.bit.fcgen == 0b0101) {
    if (current > 32000) {
      current = 32000;
      LOGW("Current cannot exceed 32000 nA at this frequency.");
    }
  } else if (cnfg_bioz.bit.fcgen == 0b0100) {
    if (current > 96000) {
      current = 96000;
      LOGW("Current cannot exceed 96000 nA at this frequency.");
    }
  }

  // Disable if current is 0
  if (current == 0) {
    cnfg_bioz.cmag = 0b000;  // Turn off current generators
    BIOZ_cgmag = 0;
    LOGI("BioZ current generator disabled.");
    return;  // Exit early as no other operations are needed
  }

  // Set High Current Mode --------------------------------

  // Values are in Nano Amps
  if (current >= 8000) {
    cnfg_bioz_lc.bit.hi_lob = 1;
    if        (current < 12000){
      cnfg_bioz.cmag = 0b001;
      BIOZ_cgmag = 8000;
    } else if (current < 24000) {
      cnfg_bioz.cmag = 0b010;
      BIOZ_cgmag = 16000;
    } else if (current < 40000) {
      cnfg_bioz.cmag = 0b011;
      BIOZ_cgmag = 32000;
    } else if (current < 56000) {
      cnfg_bioz.cmag = 0b100;
      BIOZ_cgmag = 48000;
    } else if (current < 72000) {
      cnfg_bioz.cmag = 0b101;
      BIOZ_cgmag = 64000;
    } else if (current < 88000) {
      cnfg_bioz.cmag = 0b110;
      BIOZ_cgmag = 80000;
    } else if (current == 96000) {
      cnfg_bioz.cmag = 0b111;
      BIOZ_cgmag = 96000;
    }
  } 
  
  // Set Low current Mode ------------------------------
  // values are in Nano Amps
  else {
    cnfg_bioz.cmag = 0b001;
    cnfg_bioz_lc.bit.hi_lob = 0;

    if          (current < 80){
      cnfg_bioz_lc.bit.lc2x = 0;
      cnfg_bioz_lc.bit.cmag_lc = 0b001;
      BIOZ_cgmag = 55;
    } else {
      cnfg_bioz_lc.bit.lc2x = 1;
      if        (current < 165){
        cnfg_bioz_lc.bit.cmag_lc = 0b001;
        BIOZ_cgmag = 110;
      } else if (current < 330){
        cnfg_bioz_lc.bit.cmag_lc = 0b010;
        BIOZ_cgmag = 220;
      } else if (current < 550){
        cnfg_bioz_lc.bit.cmag_lc = 0b011;
        BIOZ_cgmag = 440;
      } else if (current < 770){
        cnfg_bioz_lc.bit.cmag_lc = 0b100;
        BIOZ_cgmag = 660;
      } else if (current < 990){
        cnfg_bioz_lc.bit.cmag_lc = 0b101;
        BIOZ_cgmag = 880;
      } else {
        cnfg_bioz_lc.bit.cmag_lc = 0b110;
        BIOZ_cgmag = 1100;
      }
    } 
  }
  
  // Adjust Common Mode Current Feedback 
  // -----------------------------------
  //
  // There is feedback for high current mode
  // There is feedback for low current mode
  // BIOZ_cmag is in [nA]
  //
  // High Current 8000 - 96000 nA
  // ----------------------------
  // active and passive(resistive) current mode feedback is vailable (BMUX_CG_MODE)
  //
  // Options for current feedback:
  // Passive:
  //  1) external 324kOhm resistor on R_BIAS to A_GND, EXT_RBIAS = on
  //     MediBrick board has resistor in place
  //  2) BISTR [0] 27k, [1] 108k, [2] 487k, [3] 1029k, EN_BISTR = on
  //     This is usually used for calibration
  // Active:
  //  3) no external no internal, uses active
  //
  // Low Current 55 - 1100 nA
  // ------------------------
  // only passive(resistive) common mode current feedback 
  //
  // cnfg_bioz_lc.cmres needs to be matched to current magnitude
  // It should be approximately  5000 / (drive current [µA]) [kΩ]
  // available are: 5, 5.5, 6.5, 7.5, 10, 12.5, 20, 40, 80, 100, 160, 320, MOhm

  // Adjust Common Mode Current Feedback based on the current mode (high or low)
  if (cnfg_bioz_lc.bit.hi_lob == 1){
    // High Current Common Mode: enable external resistor
    cnfg_bioz.bit.ext_bias = 1;
    LOGI("External BioZ common mode current feedback resistor for high current enabled.");
  } else {
    // Low Current Common Mode: select apprropriate internal resistor
    uint32_t cmres = 5000 * 1000 / BIOZ_cgmag; // in kOhm
    if        (cmres <= 5250) {
      cnfg_bioz_lc.bit.cmres = 0b1111; 
      BIOZ_cmres = 5000;
    } else if (cmres <= 6000) {
      cnfg_bioz_lc.bit.cmres = 0b1110; 
      BIOZ_cmres = 5500;
    } else if (cmres <= 7000) {
      cnfg_bioz_lc.bit.cmres = 0b1101; 
      BIOZ_cmres = 6500;
    } else if (cmres <= 8750) {
      cnfg_bioz_lc.bit.cmres = 0b1100; 
      BIOZ_cmres = 7500;
    } else if (cmres <= 11250) {
      cnfg_bioz_lc.bit.cmres = 0b1011; 
      BIOZ_cmres = 10000;
    } else if (cmres <= 16250) {
      cnfg_bioz_lc.bit.cmres = 0b1010; 
      BIOZ_cmres = 12500;
    } else if (cmres <= 30000) {
      cnfg_bioz_lc.bit.cmres = 0b1001; 
      BIOZ_cmres = 20000;
    } else if (cmres <= 60000) {
      cnfg_bioz_lc.bit.cmres = 0b1000; 
      BIOZ_cmres = 40000;
    } else if (cmres <= 90000) {
      cnfg_bioz_lc.bit.cmres = 0b0111; 
      BIOZ_cmres = 80000;
    } else if (cmres <= 13000) {
      cnfg_bioz_lc.bit.cmres = 0b0101; 
      BIOZ_cmres = 100000;
    } else if (cmres <= 240000) {
      cnfg_bioz_lc.bit.cmres = 0b0011; 
      BIOZ_cmres = 160000;
    } else if (cmres >240000) {
      cnfg_bioz_lc.bit.cmres = 0b0001; 
      BIOZ_cmres = 320000;
    }

    LOGI("BioZ common mode current feedback resistance for low current set to %5u [kΩ]", BIOZ_cmres);
  }

  writeRegister(MAX30001_CNFG_BIOZ,    cnfg_bioz.all);
  writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);

  LOGI("BioZ current set to %5u [nA]", BIOZ_cgmag);
}

void MAX30001G::setBIOZPhaseOffsetbyPhase(uint16_t frequency, float phase){
  // frequency: 128,000, 80,000, < 80,000
  float phase_offset = 0.0;
  float max_phase = 0.0;
  uint8_t phase_selector = 0;

  if (frequency >= 100000) {
    // 128KHz
    phase_offset = 45.0;
    max_phase = 135.0;
  } else if (frequency >=80000) {
    // 80kHz
    phase_offset = 22.5;
    max_phase = 157.5;
  } else {
    // < 80kHz
     phase_offset = 11.25;
     max_phase = 168.75;
  }

  if (phase > max_phase) {
    phase = max_phase;
    LOGW("Phase limited to %f for modulation frequency.", max_phase);
  } 

  phase_selector  =  floor(phase / phase_offset);

  setBIOZPhaseOffset(phase_selector)
}

void MAX30001G::setBIOZPhaseOffsetbyIndex(uint8_t selector){
/**
  * Freq   FCGEN  Phase               Phase selector.
  * lower  other  0.. 11.25 ..168.75  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
  * 80kHz  0b0001 0.. 22.5  ..157.50  0,1,2,3,4,5,6,7
  * 128kHz 0b0000 0.. 45.0  ..135     0,1,2,3
  * 
  * BIOZ_phase is a global variable representing current phase setting
  * 
  * For quadrature measurement we want phase at 0 and 90deg
  *   magnitude is sqrt(R_0^2 + R_90^2)
  *   phase is atan(R_90/R_0)
  * 
  * If we measure multiple phases we need to fit curve
  *   measured R(offset) = mag*cos(phase+offset) 
  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  
  switch (cnfg_bioz.bit.fcgen) {
    case 0b0000: // 45-degree phase steps
      // 128kHz
      if selector > 3 {
        selector = 3;
        LOGW("Phase offset selector limited to 0..3 for 128kHz modulation frequency.");
      }
      BIOZ_phase = selector * 45.0;
      break;
    case 0b0001: // 22.5-degree phase steps
      // 80kHz
      if selector > 7 {
        selector = 7;
        LOGW("Phase offset selector limited to 0..7 for 80kHz modulation frequency.");
      }
      BIOZ_phase = selector * 22.5;
      break;
    default:     // Other values use 11.25-degree phase steps
      // 40kHz, 18kHz, 8kHz, 4kHz, 2kHz, 1kHz, 0.5kHz, 0.25kHz, 0.125kHz
      if selector > 15 {
        selector = 15;
        LOGW("Phase offset selector limited to 0..15 for 40kHz and lower modulation frequencies.");
      }
      BIOZ_phase = selector * 11.25;
      break;

  }

  cnfg_bioz.bit.phoff = selector;

  // Store register
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  
  // Log the updated phase for debugging
  LOGI("BIOZ phase set to %.2f degrees", BIOZ_phase);
}

void MAX30001G::setBIOZfilter(uint8_t ahpf, uint8_t lpf, uint8_t hpf) {
/*
 analog high pass is 
  [6,7]  0 Hz
  [0]   60 Hz
  [1]  150 Hz
  [2]  500 Hz * default
  [3] 1000 Hz
  [4] 2000 Hz 
  [5] 4000 Hz
  analog filter is located before demodulator

 digital low pass is approx
  [0] 0  Hz
  [1] 4  Hz * default
  [2] 8  Hz
  [3] 16 Hz 
  depending on FMSTR and BIOZ sampling rate

 digital high pass is 
  [0] 0    Hz * default
  [1] 0.05 Hz 
  [2] 0.5  Hz
 
 
 digital filters are after demodulator
*/

  cnfg_bioz.all  = readRegister24(MAX30001_CNFG_BIOZ);

  if (hpf == 0) {
    cfg_bioz.bit.dhpf = 0b00;
    BIOZ_dhpf = 0.;
  } else if (hpf ==1) {
    cfg_bioz.bit.dhpf = 0b01;
    BIOZ_dhpf = 0.05;
  } else if (hpf >= 2) {
    cfg_bioz.bit.dhpf = 0b10;
    BIOZ_dhpf = 0.5;
  }

  if (lpf == 0) {
    cfg_bioz.bit.dlpf = 0b00;
  } else if (hpf == 1) {
    cfg_bioz.bit.dlpf = 0b01;
  } else if (hpf == 2) {
    cfg_bioz.bit.dlpf = 0b10;
  } else if (hpf == 3) {
    cfg_bioz.bit.dlpf = 0b10;
  }
  updateGlobalBIOZ_dlpf();

  switch (ahpf) {
   case 0:
    cfg_bioz.bit.ahpf = 0b000;
    BIOZ_ahpf = 60.;
    break;

   case 1:
    cfg_bioz.bit.ahpf = 0b001;
    BIOZ_ahpf = 150.;
    break;

   case 2:
    cfg_bioz.bit.ahpf = 0b010;
    BIOZ_ahpf = 500.;
    break;

   case 3:
    cfg_bioz.bit.ahpf = 0b011;
    BIOZ_ahpf = 1000.;
    break;

   case 4:
    cfg_bioz.bit.ahpf = 0b100;
    BIOZ_ahpf = 2000.;
    break;

   case 5:
    cfg_bioz.bit.ahpf = 0b101;
    BIOZ_ahpf = 4000.;
    break;

   default:
    cnfg_bioz.bit.ahpf = 0b110; // Set to 0 Hz, bypass filter
    BIOZ_ahpf = 0.0;
    break;
  }

  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

  LOGI("BIOZ filter configured: HPF=%.2f Hz, LPF=%.2f Hz, AHPF=%.2f Hz", BIOZ_ahpf, BIOZ_lpf, BIOZ_ahpf);

}

void MAX30001G::setBIOZmodulation(uint8_t mode) {
/*  
 0 = Unchopped Sources with Low Pass Filter * default
     (higher noise, excellent 50/60Hz rejection, recommended for ECG, BioZ applications)
 1 = Chopped Sources without Low Pass Filter
     (low noise, no 50/60Hz rejection, recommended for BioZ applications with digital LPF, possibly battery powered ECG, BioZ applications)
 2 = Chopped Sources with Low Pass Filter
     (low noise, excellent 50/60Hz rejection)
 3 = Chopped Sources with Resistive CM
     (Not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance)
 */

  if (mode > 3) {
    LOGI("Invalid mode value. It must be between 0 and 3.");
    return; // Exit the function if the mode is invalid
  }
  
  // Read the current BMUX register
  cnfg_bmux.all = readRegister24(MAX30001_CNFG_BMUX);

  // Set the modulation mode to chopped with resistive CM
  cnfg_bmux.bit.cg_mode = mode; // Assuming 0b11 is for "chopped with resistive CM"

  // Write back the modified register
  writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);

  switch (mode) {
    case 0:
      LOGI("BIOZ modulation mode set to: Unchopped Sources with Low Pass Filter.");
      break;
    case 1:
      LOGI("BIOZ modulation mode set to: Chopped Sources without Low Pass Filter.");
      break;
    case 2:
      LOGI("BIOZ modulation mode set to: Chopped Sources with Low Pass Filter.");
      break;
    case 3:
      LOGI("BIOZ modulation mode set to: Chopped Sources with Resistive CM.");
      break;
  }
}

void setBIOZCurrentMonting(bool enable) {
  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz.bit.cgmon = enable;
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  LOGI("BIOZ current monitoring %s", enable ? "enabled" : "disabled");
}

/******************************************************************************************************/
// Set BIOZ impedance test
/******************************************************************************************************/

void setDefaultNoBIOZImpedanceTest() {
  setBIOZImpedanceTest(
    false, // disable
    false, // useHighResistance
    false, // enableModulation
    0b000, // resistance 5kOhm 
    0b000, // 3 Ohm, switches between 5k Ohm and 5k - 3 Ohm
    0b01   // modFreq 1 Hz
    );
}

void MAX30001G::setDefaultBIOZImpedanceTest() {
  setBIOZImpedanceTest(
    true,  // enable 
    false, // useHighResistance
    true,  // enableModulation
    0b000, // resistance 5kOhm 
    0b000, // 3 Ohm, switches between 5k Ohm and 5k - 3 Ohm
    0b01,  // modFreq 1 Hz if (modulation . 
    )  
}

void MAX30001G::setBIOZImpedanceTest(bool enable, bool useHighResistance, bool enableModulation, uint8_t rnomValue, uint8_t rmodValue,  uint8_t modFreq) {
    /*
    Configures the BIOZ impedance test.
    This function attaches an internal resistance to the BIOZ pins (BIP and BIN).

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


    // Read the current register settings
    cnfg_bmux.all     = readRegister24(MAX30001_CNFG_BMUX);
    cnfg_bioz_lc.all  = readRegister24(MAX30001_CNFG_BIOZ_LC);
    cnfg_cal.all      = readRegister24(MAX30001_CNFG_CAL);

    if (enable) {

      cnfg_cal.bit.en_vcal= 0 // disable calibration voltage source VCALP and VCALN 

      if (useHighResistance) {
        // Enable built-in high resistance test
        cnfg_bioz_lc.bit.en_bistr = 1;
        // Set resistance value
        cnfg_bioz_lc.bit.enableModulationt.bistr = rnomValue & 0x03;
        if (enableModulation) {
          LOGW("High resistance test does not support modulation. Disabling modulation");
        }
        // Modulation is not possible
        cnfg_bmux.bit.rmod = 0; // Dsiable modulation resistor byu setting it to 0
        cnfg_bmux.bit.en_bist = 0; // Disable modulation
        LOGD("High resistance test enabled with resistance value: %u", rnomValue);

     } else {
        // useLow Resistance
        cnfg_bmux.bit.en_bist = 1;
        cnfg_bmux.bit.rnorm = rnomValue & 0x07; // nominal resistance 0..7
        LOGD("Low resistance test enabled with nominal resistance: %u", rnomValue);

        // Modulate resistance if requested
        if (modulate) {
            // Enable modulation, set modulation value and frequency
            cnfg_bmux.bit.rmod = rmodValue & 0x07; // Set modulated resistance value
            cnfg_bmux.bit.fbist = modFreq & 0x03; // Set modulation frequency
            updateGlobalRCAL_freq(); // Update the frequency for calibration
            LOGD("Modulation enabled with rmodValue: %u and modFreq: %u", rmodValue, modFreq);
        } else {
            // Disable modulation
            cnfg_bmux.bit.rmod = 0;
            LOGD("Modulation disabled.");
        }
      }

    } else {
        // Disable built-in resistance test
        cnfg_bioz_lc.bit.en_bistr = 0;
        cnfg_bmux.bit.en_bist =0;
        LOGD("BIOZ impedance test disabled.");
    }

    // Write back the modified registers
    writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
    writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);
    writeRegister(MAX30001_CNFG_CAL, cnfg_cal.all);

    updateGlobalBIOZ_test_impedance();

}


