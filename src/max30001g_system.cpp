/******************************************************************************************************/
// General System Functions
/******************************************************************************************************/

#include <Arduino.h>
#include "logger.h"      // Logging 
#include "max30001g.h"

void MAX30001G::refreshTimingGlobals(void) {
  /*
   * Rebuild all timing/filter globals from current register contents.
   * This keeps globals coherent after direct register writes by advanced code.
   */
  cnfg_gen.all = readRegister24(MAX30001_CNFG_GEN);
  setFMSTR(static_cast<uint8_t>(cnfg_gen.bit.fmstr));
}

HealthCheckResult MAX30001G::healthCheck(void) { // Perform a health check and return the result
  HealthCheckResult result;
  result.spi_ok = spiCheck();
  result.info_reg = readRegister24(MAX30001_INFO);
  result.status_reg = readStatusAndLatchFlags();
  result.pll_unlocked = ((result.status_reg & MAX30001_STATUS_PLLINT) != 0U);
  const uint32_t faultMask = MAX30001_STATUS_EOVF |
                             MAX30001_STATUS_BOVF |
                             MAX30001_STATUS_BOVER |
                             MAX30001_STATUS_BUNDR |
                             MAX30001_STATUS_DCLOFFINT |
                             MAX30001_STATUS_PLLINT;
  result.fault_present = ((result.status_reg & faultMask) != 0U);
  return result;
}

void MAX30001G::synch(void){
    /* Synchronize the device 
     *  This affects measurement process
    */
      writeRegister(MAX30001_SYNCH, 0x000000);  
    }
    
void MAX30001G::swReset(void){
    /* Reset Device */
      writeRegister(MAX30001_SW_RST, 0x000000);
      delay(100);
    }
    
void MAX30001G::setFMSTR(uint8_t fmstr_select){ 
    /*
    * Set Base Clock Frequency. 
    *  This will affect the timing of the ECG and BIOZ measurements.
    *
    *  FMSTR[1:0] Mapping:
    *  0 0b00 = 32768 Hz = FCLK
    *  1 0b01 = 32000 Hz = FCLK*625/640
    *  2 0b10 = 32000 Hz = FCLK*625/640
    *  3 0b11 = 31968.78 Hz = FCLK*640/656
    * 
    *  The following global variables are updated
    *    fmstr
    *    tres
    *    ECG_progression
    *    Rtor resolution
    *    RtoR delay
    *    CAL_resolution
    *
    *    ECG_samplingRate
    *    BIOZ_samplingRate
    *    ECG_lpf
    *    ECG_latency
    *    BIOZ_dlpf
    *    BIOZ Frequency
    */
    
      fmstr_select &= 0x03;

      cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);
    
      // Apply FMSTR
      cnfg_gen.bit.fmstr = fmstr_select;
      writeRegister(MAX30001_CNFG_GEN, cnfg_gen.all);
    
      // Update Global Variables
      switch (cnfg_gen.bit.fmstr) {
    
          case 0b00: // FMSTR = 32768Hz, TRES = 15.26µs, 512Hz
              fmstr           = 32768.0f;  // Hz
              tres            = 15.26;    // µs
              ECG_progression = 512.0;    // per second
              RtoR_resolution = 7.8125;   // ms
              RtoR_delay      = 102.844;  // ms
              CAL_resolution  = 30.52;    // µs
              break;
    
          case 0b01: // FMSTR = 32000Hz, TRES = 15.63µs, 500Hz
              fmstr           = 32000.0f;
              tres            = 15.63;
              ECG_progression = 500.0;
              RtoR_resolution = 8.0;
              RtoR_delay      = 105.313;  // ms
              CAL_resolution  = 31.25;    // µs
              break;
    
          case 0b10: // FMSTR = 32000Hz, TRES = 15.63µs, 200Hz
              fmstr           = 32000.0f;
              tres            = 15.63;
              ECG_progression = 200.0;
              RtoR_resolution = 8.0;
              RtoR_delay      = 105.313;  // ms
              CAL_resolution  = 31.25;    // µs
              break;
    
          case 0b11: // FMSTR = 31968.78Hz, TRES = 15.64µs, 199.8049Hz
              fmstr           = 31968.78f;
              tres            = 15.64;    // µs
              ECG_progression = 199.8049; // Hz
              RtoR_resolution = 8.008;    // ms
              RtoR_delay      = 105.415;  // ms
              CAL_resolution  = 31.28;    // µs
              break;
          
          default:
              LOGE("Error fmstr, can only be 0,1,2,3: given %u", fmstr_select);
              LOGE("All timing set to 0.0. System will not work.");
              fmstr             = 0.0;
              tres              = 0.0;
              ECG_progression   = 0.0;
              RtoR_resolution   = 0.0;
              RtoR_delay        = 0.0; // ms
              CAL_resolution    = 0.0;
              break;
      }
    
      // Read the dependent configuration registers before deriving globals.
      cnfg_ecg.all     = readRegister24(MAX30001_CNFG_ECG);
      cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
      cnfg_bmux.all    = readRegister24(MAX30001_CNFG_BMUX);
      cnfg_cal.all     = readRegister24(MAX30001_CNFG_CAL);
      cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);

      updateGlobalECG_samplingRate();
      updateGlobalBIOZ_samplingRate();
      updateGlobalECG_lpf();
      updateGlobalBIOZ_dlpf();
      updateGlobalCAL_fcal();
      updateGlobalECG_latency();
      updateGlobalBIOZ_frequency();
      updateGlobalBIOZ_test_impedance();
      updateGlobalRCAL_freq();
    
    }
    
void MAX30001G::updateGlobalECG_samplingRate(void){ 
    /* Update ECG Sampling Rate Global */
      // cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);
      // cnfg_ecg.all  = readRegister24(MAX30001_CNFG_ECG);
    
      switch (cnfg_gen.bit.fmstr) {
    
        case 0b00:                            // FMSTR = 32768Hz, TRES = 15.26µs, 512Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b00:
              ECG_samplingRate = 512.;
              break;
            case 0b01:
              ECG_samplingRate = 256.;
              break;
            case 0b10:
              ECG_samplingRate = 128.;
              break;
            default:
              ECG_samplingRate = 0.;
              break;
          }      
          break;
    
        case 0b01:                            // FMSTR = 32000Hz, TRES = 15.63µs, 500Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b00:
              ECG_samplingRate = 500.;
              break;
            case 0b01:
              ECG_samplingRate = 250.;
              break;
            case 0b10:
              ECG_samplingRate = 125.;
              break;
            default:
              ECG_samplingRate = 0.;
              break;
          }
          break;
    
        case 0b10:                            // FMSTR = 32000Hz, TRES = 15.63µs, 200Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b10:
              ECG_samplingRate = 200.;
              break;
            default:
              ECG_samplingRate = 0.;
              break;
          }
          break;
    
        case 0b11:                            // FMSTR = 31968.78Hz, TRES = 15.64µs, 199.8049Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b10:
              ECG_samplingRate = 199.8049;
              break;
            default:
              ECG_samplingRate = 0.;
              break;
          }
          break;
        
        default:
          ECG_samplingRate  = 0.;
          break;
      }
    }
    
    void MAX30001G::updateGlobalBIOZ_samplingRate(void){ 
    /* Update Sampling Rate Globla for BIOZ */
      switch (cnfg_gen.bit.fmstr) {
        case 0b00: // FMSTR = 32768Hz
          BIOZ_samplingRate = (cnfg_bioz.bit.rate == 0) ? 64.0 : 32.0;
          break;
        case 0b01: // FMSTR = 32000Hz
          BIOZ_samplingRate = (cnfg_bioz.bit.rate == 0) ? 62.5 : 31.25;
          break;
        case 0b10: // FMSTR = 32000Hz (for lower rates)
          BIOZ_samplingRate = (cnfg_bioz.bit.rate == 0) ? 50.0 : 25.0;
          break;
        case 0b11: // FMSTR = 31968.78Hz
          BIOZ_samplingRate = (cnfg_bioz.bit.rate == 0) ? 49.95 : 24.98;
          break;
        default:
          LOGE("Invalid FMSTR setting. Could not determine BIOZ sampling rate.");
          BIOZ_samplingRate = 0.0;
          break;
      }

    }
    
    void MAX30001G::updateGlobalCAL_fcal(void) {
    /*
     * Frequency calibration for ECG pulses
     * can also be used for BIOZ
     */
    
      switch (cnfg_cal.bit.fcal) {
        case 0b000:
          CAL_fcal = fmstr / 128.0;
          break;
        case 0b001:
          CAL_fcal = fmstr / 512.0;
          break;
        case 0b010:
          CAL_fcal = fmstr / 2048.0;
          break;
        case 0b011:
          CAL_fcal = fmstr / 8192.0;
          break;
        case 0b100:
          CAL_fcal = fmstr / 32768.0;
          break;
        case 0b101:
          CAL_fcal = fmstr / 131072.0;
          break;
        case 0b110:
          CAL_fcal = fmstr / 524288.0;
          break;
        case 0b111:
          CAL_fcal = fmstr / 2097152.0;
          break;
        default:
          CAL_fcal = fmstr / 32768.0;
          break;
      }
    }
    
    void MAX30001G::updateGlobalECG_latency(void) {
    /*
     * ECG delay based on ECG progression and digital HPF
     */
    
      if (ECG_progression >= 512) {
          if (ECG_hpf > 0) {
              ECG_latency = 31.55; // ms
          } else {
              ECG_latency = 19.836; // ms
          }
      } else if (ECG_progression >= 500) {
          if (ECG_hpf > 0) {
              ECG_latency = 32.313; // ms
          } else {
              ECG_latency = 20.313; // ms
          }
      } else if (ECG_progression >= 256) {
          if (ECG_hpf > 0) {
              ECG_latency = 112.610; // ms
          } else {
              ECG_latency = 89.127; // ms
          }
      } else if (ECG_progression >= 250) {
          if (ECG_hpf > 0) {
              ECG_latency = 115.313; // ms
          } else {
              ECG_latency = 91.313; // ms
          }
      } else if (ECG_progression >= 200) {
          if (ECG_hpf > 0) {
              ECG_latency = 68.813; // ms
          } else {
              ECG_latency = 38.813; // ms
          }
      } else if (ECG_progression >= 199.8049) {
          if (ECG_hpf > 0) {
              ECG_latency = 68.881; // ms
          } else {
              ECG_latency = 38.851; // ms
          }
      } else if (ECG_progression >= 128) {
          if (ECG_hpf > 0) {
              ECG_latency = 149.719; // ms
          } else {
              ECG_latency = 102.844; // ms
          }
      } else if (ECG_progression >= 125) {
          if (ECG_hpf > 0) {
              ECG_latency = 153.313; // ms
          } else {
              ECG_latency = 105.313; // ms
          }
      } else {
          ECG_latency = 0.0;
      }
    }
    
    void MAX30001G::updateGlobalBIOZ_frequency(void) {
    /*
     * BIOZ current-generator modulation frequency (CNFG_BIOZ.BIOZ_FCGEN).
     * Datasheet Table 40:
     * 0000,0001,0010,0011 are intentionally offset to reduce ECG/BIOZ crosstalk.
     * 0100..1001 are direct fMSTR divisors.
     * 101x and 11xx map to fMSTR/256.
     */
      switch (cnfg_bioz.bit.fcgen) {
        case 0b0000: BIOZ_frequency = 4.0f      * fmstr; break; // ~128kHz
        case 0b0001: BIOZ_frequency = 2.5f      * fmstr; break; // ~80kHz
        case 0b0010: BIOZ_frequency = 1.25f     * fmstr; break; // ~40kHz
        case 0b0011: BIOZ_frequency = 0.555625f * fmstr; break; // ~18kHz
        case 0b0100: BIOZ_frequency = fmstr /   4.0f; break;    // ~8kHz
        case 0b0101: BIOZ_frequency = fmstr /   8.0f; break;    // ~4kHz
        case 0b0110: BIOZ_frequency = fmstr /  16.0f; break;    // ~2kHz
        case 0b0111: BIOZ_frequency = fmstr /  32.0f; break;    // ~1kHz
        case 0b1000: BIOZ_frequency = fmstr /  64.0f; break;    // ~500Hz
        case 0b1001: BIOZ_frequency = fmstr / 128.0f; break;    // ~250Hz
        default:     BIOZ_frequency = fmstr / 256.0f; break;    // 101x and 11xx
      }
    }
    
    void MAX30001G::updateGlobalBIOZ_test_impedance(void) {
      // If neither high-R load nor RMOD-BIST is enabled, internal impedance test is off.
      if ((cnfg_bioz_lc.bit.en_bistr == 0U) && (cnfg_bmux.bit.en_bist == 0U)) {
        BIOZ_test_rnom = 0.0f;
        BIOZ_test_rmod = 0.0f;
        BIOZ_test_frequency = 0.0f;
        return;
      }

      if (cnfg_bioz_lc.bit.en_bistr == 1) {
        // high resistance mode

        BIOZ_test_rmod = 0.0;                 // [Ohm] BIOZ test resistor modulation value  
        BIOZ_test_frequency = 0.0;            // [Hz] 
        // Nominal Reistance
        if (cnfg_bioz_lc.bit.bistr == 0){
         BIOZ_test_rnom = 27000.0; // 27kOhm 
        } else if (cnfg_bioz_lc.bit.bistr == 1){
          BIOZ_test_rnom = 108000.0; // 108kOhm      
        } else if (cnfg_bioz_lc.bit.bistr == 2){
          BIOZ_test_rnom = 487000.0; // 487kOhm           
        } else if (cnfg_bioz_lc.bit.bistr == 3){
          BIOZ_test_rnom = 1029000.0; // 1,029kOhm      
        } else {
          BIOZ_test_rnom = 0.0; // N.A.
        } 
      } else {
        // low resistance mode

        // Nominal Resistance, Modulated Resistance
        if        (cnfg_bmux.bit.rnom == 0b000) {
          BIOZ_test_rnom = 5000.0;                 // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod = 2960.7;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod = 980.6;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod = 247.5;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          } 
        } else if (cnfg_bmux.bit.rnom == 0b001) {
          BIOZ_test_rnom = 2500.0;                 // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod = 740.4;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod = 245.2;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =  61.9;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b010) {
          BIOZ_test_rnom = 1666.667;               // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod = 329.1;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod = 109.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =  27.5;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b011) {
          BIOZ_test_rnom = 1250.0;                 // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod =  185.1;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod =   61.3;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b100) {
          BIOZ_test_rnom = 1000.0;                 // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod =  118.5;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod =   39.2;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b101) {
          BIOZ_test_rnom = 833.33;                 // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod =  82.3;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod =  27.2;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b110) {
          BIOZ_test_rnom = 714.286;                // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod =   60.5;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod =   20.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =    0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =    0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else if (cnfg_bmux.bit.rnom == 0b111) {
          BIOZ_test_rnom = 625.0;                  // [Ohm] BIOZ test resistor nominal value          
          // mod
          if        (cnfg_bmux.bit.rmod == 0b0000) {
            BIOZ_test_rmod =   46.3;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0001) {
            BIOZ_test_rmod =   15.3;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0010) {
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else if (cnfg_bmux.bit.rmod == 0b0100) {  
            BIOZ_test_rmod =   0.0;                 // [Ohm] BIOZ test resistor modulation value  
          } else {
            BIOZ_test_rmod =   0.0;
          }  
        } else {
          BIOZ_test_rnom = 0.0;                  // [Ohm] BIOZ test resistor nominal value          
          BIOZ_test_rmod = 0.0;                  // [Ohm] BIOZ test resistor modulation value  
        }

        // Frequency
        if (cnfg_bmux.bit.fbist == 0b00) {
          BIOZ_test_frequency = fmstr / 8192.0f;    // ~4Hz at FMSTR=32768
        } else if (cnfg_bmux.bit.fbist == 0b01) {
          BIOZ_test_frequency = fmstr / 32768.0f;   // ~1Hz
        } else if (cnfg_bmux.bit.fbist == 0b10) {
          BIOZ_test_frequency = fmstr / 131072.0f;  // ~0.25Hz
        } else if (cnfg_bmux.bit.fbist == 0b11) {
          BIOZ_test_frequency = fmstr / 524288.0f;  // ~0.0625Hz
        } else {
          LOGE("Invalid fbist setting.");
          BIOZ_test_frequency = 0.0;
        }
      }
    }

    void MAX30001G::updateGlobalECG_lpf(void){ 
    /*
     * ECG digital low-pass filter corner frequency.
     * Values follow datasheet Table 35 including internal remapping
     * for unsupported DLPF/rate combinations.
     */
    
      switch (cnfg_gen.bit.fmstr) {
    
        case 0b00: // FMSTR = 32768Hz, TRES = 15.26µs, 512Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b00:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 40.96;  
                  break;
                case 0b10:
                  ECG_lpf = 102.4;  
                  break;
                case 0b11:
                  ECG_lpf = 153.6;  
                  break;
              }
              break;
            case 0b01:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 40.96;  
                  break;
                case 0b10:
                  ECG_lpf = 102.4;  
                  break;
                case 0b11:
                  ECG_lpf = 40.96;  
                  break;
              }
              break;
            case 0b10:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 28.35;  
                  break;
                case 0b10:
                  ECG_lpf = 28.35;  
                  break;
                case 0b11:
                  ECG_lpf = 28.35;  
                  break;
              }
              break;
            default:
              ECG_lpf = 0.;  
              break;
          }
          break;
    
        case 0b01: // FMSTR = 32000Hz, TRES = 15.63µs, 500Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b00:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 40.;  
                  break;
                case 0b10:
                  ECG_lpf = 100.;  
                  break;
                case 0b11:
                  ECG_lpf = 150.;  
                  break;
              }
              break;
            case 0b01:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 40.;  
                  break;
                case 0b10:
                  ECG_lpf = 100.;  
                  break;
                case 0b11:
                  ECG_lpf = 40.0;  // Table 35: unsupported combo remaps internally to DLPF=01
                  break;
              }
              break;
            case 0b10:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 27.68;  
                  break;
                case 0b10:
                  ECG_lpf = 27.68;  
                  break;
                case 0b11:
                  ECG_lpf = 27.68;  
                  break;
              }
              break;
            default:
              ECG_lpf = 0.;
              break;
          }
          break;
    
        case 0b10: // FMSTR = 32000Hz, TRES = 15.63µs, 200Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b10:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 40.;  
                  break;
                case 0b10:
                  ECG_lpf = 40.;  
                  break;
                case 0b11:
                  ECG_lpf = 40.;  
                  break;
              }
              break;
            default:
              ECG_lpf = 0.;  
              break;
          }
          break;
    
        case 0b11: // FMSTR = 31968.78Hz, TRES = 15.64µs, 199.8049Hz
          switch (cnfg_ecg.bit.rate) {
            case 0b10:
              switch (cnfg_ecg.bit.dlpf){
                case 0b00:
                  ECG_lpf = 0.;  
                  break;
                case 0b01:
                  ECG_lpf = 39.96;  
                  break;
                case 0b10:
                  ECG_lpf = 39.96;  
                  break;
                case 0b11:
                  ECG_lpf = 39.96;  
                  break;
              }
              break;
            default:
              ECG_lpf = 0.;  
              break;
          }
          break;
    
        default:
          ECG_lpf = 0.;
          break;
      }
    }
    
    void MAX30001G::updateGlobalBIOZ_dlpf(void){ 
    /*
     * BIOZ digital low-pass filter corner frequency.
     * Derived from datasheet Table 41:
     *   DLPF=01 -> fMSTR/8000
     *   DLPF=10 -> fMSTR/4000
     *   DLPF=11 -> fMSTR/2000
     * Unsupported combination RATE=1 and DLPF=11 is internally remapped
     * to DLPF=01 by the device and reflected here.
     */

      uint8_t rate = cnfg_bioz.bit.rate & 0x01;
      uint8_t effective_dlpf = cnfg_bioz.bit.dlpf & 0x03;

      // Table 41: RATE=1 with DLPF=11 is unsupported and maps to 01 internally.
      if ((rate == 0x01) && (effective_dlpf == 0x03)) {
        effective_dlpf = 0x01;
      }

      switch (effective_dlpf) {
        case 0x00:
          BIOZ_dlpf = 0.0f;              // bypass
          break;
        case 0x01:
          BIOZ_dlpf = fmstr / 8000.0f;
          break;
        case 0x02:
          BIOZ_dlpf = fmstr / 4000.0f;
          break;
        case 0x03:
          BIOZ_dlpf = fmstr / 2000.0f;
          break;
        default:
          BIOZ_dlpf = 0.0f;
          LOGE("Invalid BIOZ DLPF setting.");
          break;
      }
    }
    
    void MAX30001G::updateGlobalRCAL_freq(void) {
    /*
     * Test resistor modulation frequency.
     * Datasheet Table 37: FBIST 00/01/10/11 -> fMSTR/2^13 /2^15 /2^17 /2^19
     */
    
      switch (cnfg_bmux.bit.fbist) {
        case 0:
          RCAL_freq = fmstr / 8192.0f;
          break;
        case 1:
          RCAL_freq = fmstr / 32768.0f;
          break;
        case 2:
          RCAL_freq = fmstr / 131072.0f;
          break;
        case 3:
          RCAL_freq = fmstr / 524288.0f;
          break;
        default:
          RCAL_freq = 0.0f;
          break;
      }
    }
