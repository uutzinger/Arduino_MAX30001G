/******************************************************************************************************/
// Registers
/******************************************************************************************************/

#include "logger.h"                  // Logging 
#include "max30001g.h"

/******************************************************************************************************/

void MAX30001G::readAllRegisters() {
  readStatusAndLatchFlags();
  en_int1.all      = readRegister24(MAX30001_EN_INT1);
  en_int2.all      = readRegister24(MAX30001_EN_INT2);
  mngr_int.all     = readRegister24(MAX30001_MNGR_INT);
  mngr_dyn.all     = readRegister24(MAX30001_MNGR_DYN);
  info.all         = readRegister24(MAX30001_INFO);
  cnfg_gen.all     = readRegister24(MAX30001_CNFG_GEN);
  cnfg_cal.all     = readRegister24(MAX30001_CNFG_CAL);
  cnfg_emux.all    = readRegister24(MAX30001_CNFG_EMUX);
  cnfg_ecg.all     = readRegister24(MAX30001_CNFG_ECG);
  cnfg_bmux.all    = readRegister24(MAX30001_CNFG_BMUX);
  cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);
  cnfg_rtor1.all   = readRegister24(MAX30001_CNFG_RTOR1);
  cnfg_rtor2.all   = readRegister24(MAX30001_CNFG_RTOR2);
}

void MAX30001G::saveConfig() {
  _savedConfig.en_int1    = readRegister24(MAX30001_EN_INT1);
  _savedConfig.en_int2    = readRegister24(MAX30001_EN_INT2);
  _savedConfig.mngr_int   = readRegister24(MAX30001_MNGR_INT);
  _savedConfig.mngr_dyn   = readRegister24(MAX30001_MNGR_DYN);
  _savedConfig.cnfg_gen   = readRegister24(MAX30001_CNFG_GEN);
  _savedConfig.cnfg_cal   = readRegister24(MAX30001_CNFG_CAL);
  _savedConfig.cnfg_emux  = readRegister24(MAX30001_CNFG_EMUX);
  _savedConfig.cnfg_ecg   = readRegister24(MAX30001_CNFG_ECG);
  _savedConfig.cnfg_bmux  = readRegister24(MAX30001_CNFG_BMUX);
  _savedConfig.cnfg_bioz  = readRegister24(MAX30001_CNFG_BIOZ);
  _savedConfig.cnfg_bioz_lc = readRegister24(MAX30001_CNFG_BIOZ_LC);
  _savedConfig.cnfg_rtor1 = readRegister24(MAX30001_CNFG_RTOR1);
  _savedConfig.cnfg_rtor2 = readRegister24(MAX30001_CNFG_RTOR2);
  _savedConfig.valid      = true;
}

void MAX30001G::restoreConfig() {
  if (!_savedConfig.valid) {
    LOGW("AFE: restoreConfig() called without a saved snapshot.");
    return;
  }

  writeRegister(MAX30001_EN_INT1,   _savedConfig.en_int1);
  writeRegister(MAX30001_EN_INT2,   _savedConfig.en_int2);
  writeRegister(MAX30001_MNGR_INT,  _savedConfig.mngr_int);
  writeRegister(MAX30001_MNGR_DYN,  _savedConfig.mngr_dyn);
  writeRegister(MAX30001_CNFG_GEN,  _savedConfig.cnfg_gen);
  writeRegister(MAX30001_CNFG_CAL,  _savedConfig.cnfg_cal);
  writeRegister(MAX30001_CNFG_EMUX, _savedConfig.cnfg_emux);
  writeRegister(MAX30001_CNFG_ECG,  _savedConfig.cnfg_ecg);
  writeRegister(MAX30001_CNFG_BMUX, _savedConfig.cnfg_bmux);
  writeRegister(MAX30001_CNFG_BIOZ, _savedConfig.cnfg_bioz);
  writeRegister(MAX30001_CNFG_BIOZ_LC, _savedConfig.cnfg_bioz_lc);
  writeRegister(MAX30001_CNFG_RTOR1, _savedConfig.cnfg_rtor1);
  writeRegister(MAX30001_CNFG_RTOR2, _savedConfig.cnfg_rtor2);

  refreshTimingGlobals();

  readAllRegisters();
}

void MAX30001G::printConfig() {
  readAllRegisters();
  printAllRegisters();
}

void MAX30001G::printAllRegisters() {
  /*
  Read and report all known register
  */
  printStatus();
  printEN_INT1();
  printEN_INT2();
  printMNGR_INT();
  printMNGR_DYN();
  printInfo();
  printCNFG_GEN();
  printCNFG_CAL();
  printCNFG_EMUX();
  printCNFG_ECG();
  printCNFG_BMUX();
  printCNFG_BIOZ();
  printCNFG_BIOZ_LC();
  printCNFG_RTOR1();
  printCNFG_RTOR2();
}

void MAX30001G::readInfo(void)
{
    /*
    Read content of information register.
    This should contain the revision number of the chip in info.revision.
    It should also include part ID but it's not clear how this is stored.
    */
    info.all = readRegister24(MAX30001_INFO);
}

void MAX30001G::readStatusRegisters() {
  // Read status registers to check for over/under-voltage conditions
  readStatusAndLatchFlags();

  // Check over-voltage and under-voltage conditions
  over_voltage_detected = (status.bit.bover == 1);
  under_voltage_detected = (status.bit.bundr == 1);
  // = (status.bit.bovf == 1);
  // = (status.bit.eovf == 1);

}

void MAX30001G::printStatus(void) {
    LOGln("MAX30001 Status Register:");
    LOGln("----------------------------");
  
    LOGln("ECG ------------------------");
  
    if (status.bit.dcloffint == 1) {
      LOGln("Lead ECG leads are off:");
      if (status.bit.ldoff_nl == 1) {
        LOGln("Lead ECGN below VTHL");
      } else if (status.bit.ldoff_nh == 1) {
        LOGln("Lead ECGN above VTHH");
      }
      if (status.bit.ldoff_pl == 1) {
        LOGln("Lead ECGP below VHTL");
      } else if (status.bit.ldoff_ph == 1) {
        LOGln("Lead ECGP above VHTH");
      }
    } else {
      LOGln("Lead ECG leads are on.");
    }
  
    LOGln("ECG FIFO interrupt is %s", status.bit.eint ? "on" : "off");
    LOGln("ECG FIFO overflow is  %s", status.bit.eovf ? "on" : "off");
    LOGln("ECG Sample interrupt  %s", status.bit.samp ? "occurred" : "not present");
    LOGln("ECG R to R interrupt  %s", status.bit.rrint ? "occurred" : "not present");
    LOGln("ECG Fast Recovery interrupt is %s", status.bit.fstint ? "on" : "off");
  
    LOGln("BIOZ -----------------------");
  
    if (status.bit.bcgmon == 1) {
      LOGln("BIOZ leads are off.");
      if (status.bit.bcgmn == 1) { LOGln("BIOZ N lead off"); }
      if (status.bit.bcgmp == 1) { LOGln("BIOZ P lead off"); }
      if (status.bit.bundr == 1) { LOGln("BIOZ output magnitude under BIOZ_LT (4 leads)"); }
      if (status.bit.bover == 1) { LOGln("BIOZ output magnitude over BIOZ_HT (4 leads)"); }
    } else {
      LOGln("BIOZ leads are on.");
    }
  
    LOGln("BIOZ ultra low power leads interrupt is %s", status.bit.lonint ? "on" : "off");
    LOGln("PLL %s", status.bit.pllint ? "has lost signal" : "is working");
  }
  
  void MAX30001G::printEN_INT(max30001_en_int_reg_t en_int) {
    LOGln("MAX30001 Interrupts:");
    LOGln("----------------------------");
    if (en_int.bit.intb_type == 0) {
      LOGln("Interrupts are disabled");
    } else if (en_int.bit.intb_type == 1) {
      LOGln("Interrupt is CMOS driver");
    } else if (en_int.bit.intb_type == 2) {
      LOGln("Interrupt is Open Drain driver");
    } else if (en_int.bit.intb_type == 3) {
      LOGln("Interrupt is Open Drain with 125k pullup driver");
    }
    LOGln("PLL interrupt is                                %s", en_int.bit.en_pllint    ? "enabled" : "disabled");
    LOGln("Sample synch pulse is                           %s", en_int.bit.en_samp      ? "enabled" : "disabled");
    LOGln("R to R detection interrupt is                   %s", en_int.bit.en_rrint     ? "enabled" : "disabled");
    LOGln("Ultra low power leads on detection interrupt is %s", en_int.bit.en_lonint    ? "enabled" : "disabled");
    LOGln("BIOZ current monitor interrupt is               %s", en_int.bit.en_bcgmon    ? "enabled" : "disabled");
    LOGln("BIOZ under range interrupt is                   %s", en_int.bit.en_bunder    ? "enabled" : "disabled");
    LOGln("BIOZ over range interrupt is                    %s", en_int.bit.en_bover     ? "enabled" : "disabled");
    LOGln("BIOZ FIFO overflow interrupt is                 %s", en_int.bit.en_bovf      ? "enabled" : "disabled");
    LOGln("BIOZ FIFO interrupt is                          %s", en_int.bit.en_bint      ? "enabled" : "disabled");
    LOGln("ECG dc lead off interrupt is                    %s", en_int.bit.en_dcloffint ? "enabled" : "disabled");
    LOGln("ECG fast recovery interrupt is                  %s", en_int.bit.en_fstint    ? "enabled" : "disabled");
    LOGln("ECG FIFO overflow interrupt is                  %s", en_int.bit.en_eovf      ? "enabled" : "disabled");
    LOGln("ECG FIFO interrupt is                           %s", en_int.bit.en_eint      ? "enabled" : "disabled");
  }
  
  void MAX30001G::printMNGR_INT(void) {
    LOGln("MAX30001 Interrupt Management:");
    LOGln("----------------------------");
  
    if (mngr_int.bit.samp_it == 0) {
      LOGln("Sample interrupt on every sample");
    } else if (mngr_int.bit.samp_it == 1) {
      LOGln("Sample interrupt on every 2nd sample");
    } else if (mngr_int.bit.samp_it == 2) {
      LOGln("Sample interrupt on every 4th sample");
    } else if (mngr_int.bit.samp_it == 3) {
      LOGln("Sample interrupt on every 16th sample");
    }
  
    LOGln("Sample synchronization pulse is cleared %s", mngr_int.bit.clr_samp ? "automatically" : "on status read");
  
    if (mngr_int.bit.clr_rrint == 0) {
      LOGln("RtoR interrupt is cleared on status read");
    } else if (mngr_int.bit.clr_rrint == 1) {
      LOGln("RtoR interrupt is cleared on RTOR read");
    } else if (mngr_int.bit.clr_rrint == 2) {
      LOGln("RtoR interrupt is cleared automatically");
    } else {
      LOGln("RtoR interrupt clearance is not defined");
    }
  
    LOGln("FAST_MODE interrupt %s", mngr_int.bit.clr_fast ? "remains on until FAST_MODE is disengaged" : "is cleared on status read");
    LOGln("BIOZ FIFO interrupt after %u samples", mngr_int.bit.b_fit + 1);
    LOGln("ECG FIFO interrupt after %u samples", mngr_int.bit.e_fit + 1);
  }
  
  void MAX30001G::printMNGR_DYN(void) {
    LOGln("MAX30001 Dynamic Modes:");
    LOGln("----------------------------");
  
    if (cnfg_gen.bit.en_bloff >= 2) {
      LOGln("BIOZ lead off high threshold: +/- %u * 32", mngr_dyn.bit.bloff_hi_it);
      LOGln("BIOZ lead off low threshold: +/- %u * 32", mngr_dyn.bit.bloff_lo_it);
    } else {
      LOGln("BIOZ lead off thresholds not applicable");
    }
  
    if (mngr_dyn.bit.fast == 0) {
      LOGln("ECG fast recovery is disabled");
    } else if (mngr_dyn.bit.fast == 1) {
      LOGln("ECG manual fast recovery is enabled");
    } else if (mngr_dyn.bit.fast == 2) {
      LOGln("ECG automatic fast recovery is enabled");
      LOGln("ECG fast recovery threshold is %u", 2048 * mngr_dyn.bit.fast_th);
    } else {
      LOGln("ECG fast recovery is not defined");
    }
  }
  
  void MAX30001G::printInfo(void)
  {
    /*
    Print the information register
    */
    LOGln("MAX30001 Information Register:");
    LOGln("----------------------------");
    LOGln("Nibble 1:       %u", info.bit.n1);
    LOGln("Nibble 2:       %u", info.bit.n2);
    LOGln("Nibble 3:       %u", info.bit.n3);
    LOGln("Constant 1: (should be 1) %u", info.bit.c1);
    LOGln("2 Bit Value:    %u", info.bit.n4);
    LOGln("Revision:       %u", info.bit.revision);
    LOGln("Constant 2: (should be 5) %u", info.bit.c2);
  }
  
  void MAX30001G::printCNFG_GEN()
  {
    LOGln("MAX30001 General Config:");
    LOGln("----------------------------");
    LOGln("ECG  is %s", cnfg_gen.bit.en_ecg ? "enabled" : "disabled");
    LOGln("BIOZ is %s", cnfg_gen.bit.en_bioz ? "enabled" : "disabled");
  
    switch (cnfg_gen.bit.fmstr) {
      case 0:
        LOGln("FMSTR is 32768Hz, global var: %f", fmstr);
        LOGln("TRES is 15.26us, global var: %f", tres);
        LOGln("ECG progression is 512Hz, global var: %f", ECG_progression);
        break;
      case 1:
        LOGln("FMSTR is 32000Hz, global var: %f", fmstr);
        LOGln("TRES is 15.63us, global var: %f", tres);
        LOGln("ECG progression is 500Hz, global var: %f", ECG_progression);
        break;
      case 2:
        LOGln("FMSTR is 32000Hz, global var: %f", fmstr);
        LOGln("TRES is 15.63us, global var: %f", tres);
        LOGln("ECG progression is 200Hz, global var: %f", ECG_progression);
        break;
      case 3:
        LOGln("FMSTR is 31968.78Hz, global var: %f", fmstr);
        LOGln("TRES is 15.64us, global var: %f", tres);
        LOGln("ECG progression is 199.8049Hz, global var: %f", ECG_progression);
        break;
      default:
        LOGln("FMSTR is undefined");
        break;
    }
  
    LOGln("--------------------------");
  
    if (cnfg_gen.bit.en_rbias > 0) {
      if (cnfg_gen.bit.en_rbias == 1 && cnfg_gen.bit.en_ecg == 1) {
        LOGln("ECG bias resistor is enabled");
      } else if (cnfg_gen.bit.en_rbias == 2 && cnfg_gen.bit.en_bioz == 1) {
        LOGln("BIOZ bias resistor is enabled");
      } else {
        LOGln("ECG and BIOZ bias resistors are undefined");
      }
      LOGln("N bias resistance is %s", cnfg_gen.bit.rbiasn ? "enabled" : "disabled");
      LOGln("P bias resistance is %s", cnfg_gen.bit.rbiasp ? "enabled" : "disabled");
      switch (cnfg_gen.bit.rbiasv) {
        case 0: LOGln("bias resistor is 50MOhm"); break;
        case 1: LOGln("bias resistor is 100MOhm"); break;
        case 2: LOGln("bias resistor is 200MOhm"); break;
        default: LOGln("bias resistor is not defined"); break;
      }
    } else {
      LOGln("ECG and BIOZ bias resistors are disabled");
    }
  
    LOGln("--------------------------");
  
    if (cnfg_gen.bit.en_dcloff == 1) {
      LOGln("ECG lead off detection is enabled");
      switch (cnfg_gen.bit.vth) {
        case 0: LOGln("ECG lead off/on threshold is VMID +/-300mV"); break;
        case 1: LOGln("ECG lead off/on threshold is VMID +/-400mV"); break;
        case 2: LOGln("ECG lead off/on threshold is VMID +/-450mV"); break;
        case 3: LOGln("ECG lead off/on threshold is VMID +/-500mV"); break;
        default: LOGln("ECG lead off/on threshold is not defined"); break;
      }
      switch (cnfg_gen.bit.imag) {
        case 0: LOGln("ECG lead off/on current source is 0nA"); break;
        case 1: LOGln("ECG lead off/on current source is 5nA"); break;
        case 2: LOGln("ECG lead off/on current source is 10nA"); break;
        case 3: LOGln("ECG lead off/on current source is 20nA"); break;
        case 4: LOGln("ECG lead off/on current source is 50nA"); break;
        case 5: LOGln("ECG lead off/on current source is 100nA"); break;
        default: LOGln("ECG lead off/on current source is not defined"); break;
      }
      LOGln("ECG lead off/on polarity is %s", cnfg_gen.bit.ipol ? "P is pull down" : "P is pull up");
    } else {
      LOGln("ECG lead off detection is disabled");
    }
  
    LOGln("--------------------------");
  
    switch (cnfg_gen.bit.en_ulp_lon) {
      case 0: LOGln("ECG Ultra low power leads on detection is disabled"); break;
      case 1: LOGln("ECG Ultra low power leads on detection is enabled"); break;
      default: LOGln("ECG Ultra low power leads on detection is not defined"); break;
    }
  
    LOGln("--------------------------");
  
    if (cnfg_gen.bit.en_bloff > 0) {
      LOGln("BIOZ lead off detection is enabled");
      switch (cnfg_gen.bit.en_bloff) {
        case 1: LOGln("BIOZ lead off/on is under range detection for 4 electrodes operation"); break;
        case 2: LOGln("BIOZ lead off/on is over range detection for 2 and 4 electrodes operation"); break;
        case 3: LOGln("BIOZ lead off/on is under and over range detection for 4 electrodes operation"); break;
        default: LOGln("BIOZ lead off/on detection is not defined"); break;
      }
    } else {
      LOGln("BIOZ lead off detection is disabled");
    }
  }
  
  void MAX30001G::printCNFG_CAL() {
      LOGln("MAX30001 Internal Voltage Calibration Source:");
      LOGln("---------------------------------------------");
  
      if (cnfg_cal.bit.vcal == 1) {
        LOGln("Internal voltage calibration source is enabled");
        LOGln("Voltage calibration source is %s", cnfg_cal.bit.vmode ? "bipolar" : "unipolar");
        LOGln("Magnitude is %s", cnfg_cal.bit.vmag ? "0.5mV" : "0.25mV");
        switch (cnfg_cal.bit.fcal) {
          case 0: LOGln("Frequency is %u Hz", fmstr / 128); break;
          case 1: LOGln("Frequency is %u Hz", fmstr / 512); break;
          case 2: LOGln("Frequency is %u Hz", fmstr / 2048); break;
          case 3: LOGln("Frequency is %u Hz", fmstr / 8192); break;
          case 4: LOGln("Frequency is %u Hz", fmstr / 32768); break;
          case 5: LOGln("Frequency is %u Hz", fmstr / 131072); break;
          case 6: LOGln("Frequency is %u Hz", fmstr / 524288); break;
          case 7: LOGln("Frequency is %u Hz", fmstr / 2097152); break;
          default: LOGln("Frequency is not defined"); break;
        }
    } else {
      LOGln("Internal voltage calibration source is disabled");
    }
  
    if (cnfg_cal.bit.fifty == 1) {
      LOGln("50%% duty cycle");
    } else {
      LOGln("Pulse length %u [us]", cnfg_cal.bit.thigh * CAL_resolution);
    }
  }
  
  void MAX30001G::printCNFG_EMUX() {
    LOGln("MAX30001 ECG multiplexer:");
    LOGln("-------------------------");
  
    if (cnfg_emux.bit.caln_sel == 0) {
        LOGln("ECG N is not connected to calibration signal");
    } else if (cnfg_emux.bit.caln_sel == 1) {
        LOGln("ECG N is connected to V_MID");
    } else if (cnfg_emux.bit.caln_sel == 2) {
        LOGln("ECG N is connected to V_CALP");
    } else if (cnfg_emux.bit.caln_sel == 3) {
        LOGln("ECG N is connected to V_CALN");
    }
  
    if (cnfg_emux.bit.calp_sel == 0) {
        LOGln("ECG P is not connected to calibration signal");
    } else if (cnfg_emux.bit.calp_sel == 1) {
        LOGln("ECG P is connected to V_MID");
    } else if (cnfg_emux.bit.calp_sel == 2) {
        LOGln("ECG P is connected to V_CALP");
    } else if (cnfg_emux.bit.calp_sel == 3) {
        LOGln("ECG P is connected to V_CALN");
    }
  
    LOGln("ECG N is %s from AFE", cnfg_emux.bit.openn ? "disconnected" : "connected");
    LOGln("ECG P is %s from AFE", cnfg_emux.bit.openp ? "disconnected" : "connected");
    LOGln("ECG input polarity is %s", cnfg_emux.bit.pol ? "inverted" : "not inverted");
  }
  
  void MAX30001G::printCNFG_ECG() {
    LOGln("MAX30001 ECG settings:");
    LOGln("----------------------");
  
    // Checking digital low pass filter settings
    if (cnfg_gen.bit.fmstr == 0) {
      if (cnfg_ecg.bit.rate == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
      } else if (cnfg_ecg.bit.rate == 1) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 40.96Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 102.4Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
          LOGln("ECG digital low pass filter is 153.6Hz, global: %.2f", ECG_lpf);
        }
      } else if (cnfg_ecg.bit.rate == 2) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 40.96Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 102.4Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
          LOGln("ECG digital low pass filter is 40.96Hz, global: %.2f", ECG_lpf);
        }
      }
    } else if (cnfg_gen.bit.fmstr == 1) {
      if (cnfg_ecg.bit.rate == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
      } else if (cnfg_ecg.bit.rate == 1) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 40.00Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 100.0Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
          LOGln("ECG digital low pass filter is 150.0Hz, global: %.2f", ECG_lpf);
        }
      } else if (cnfg_ecg.bit.rate == 2) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 27.68Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 27.68Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
          LOGln("ECG digital low pass filter is 27.68Hz, global: %.2f", ECG_lpf);
        }
      }
    } else if (cnfg_gen.bit.fmstr == 2) {
      if (cnfg_ecg.bit.rate == 2) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 40.00Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 40.00Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
          LOGln("ECG digital low pass filter is 40.00Hz, global: %.2f", ECG_lpf);
        }
      } else {
        LOGln("ECG digital low pass filter is not set, global: %.2f", ECG_lpf);
      }
    } else if (cnfg_gen.bit.fmstr == 3) {
      if (cnfg_ecg.bit.rate == 2) {
        if (cnfg_ecg.bit.dlpf == 0) {
          LOGln("ECG digital low pass filter is bypassed, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 1) {
          LOGln("ECG digital low pass filter is 39.96Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 2) {
          LOGln("ECG digital low pass filter is 39.96Hz, global: %.2f", ECG_lpf);
        } else if (cnfg_ecg.bit.dlpf == 3) {
         LOGln("ECG digital low pass filter is 39.93Hz, global: %.2f", ECG_lpf);
        }
      } else {
        LOGln("ECG digital low pass filter is not set, global: %.2f", ECG_lpf);
      }
    }
  
    // Checking digital high pass filter settings
    LOGln("ECG digital high pass filter is %s Hz, global: %.2f", cnfg_ecg.bit.dhpf ? "bypassed" : "0.5", ECG_hpf);
  
    // Checking gain settings
    LOG("ECG gain is ");
    if (cnfg_ecg.bit.gain == 0) {
      LOGln("20V/V, global %d", ECG_gain);
    } else if (cnfg_ecg.bit.gain == 1) {
      LOGln("40V/V, global %d", ECG_gain);
    } else if (cnfg_ecg.bit.gain == 2) {
      LOGln("80V/V, global %d", ECG_gain);
    } else if (cnfg_ecg.bit.gain == 3) {
      LOGln("160V/V, global %d", ECG_gain);
    }
  
    // Checking data rate settings
    LOGln("ECG data rate is ");
    if (cnfg_ecg.bit.rate == 0) {
      if (cnfg_gen.bit.fmstr == 0b00) {
        LOGln("512SPS, global %d", ECG_samplingRate);
      } else if (cnfg_gen.bit.fmstr == 0b01) {
        LOGln("500SPS, global %d", ECG_samplingRate);
      } else {
        LOGln("not defined, global %d", ECG_samplingRate);
      }
    } else if (cnfg_ecg.bit.rate == 1) {
      if (cnfg_gen.bit.fmstr == 0b00) {
        LOGln("256SPS, global %d", ECG_samplingRate);
      } else if (cnfg_gen.bit.fmstr == 0b01) {
        LOGln("250SPS, global %d", ECG_samplingRate);
      } else {
        LOGln("not defined, global %d", ECG_samplingRate);
      }
    } else if (cnfg_ecg.bit.rate == 2) {
      if (cnfg_gen.bit.fmstr == 0b00) {
        LOGln("128SPS, global %d", ECG_samplingRate);
      } else if (cnfg_gen.bit.fmstr == 0b01) {
        LOGln("125SPS, global %d", ECG_samplingRate);
      } else if (cnfg_gen.bit.fmstr == 0b10) {
        LOGln("200SPS, global %d", ECG_samplingRate);
      } else if (cnfg_gen.bit.fmstr == 0b11) {
        LOGln("199.8SPS, global %d", ECG_samplingRate);
      }
    }
  }
  
  void MAX30001G::printCNFG_BMUX() {
    LOGln("BIOZ MUX Configuration");
    LOGln("----------------------");
  
    LOGln("Calibration");
    LOG("BIOZ Calibration source frequency is ");
    if (cnfg_bmux.bit.fbist == 0) {
      LOGln("%5u", fmstr / (1 << 13));
    } else if (cnfg_bmux.bit.fbist == 1) {
      LOGln("%5u", fmstr / (1 << 15));
    } else if (cnfg_bmux.bit.fbist == 2) {
      LOGln("%5u", fmstr / (1 << 17));
    } else if (cnfg_bmux.bit.fbist == 3) {
      LOGln("%5u", fmstr / (1 << 19));
    }
  
    LOG("BIOZ nominal resistance is ");
    switch (cnfg_bmux.bit.rnom) {
      case 0:
        LOG("5000 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("2960.7 mOhm");
            break;
          case 1:
            LOGln("980.6 mOhm");
            break;
          case 2:
            LOGln("247.5 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 1:
        LOG("2500 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("740.4 mOhm");
            break;
          case 1:
            LOGln("245.2 mOhm");
            break;
          case 2:
            LOGln("61.9 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 2:
        LOG("1667 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("329.1 mOhm");
            break;
          case 1:
            LOGln("109.0 mOhm");
            break;
          case 2:
            LOGln("27.5 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 3:
        LOG("1250 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("185.1 mOhm");
            break;
          case 1:
            LOGln("61.3 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 4:
        LOG("1000 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("118.5 mOhm");
            break;
          case 1:
            LOGln("39.2 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 5:
        LOG("833 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("82.3 mOhm");
            break;
          case 1:
            LOGln("27.2 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 6:
        LOG("714 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("60.5 mOhm");
            break;
          case 1:
            LOGln("20.0 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
      case 7:
        LOG("625 Ohm with modulated resistance of ");
        switch (cnfg_bmux.bit.rmod) {
          case 0:
            LOGln("46.3 mOhm");
            break;
          case 1:
            LOGln("15.3 mOhm");
            break;
          default:
            LOGln("unmodulated");
            break;
        }
        break;
    }
  
    LOGln("Built-in self-test %s", cnfg_bmux.bit.en_bist ? "enabled, BIP/N should be open." : "disabled");
  
    LOG("BIOZ current generator for ");
    switch (cnfg_bmux.bit.cg_mode) {
      case 0:
        LOGln("unchopped sources (ECG & BIOZ application), or low current range mode");
        break;
      case 1:
        LOGln("chopped sources, BIOZ without low pass filter");
        break;
      case 2:
        LOGln("chopped sources, BIOZ with low pass filter");
        break;
      case 3:
        LOGln("chopped sources, with resistive CM setting, low impedance");
        break;
    }
  
    LOGln("Connections");
    LOG("BIOZ N is connected to ");
    switch (cnfg_bmux.bit.caln_sel) {
      case 0:
        LOGln("no calibration");
        break;
      case 1:
        LOGln("V_MID");
        break;
      case 2:
        LOGln("V_CALP");
        break;
      case 3:
        LOGln("V_CALN");
        break;
    }
    
    LOG("BIOZ P is connected to ");
    switch (cnfg_bmux.bit.calp_sel) {
      case 0:
        LOGln("no calibration");
        break;
      case 1:
        LOGln("V_MID");
        break;
      case 2:
        LOGln("V_CALP");
        break;
      case 3:
        LOGln("V_CALN");
        break;
    }
  
    LOGln("BIOZ N is %s from AFE", cnfg_bmux.bit.openn ? "disconnected" : "connected");
    LOGln("BIOZ P is %s from AFE", cnfg_bmux.bit.openp ? "disconnected" : "connected");
  }
  
  void MAX30001G::printCNFG_BIOZ() {
    LOGln("BIOZ Configuration");
    LOGln("----------------------");
  
    LOG("BIOZ phase offset is ");
    if (cnfg_bioz.bit.fcgen == 0) {
      LOGln("%f", cnfg_bioz.bit.phoff * 45.0);
    } else if (cnfg_bioz.bit.fcgen == 1) {
      LOGln("%f", cnfg_bioz.bit.phoff * 22.5);
    } else if (cnfg_bioz.bit.fcgen >= 2) {
      LOGln("%f", cnfg_bioz.bit.phoff * 11.25);
    }
  
    LOG("BIOZ current generator ");
    switch (cnfg_bioz.bit.cgmag) {
      case 0:
        LOGln("is off");
        break;
      case 1:
        LOGln("magnitude is 8uA or low current mode");
        break;
      case 2:
        LOGln("magnitude is 16uA");
        break;
      case 3:
        LOGln("magnitude is 32uA");
        break;
      case 4:
        LOGln("magnitude is 48uA");
        break;
      case 5:
        LOGln("magnitude is 64uA");
        break;
      case 6:
        LOGln("magnitude is 80uA");
        break;
      case 7:
        LOGln("magnitude is 96uA");
        break;
    }
  
    LOGln("BIOZ current generator monitor(lead off) is %s", cnfg_bioz.bit.cgmon ? "enabled" : "disabled");
  
    LOG("Modulation frequency is ");
    switch (cnfg_bioz.bit.fcgen) {
      case 0:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "131072 Hz" : cnfg_gen.bit.fmstr == 1 ? "128000 Hz" : cnfg_gen.bit.fmstr == 2 ? "128000 Hz" : "127872 Hz");
        break;
      case 1:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "81920 Hz" : cnfg_gen.bit.fmstr == 1 ? "80000 Hz" : cnfg_gen.bit.fmstr == 2 ? "80000 Hz" : "81920 Hz");
        break;
      case 2:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "40960 Hz" : cnfg_gen.bit.fmstr == 1 ? "40000 Hz" : cnfg_gen.bit.fmstr == 2 ? "40000 Hz" : "40960 Hz");
        break;
      case 3:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "18204 Hz" : cnfg_gen.bit.fmstr == 1 ? "17780 Hz" : cnfg_gen.bit.fmstr == 2 ? "17780 Hz" : "18204 Hz");
        break;
      case 4:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "8192 Hz" : cnfg_gen.bit.fmstr == 1 ? "8000 Hz" : cnfg_gen.bit.fmstr == 2 ? "8000 Hz" : "7992 Hz");
        break;
      case 5:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "4096 Hz" : cnfg_gen.bit.fmstr == 1 ? "4000 Hz" : cnfg_gen.bit.fmstr == 2 ? "4000 Hz" : "3996 Hz");
        break;
      case 6:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "2048 Hz" : cnfg_gen.bit.fmstr == 1 ? "2000 Hz" : cnfg_gen.bit.fmstr == 2 ? "2000 Hz" : "1998 Hz");
        break;
      case 7:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "1024 Hz" : cnfg_gen.bit.fmstr == 1 ? "1000 Hz" : cnfg_gen.bit.fmstr == 2 ? "1000 Hz" : "999 Hz");
        break;
      case 8:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "512 Hz" : cnfg_gen.bit.fmstr == 1 ? "500 Hz" : cnfg_gen.bit.fmstr == 2 ? "500 Hz" : "500 Hz");
        break;
      case 9:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "256 Hz" : cnfg_gen.bit.fmstr == 1 ? "250 Hz" : cnfg_gen.bit.fmstr == 2 ? "250 Hz" : "250 Hz");
        break;
      default:
        LOGln(cnfg_gen.bit.fmstr == 0 ? "128 Hz" : cnfg_gen.bit.fmstr == 1 ? "125 Hz" : cnfg_gen.bit.fmstr == 2 ? "125 Hz" : "125 Hz");
        break;
    }
  
    LOG("Low pass cutoff frequency is ");
    if (cnfg_bioz.bit.dlpf == 0) {
      LOGln("bypassed");
    } else if (cnfg_bioz.bit.dlpf == 1) {
      LOGln("4.0 Hz");
    } else if (cnfg_bioz.bit.dlpf == 2) {
      LOGln("8.0 Hz");
    } else if (cnfg_bioz.bit.dlpf == 3) {
      if (cnfg_bioz.bit.rate == 0) {
        LOGln("16.0 Hz");
      } else {
        LOGln("4.0 Hz");
      }
    }
  
    LOG("High pass cutoff frequency is ");
    switch (cnfg_bioz.bit.dhpf) {
      case 0:
        LOGln("bypassed");
        break;
      case 1:
        LOGln("0.05 Hz");
        break;
      default:
        LOGln("0.5 Hz");
        break;
    }
  
    LOG("BIOZ gain is ");
    switch (cnfg_bioz.bit.gain) {
      case 0:
        LOGln("10 V/V");
        break;
      case 1:
        LOGln("20 V/V");
        break;
      case 2:
        LOGln("40 V/V");
        break;
      case 3:
        LOGln("80 V/V");
        break;
    }
  
    LOGln("BIOZ INA is in %s mode", cnfg_bioz.bit.ln_bioz ? "low noise" : "low power");
    LOGln("BIOZ external bias resistor is %s", cnfg_bioz.bit.ext_rbias ? "enabled" : "disabled");
  
    LOG("BIOZ analog high pass cutoff frequency is ");
    switch (cnfg_bioz.bit.ahpf) {
      case 0:
        LOGln("60.0 Hz");
        break;
      case 1:
        LOGln("150 Hz");
        break;
      case 2:
        LOGln("500 Hz");
        break;
      case 3:
        LOGln("1000 Hz");
        break;
      case 4:
        LOGln("2000 Hz");
        break;
      case 5:
        LOGln("4000 Hz");
        break;
      default:
        LOGln("bypassed");
        break;
    }
  
    LOG("BIOZ data rate is ");
    if (cnfg_bioz.bit.rate == 0) {
      switch (cnfg_gen.bit.fmstr) {
        case 0:
          LOGln("64 SPS");
          break;
        case 1:
          LOGln("62.5 SPS");
          break;
        case 2:
          LOGln("50 SPS");
          break;
        default:
          LOGln("49.95 SPS");
          break;
        }
    } else {
      switch (cnfg_gen.bit.fmstr) {
        case 0:
          LOGln("32 SPS");
          break;
        case 1:
          LOGln("31.25 SPS");
          break;
        case 2:
          LOGln("25 SPS");
          break;
        default:
          LOGln("24.98 SPS");
          break;
      }
    }
  }
  
  void MAX30001G::printCNFG_BIOZ_LC() {
    LOGln("BIOZ Low Current Configuration");
    LOGln("------------------------------");
  
    LOG("BIOZ low current magnitude is ");
    if (cnfg_bioz_lc.bit.cmag_lc == 0) {
      LOGln("0nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 1) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "55nA" : "110nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 2) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "110nA" : "220nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 3) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "220nA" : "440nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 4) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "330nA" : "660nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 5) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "440nA" : "880nA");
    } else if (cnfg_bioz_lc.bit.cmag_lc == 6) {
      LOGln(cnfg_bioz_lc.bit.lc2x == 0 ? "550nA" : "1100nA");
    }
  
    LOG("BIOZ common mode feedback resistance for current generator is ");
    switch (cnfg_bioz_lc.bit.cmres) {
      case 0:
        LOGln("off");
        break;
      case 1:
        LOGln("320MOhm");
        break;
      case 3:
        LOGln("160MOhm");
        break;
      case 5:
        LOGln("100MOhm");
        break;
      case 7:
        LOGln("80MOhm");
        break;
      case 8:
        LOGln("40MOhm");
        break;
      case 9:
        LOGln("20MOhm");
        break;
      case 10:
        LOGln("12.5MOhm");
        break;
      case 11:
        LOGln("10MOhm");
        break;
      case 12:
        LOGln("7.5MOhm");
        break;
      case 13:
        LOGln("6.5MOhm");
        break;
      case 14:
        LOGln("5.5MOhm");
        break;
      case 15:
        LOGln("5.0MOhm");
        break;
    }
  
    LOGln("BIOZ high resistance is %s", cnfg_bioz_lc.bit.en_bistr ? "enabled" : "disabled");
  
    LOG("BIOZ high resistance load is ");
    switch (cnfg_bioz_lc.bit.bistr) {
      case 0:
        LOGln("27kOhm");
        break;
      case 1:
        LOGln("108kOhm");
        break;
      case 2:
        LOGln("487kOhm");
        break;
      case 3:
        LOGln("1029kOhm");
        break;
    }
  
    LOGln("BIOZ low current mode is %s", cnfg_bioz_lc.bit.lc2x ? "2x" : "1");
    LOGln("BIOZ drive current range is %s", cnfg_bioz_lc.bit.hi_lob ? "high [uA]" : "low [nA]");
  }
  
  void MAX30001G::printCNFG_RTOR1() {
    LOGln("R to R configuration");
    LOGln("--------------------");
  
    LOGln("R to R peak detection is %s", cnfg_rtor1.bit.en_rtor ? "enabled" : "disabled");
    LOGln("R to R threshold scaling factor is %d/16", cnfg_rtor1.bit.ptsf + 1);
  
    LOG("R to R peak averaging weight factor is ...");
    if (cnfg_rtor1.bit.pavg == 0) {
      LOGln("2");
    } else if (cnfg_rtor1.bit.pavg == 1) {
      LOGln("4");
    } else if (cnfg_rtor1.bit.pavg == 2) {
      LOGln("8");
    } else if (cnfg_rtor1.bit.pavg == 3) {
      LOGln("16");
    }
  
    if (cnfg_rtor1.bit.gain < 0b1111) {
      LOGln("R to R gain %d", (int)pow(2, cnfg_rtor1.bit.gain));
    } else {
      LOGln("R to R gain is auto scale");
    }
  
    LOG("R to R window length is ...");
    switch (cnfg_rtor1.bit.wndw) {
      case 0:
        LOGln("%f", 6 * RtoR_resolution);
        break;
      case 1:
        LOGln("%f", 8 * RtoR_resolution);
        break;
      case 2:
        LOGln("%f", 10 * RtoR_resolution);
        break;
      case 3:
        LOGln("%f", 12 * RtoR_resolution);
        break;
      case 4:
        LOGln("%f", 14 * RtoR_resolution);
        break;
      case 5:
        LOGln("%f", 16 * RtoR_resolution);
        break;
      case 6:
        LOGln("%f", 18 * RtoR_resolution);
        break;
      case 7:
        LOGln("%f", 20 * RtoR_resolution);
        break;
      case 8:
        LOGln("%f", 22 * RtoR_resolution);
        break;
      case 9:
        LOGln("%f", 24 * RtoR_resolution);
        break;
      case 10:
        LOGln("%f", 26 * RtoR_resolution);
        break;
      case 11:
        LOGln("%f", 28 * RtoR_resolution);
        break;
      default:
        LOGln("-1");
        break;
    }
  }
  
  void MAX30001G::printCNFG_RTOR2(){
    LOGln("R to R Configuration");
    LOGln("--------------------");
  
    if (cnfg_rtor2.bit.rhsf > 0) {
      LOGln("R to R hold off scaling is %d/8", cnfg_rtor2.bit.rhsf);
    } else {
      LOGln("R to R hold off interval is determined by minimum hold off only");
    }
  
    LOG("R to R interval averaging weight factor is ");
    switch (cnfg_rtor2.bit.ravg) {
      case 0:
        LOGln("2");
        break;
      case 1:
        LOGln("4");
        break;
      case 2:
        LOGln("8");
        break;
      case 3:
        LOGln("16");
        break;
      default:
        LOGln("Unknown");
        break;
    }
  
    LOGln("R to R minimum hold off is %.2f ms", cnfg_rtor2.bit.hoff * RtoR_resolution);
  }
  
