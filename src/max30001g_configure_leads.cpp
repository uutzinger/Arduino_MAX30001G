/******************************************************************************************************/
// Configure Leads
/******************************************************************************************************/
#include "logger.h"      // Logging 
#include "max30001g.h"

void MAX30001G::setLeadsOnDetection(bool enable) {
/*
  Configure lead on detection

  Parameters:
    - enable/disable 

  Useful when system is powered down and we want to detect when lead is attached.
  In MAX30001G, ULP lead-on detect is applied on BIP/BIN:
    BIN is pulled low with >5MOhm pulldown
    BIP is pulled high with >15MOhm pullup
  If impedance between BIP and BIN is less than ~40MOhm, LONINT is asserted.
  
  cnfg_gen.bit.en_ulp_lon
    Ultra-Low Power Lead-On Detection Enable
    00 = ULP Lead-On Detection disabled
    01 = ECG ULP Lead-On Detection enabled
    10 = Reserved. Do not use.
    11 = Reserved. Do not use.
    ULP mode is only active when the ECG channel is powered down/disabled.
  
*/

  // Read the necessary registers
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);

  // Lead-On Detection Configuration
  cnfg_gen.bit.en_ulp_lon = enable ? 0b01 : 0b00;

  if (enable && cnfg_gen.bit.en_ecg) {
    LOGW("ULP lead-on mode is enabled but only becomes active when ECG channel is disabled.");
  }

  // Write the updated configuration back to the registers
  writeRegister(MAX30001_CNFG_GEN, cnfg_gen.all);

}

void MAX30001G::setLeadsOffDetection(bool enable, bool bioz_4, uint8_t electrode_impedance) {
/*
  Configure leads off detection for the ECG or BioZ channels.

  Before running this routine
    - Make sure  that ecg or bioz is active.
    - Lead polarity is set

  This function will determine which units are active and set all appropriate leads off mechanisms.
  
  Parameters:
  - enable: enable/disable leads-off detection.
  - bioz_4: true = 4 wire BIOZ, false = 2 wire BIOZ 
  - electrode_impedance: impedance of the electrode in MOhm: 0-2, 2-4, 4-10, 10-20, >20
    used to determine current magnitude, wet electrode needs higher current to detect lead-off

  Lead off detection mechanisms:
  ================================================================================

  For ECG:
    - DC lead off detection with constant current source in ECG MUX

  For BioZ there are three lead off mechanisms:
    - DC lead off detection with constant current source, same as ECG
    - BIOZ Driver Current Monitor (4 lead only)
    - Out of range detection on digital read out (AC lead off)

  DC lead off detection
  --------------------------------------------------------------------------------
    - Enable/Disable
    - Current Polarity is either Pull Up or Pull  Down
    - If Electrode is pulled above VMID + threshold, lead off is detected
    - If Electrode is pulled below VMID - threshold, lead off is detected
    - Current magnitude 0, 5, 10, 20, 50, 100nA for dry to wet electrode range
      (wet electrode has lower resistance, needs more current to create potential)
    - Voltage threshold for lead off condition:
        VMID +/- 300 mV recommended, 
        VMID +/- 400 mV_if AVDD > 1.45, 
        VMID +/- 450 mV_if AVDD > 1.55, 
        VMID +/- 500 mV if AVDD > 1.65
    - AVDD MediBrick is 1.8V
    - VMID is lead bias voltage

    It takes 115-140ms to trigger LDOFF_XX interrupt.
    Table 1 gives recommended IMAG/VTH combinations versus electrode impedance
    and lead-bias resistance (Rb).
    Current board design:
      - 200k from VCM to GND
      - 330k from RBIAS to GND

    Impedance  in MOhm 
              Electrode I_DC for on/off detect
    ------------------------------------------
     not set      0nA    
     >0- 2 MOhm 100nA
      2- 4 MOhm  50nA
      4-10 MOhm  20nA
     10-20 MOhm  10nA
       >20 MOhm   5nA

  cnfg_gen.bit.en_dcloff
    DC Lead-Off Detection Enable
    00 = DC Lead-Off Detection disabled
    01 = DCLOFF Detection applied to the ECGP/N pins
    10 = Reserved. Do not use.
    11 = Reserved. Do not use.
    DC Method, requires active selected channel, enables DCLOFF interrupt and status bit behavior.
    Uses current sources and comparator thresholds set below.    

  cnfg_gen.bit.vth
    DC Lead-Off Voltage Threshold Selection
    0b00 = VMID ± 300mV recommended
    0b01 = VMID ± 400mV AVDD>1.45V
    0b10 = VMID ± 450mV AVDD>1.55V
    0b11 = VMID ± 500mV AVDD>1.65V

  cnfg_gen.bit.imag
    DC Lead-Off Current Magnitude Selection for pull up or down resistor
    0b000 = 0nA (Disable and Disconnect Current Sources)
    0b001 = 5nA
    0b010 = 10nA
    0b011 = 20nA
    0b100 = 50nA
    0b101 = 100nA
    0b110 = Reserved. Do not use.
    0b111 = Reserved. Do not use.
        
  cnfg_gen.bit.ipol
    DC Lead-Off Current Polarity (if current sources are enabled/connected)
      0 = ECGP - Pullup   ECGN – Pulldown
      1 = ECGP - Pulldown ECGN – Pullup

  status.bit.dcloffint
    lead off detected
  ststus.bit.ldoff_p[h/l]
    lead off detected on P lead
  status.bit.ldoff_n[h/l]
    lead off detected on N lead

  Spec Sheet Table 1
    
    IDC  10nA: All Settings of Rb, VTH = VMID +/- 300, 400 for all electrodes
    IDC  20nA: All Settings of Rb, all settings of VTH if electrode impedance < 10MOhm
    IDC  20nA: All Settings of Rb, VTH = VMID +/- 400,450,500mV if el. impedance > 10MOhm
    IDC  50nA: All Settings of Rb, all settings of VTH if el. impedance < 4MOhm
    IDC  50nA: All Settings of Rb, VTH = VMID +/- 450,500mV if el. impedance 4-10MOhm
    IDC  50nA: No settings for el. impedance > 10MOhm
    IDC 100nA: All Settings of Rb, all settings of VTH if el. impedance < 2MOhm
    IDC 100nA: All Settings of Rb, all settings VTH = VMID +/- 400,450,500mV if el. impedance 2-4 MOhm
    IDC 100nA: No settings if el. impedance >4MOhm


  BIOZ Current Monitor
  --------------------------------------------------------------------------------
  This only works for 4 wire BioZ
    cnfg_bioz.bit.cgmon
      enable/disable
    status.bit.bcgmon
      driver is in off state
    status.bit.bcgmp and bcgmn 
      indicate if N or P lead off
    en_int[1/2].bit.en_bcgmon
      lead off triggers interrupt

  BIOZ AC Lead off  
  --------------------------------------------------------------------------------
  Monitors output of analog to digital converter

  cnfg_gen.bit.en_bloff
    BioZ Digital Lead Off Detection Enable
    00 = Digital Lead Off Detection disabled
    01 = Lead Off Under Range Detection, 4 electrode BioZ applications
    10 = Lead Off Over Range Detection, 2 and 4 electrode BioZ applications
    11 = Lead Off Over & Under Range Detection, 4 electrode BioZ applications
    AC Method, requires active BioZ Channel , enables BOVER & BUNDR interrupt behavior.
    Uses BioZ excitation current set in CNFG_BIOZ with digital thresholds set in MNGR_DYN.

	  mngr_dyn.bit.bloff_hi_it
	    BioZ AC Lead Off Over-Range Threshold
	    If EN_BLOFF[1:0] = 1x and the ADC output of a BioZ measurement exceeds the
	    symmetric thresholds defined by ±2048*BLOFF_HI_IT for over 128ms, the BOVER
    interrupt bit will be asserted.
    For example, the default value (BLOFF_IT= 0xFF) corresponds to a BioZ output upper
    threshold of 0x7F800 or about 99.6% of the full scale range, and a BioZ output lower
    threshold of 0x80800 or about 0.4% of the full scale range with the LSB weight ≈ 0.4%.

	  mngr_dyn.bit.bloff_lo_it
	    BioZ AC Lead Off Under-Range Threshold
	    If EN_BLOFF[1:0] = 1x and the output of a BioZ measurement is bounded by the
	    symmetric thresholds defined by ±32*BLOFF_LO_IT for over 128ms, the BUNDR
    interrupt bit will be asserted.


  Table 7 BIOZ Lead Off Detection Schemes for AC Lead-Off and Current Monitoring
  -----------------------------------------------------------------------------

  Two Electrodes:
    cnfg_gen.bit.en_bloff = 0b10 or 0b11
    mngr_dyn.bit.bloff_hi_it = over range threshold
    measures saturated input

  Four Electrodes:
    a) 1 drive electrode off, current monitoring: 
          cnfg_bioz.bit.cgmon = 1
          measures half input
    b) 1 drive electrode off, small body coupling
          cnfg_gen.bit.en_bloff = 0b10 or 0b11
          mngr_dyn.bit.bloff_hi_it = over range threshold
          measures satuated input
    c) 1 sense electrode off
          cnfg_gen.bit.en_bloff = 0b10
          measures half input
    d) both sense electrodes off
          cnfg_gen.bit.en_bloff = 0b01 or 0b11
          mngr_dyn.bit.bloff_lo_it = under range threshold   
          measures no signal
    e) 1 sense and 1 drive electrode off
          cnfg_gen.bit.en_bloff = 0b10 or 0b11
          mngr_dyn.bit.bloff_hi_it = over range threshold
          merasures rail to rail
*/

  // Read the necessary registers
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);
  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_emux.all = readRegister24(MAX30001_CNFG_EMUX);
  mngr_dyn.all  = readRegister24(MAX30001_MNGR_DYN);

  // Determine if ECG or BioZ is active and if emux polarity is inverted
  bool ecg_active                   = cnfg_gen.bit.en_ecg;
  bool bioz_active                  = cnfg_gen.bit.en_bioz;
  bool ecg_input_polarity_inverted  = cnfg_emux.bit.pol;

  // Ensure that at least one AFE mode is enabled
  if (!ecg_active && !bioz_active) {
    LOGE("MAX30001G: No ECG or BioZ channel is active. Please enable ECG or BioZ before configuring leads off detection.");
    return;
  }

  // Reset
  // -------------------------------------
  cnfg_gen.bit.en_dcloff = 0b00;  // disabled
  cnfg_gen.bit.en_bloff  = 0b00;  // disabled
  cnfg_gen.bit.imag      = 0b000; // 0nA disconnect current source
  cnfg_bioz.bit.cgmon    = 0b0;   // disable current monitor
  cnfg_gen.bit.vth       = 0b00;   // VMID ± 300mV (default)
  // cnfg_gen.bit.ipol = ecg_polarity_inverted ? 1 : 0;  // Inverted current polarity: ECGN pullup, ECGP pulldown
  // mngr_dyn.bit.bloff_lo_it = 0xFF;  // default
  // mngr_dyn.bit.bloff_hi_it = 0xFF;  // default

  if (enable) {

    // DC LEAD OFF DETECT
    // -------------------------------------
    if ((ecg_active) || (bioz_active && !bioz_4)) {

      /*
      cnfg_gen.bit.en_dcloff = 0b01;
      cnfg_gen.bit.ipol = 0b00; // ECGP - Pullup   ECGN – Pulldown
      cnfg_gen.bit.ipol = 0b01; // ECGP - Pulldown   ECGN – Pullup
        
       >0- 2 MOhm 100nA
        2- 4 MOhm  50nA
        4-10 MOhm  20nA
       10-20 MOhm  10nA
         >20 MOhm   5nA
      
      cnfg_gen.bit.imag
          0b000 = 0nA (Disable and Disconnect Current Sources)
          0b001 = 5nA
          0b010 = 10nA
          0b011 = 20nA
          0b100 = 50nA
          0b101 = 100nA

      */

      // Enable DC Leads OFF detection0
      cnfg_gen.bit.en_dcloff = 0b01;

      // Set leads-off detection current polarity based on EMUX polarity setting.
      cnfg_gen.bit.ipol = ecg_input_polarity_inverted ? 1 : 0;

      // Adjust current and threshold based on electrode impedance
      if (       electrode_impedance <= 2) { // MOhm
        cnfg_gen.bit.imag = 0b101;   // 100nA
        if        (V_AVDD > 1650) {
          cnfg_gen.bit.vth = 0b11;   // VMID ± 500mV
        } else if (V_AVDD > 1550) {
          cnfg_gen.bit.vth = 0b10;   // VMID ± 450mV
        } else if (V_AVDD > 1450) {
          cnfg_gen.bit.vth = 0b01;   // VMID ± 400mV
        } else {
          cnfg_gen.bit.vth = 0b00;   // VMID ± 300mV (default)
        }           
      } else if (electrode_impedance <= 4) { // MOhm
        cnfg_gen.bit.imag = 0b100;   // 50nA
        if        (V_AVDD > 1550) {
          cnfg_gen.bit.vth = 0b10;   // VMID ± 450mV
        } else if (V_AVDD > 1450) {
          cnfg_gen.bit.vth = 0b01;   // VMID ± 400mV
        } else {
          cnfg_gen.bit.vth = 0b00;   // VMID ± 300mV (default)
        }           
      } else if (electrode_impedance <= 10) { // MOhm
        cnfg_gen.bit.imag = 0b011;   // 20nA
        if (V_AVDD > 1450) {
          cnfg_gen.bit.vth = 0b01;   // VMID ± 400mV
        } else {
          cnfg_gen.bit.vth = 0b00;   // VMID ± 300mV (default)
        }           
      } else if (electrode_impedance <= 20) { // MOhm
        cnfg_gen.bit.imag = 0b010;   // 10nA
        cnfg_gen.bit.vth  =  0b00;   // VMID ± 300mV
      } else if (electrode_impedance > 20) { // MOhm
        cnfg_gen.bit.imag = 0b001;   // 5nA (default for high impedance)
        cnfg_gen.bit.vth  =  0b00;   // VMID ± 300mV (default)
      }

    } // end DC LEAD OFF

    // Enable BIOZ Current Monitor Leads Off Detection
    // -----------------------------------------------
    if (bioz_active && bioz_4) {

      cnfg_bioz.bit.cgmon = 0b1;   // turn on current monitor

    } // end BIOZ Current Monitor

    // Enable BIOZ AC Leads Off Detection
    // ----------------------------------
    if (bioz_active) {

      // Leads Off AC Detection for BioZ (if BioZ is active):
      //   2 wire: only over range detection
      //   4 wire: under and over range detection,
      cnfg_gen.bit.en_bloff = bioz_4 ? 0b11 : 0b10; 

      // Threshold equations:
      //   BOVER: ±2048*BLOFF_HI_IT
      //   BUNDR:  ±32*BLOFF_LO_IT
      // Datasheet defaults are both 0xFF.
      mngr_dyn.bit.bloff_lo_it = 0xFF;
      mngr_dyn.bit.bloff_hi_it = 0xFF;

    } // end BIOZ AC lead-off
  } // end enable

  // Write the updated configuration back to the registers
  writeRegister(MAX30001_CNFG_GEN, cnfg_gen.all);
  writeRegister(MAX30001_MNGR_DYN, mngr_dyn.all);
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

}

void MAX30001G::setLeadsBias(bool enable, uint8_t resistance) {
  /* 
  
    Parameters: 

    enable:
     0 = disable, 1 = enable

    resistance: 
      0 = use external lead bias (in BIOZ mode only available for high current mode)
      >=150 internal lead bias 200M Ohm 
      >  75 internal lead bias 100M Ohm
      else  internal lead bias 50M Ohm
    
    Common voltage on subject can be maintained by an internal or external pull up or down resistor.
    We will check if ECG or BIOZ or both systems are running. 
    If both are running settings are applied to ECG channel only.
    If resistance is > 0 we will use internal resistors.
    If resistance is = 0 and ECG is in use, use 3rd ECG electrode with external resistor on VCM.
      Current board design: 200k from VCM to GND.
    If resistance is = 0 and BIOZ is in use with high current mode, use external resistor on RBIAS.
      Current board design: 330k from RBIAS to GND
      Datasheet reference uses 324k precision resistor for best current accuracy.
 
    Restrictions:
     - This routine needs to be run after ECG or BIOZ was enabled.
     - This routine needs to be set after BIOZ current was set.

    Internal:
     - Do not enable both internal BIOZ and ECG lead bias
     - In BIOZ low current mode, external resistor can not be used
  
    Registers:
  
    cnfg_gen.bit.en_rbias
      00 disabled
      01 ECG bias enabled if ECG is active
      10 BIOZ bias enabled if bioz is active
      11 not valid
    cnfg_gen.bit.rbiasv
      00 50 MOhm
      01 100 MOhm default
      10 200 MOhm
      11 not valid
    cnfg_gen.bit.rbiasp
      0 not resistively connected to VMID
      1 resistively connected to VMID
    cnfg_gen.bit.rbiasn
      0 not resistively connected to VMID
      1 resistively connected to VMID
    cnfg_bioz.bit.ext_rbias
      0 internal bias generator used
      1 external bias generator used
  
    Internal Lead Bias ECG 
    -----------------------
    cnfg_gen.bit.rbiasn   = 1
    cnfg_gen.bit.rbiasp   = 1
    cnfg_gen.bit.en_rbias = 0b01 
    cnfg_gen.bit.rbiasv   = resistance selector

    Internal Lead Bias BIOZ
    ----------------------
    cnfg_gen.bit.rbiasn   =  1
    cnfg_gen.bit.rbiasp   =  1
    cnfg_gen.bit.en_rbias = 0b10 
    cnfg_gen.bit.rbiasv   = resistance selector
  
    External ECG Lead Bias
    ----------------------
    External lead bias resistor is attached at the VCM node.
    Current board design uses 200kOhm from VCM to GND.
    enable external: 
      cnfg_gen.bit.rbiasn     = 0  // disconnect
      cnfg_gen.bit.rbiasp     = 0  // disconnect
      cnfg_gen.bit.en_rbias   = 00 // disabled
      cnfg_gen.bit.rbiasv     = 01 // not relevant
      cnfg_bioz.bit.ext_rbias = 0  // disabled external current generator
      ensure external VCM bias network is populated
  
    External BIOZ Lead Bias
    -----------------------
    330kOhm external resistor needs to be attached from RBIAS to GND
    Can be used in high current mode only
    enable external:
      cnfg_gen.bit.rbiasn     = 0
      cnfg_gen.bit.rbiasp     = 0
      cnfg_gen.bit.en_rbias   = 0b00 // disabled
      cnfg_gen.bit.rbiasv     = 0b01
      cnfg_bioz.bit.ext_rbias = 1  // enable external current generator
  
  */

    bool enableECG = false;
    bool enableBIOZ = false;
  
    cnfg_gen.all     = readRegister24(MAX30001_CNFG_GEN);
    cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
    cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC); // to check if we are in high current mode
  
    // Determine if ECG or BioZ is active
    bool ecg_active  = cnfg_gen.bit.en_ecg;
    bool bioz_active = cnfg_gen.bit.en_bioz;
  
    // Reset lead bias settings to disable
    cnfg_gen.bit.rbiasn   = 0;    // n not connected
    cnfg_gen.bit.rbiasp   = 0;    // p not connected
    cnfg_gen.bit.rbiasv   = 0b01; // default 100M Ohm
    cnfg_gen.bit.en_rbias = 0b00; // disable
    cnfg_bioz.bit.ext_rbias = 0;  // disable external current generator

    RBIASV_res = 0;
    BIOZ_cmres = 0;
  
    // Ensure that the AFE is active before applying lead bias
    if (!ecg_active && !bioz_active) {
	      LOGE("ECG nor BIOZ AFE are active. Please enable ECG or BIOZ before applying lead bias.");
        enableECG = false; // Prevent lead bias from being applied
        enableBIOZ = false; // Prevent lead bias from being applied
      }
  
    if (bioz_active && enable) {
      LOGI("BIOZ is active.");
      enableBIOZ = true;
    }

    if (ecg_active && enable) {
      LOGI("ECG is active.");
      enableECG = true;
    }

    // Ensure only one of ECG or BIOZ lead bias is enabled
    if (enableECG && enableBIOZ) {
        LOGE("ECG and BIOZ active, setting ECG bias only.");
        enableECG = true;
        enableBIOZ = false;
    }
  
    if (enableECG) {
      if (resistance > 0) {
        cnfg_gen.bit.rbiasn   = 1;
        cnfg_gen.bit.rbiasp   = 1;
        cnfg_gen.bit.en_rbias = 0b01; // ECG 
        cnfg_bioz.bit.ext_rbias = 0;  // disable external BIOZ
        if (resistance >= 150) {
          cnfg_gen.bit.rbiasv = 0b10; // 200MOhm
          RBIASV_res = 200;
        } else if (resistance >  75) {
          cnfg_gen.bit.rbiasv = 0b01; // 100MOhm
          RBIASV_res = 100;
        } else { 
          cnfg_gen.bit.rbiasv = 0b00; //  50MOhm
          RBIASV_res = 50;
        }
        LOGI("Internal ECG common mode current feedback resistor %3u[MΩ] enabled.", RBIASV_res);
      } else {
        // We want to use external 3rd electrode
        cnfg_gen.bit.rbiasn   = 0;
        cnfg_gen.bit.rbiasp   = 0;
        cnfg_gen.bit.en_rbias = 0b00; // disable internal resistors
        cnfg_bioz.bit.ext_rbias = 0;  // disable external BIOZ
        LOGI("Connect 3rd electrode on ECG channel to electrically bias subject!");
      }
    } // enable ECG bias
  
    if (enableBIOZ) {
      if (resistance > 0) {
        // We want internal resistor
        cnfg_gen.bit.rbiasn   = 1;
        cnfg_gen.bit.rbiasp   = 1;
        cnfg_gen.bit.en_rbias = 0b10; // BIOZ
        cnfg_bioz.bit.ext_rbias = 0;  // disable external BIOZ
        if (resistance >= 150) {
          cnfg_gen.bit.rbiasv = 0b10; // 200MOhm
          RBIASV_res = 200;
        } else if (resistance >  75) {
          cnfg_gen.bit.rbiasv = 0b01; // 100MOhm
          RBIASV_res = 100;
        } else { 
          cnfg_gen.bit.rbiasv = 0b00; //  50MOhm
          RBIASV_res = 50;
        }   
        LOGI("Internal BioZ common mode current feedback resistor %3u[MΩ] enabled.", RBIASV_res);
      } else {
        if (cnfg_bioz_lc.bit.hi_lob == 1){ // check for high current
          cnfg_gen.bit.rbiasn     = 0;    // disconnect
          cnfg_gen.bit.rbiasp     = 0;    // disconnect
          cnfg_gen.bit.en_rbias   = 0b00; // disabled
          cnfg_bioz.bit.ext_rbias = 1;    // enable external current generator
          BIOZ_cmres = 330; // external resistor
          LOGI("Using external BioZ common mode current feedback resistor %5lu[kΩ] for high current mode.",
               (unsigned long)BIOZ_cmres);
        } else { // not in high current, use internal 200M Ohm resistor
          cnfg_gen.bit.rbiasn   = 1;
          cnfg_gen.bit.rbiasp   = 1;
          cnfg_gen.bit.en_rbias = 0b10; // bioz
          cnfg_gen.bit.rbiasv   = 0b10; // 200 MOhm
          RBIASV_res = 200;
          cnfg_bioz.bit.ext_rbias = 0;  // disable external BIOZ
          LOGI("Not in high current mode, using internal BioZ common mode current feedback resistor %3u[MΩ] instead.", RBIASV_res);
        }
      }
    } // enable BIOZ bias
     
    LOGD("Final cnfg_gen: 0x%06X, cnfg_bioz: 0x%06X", cnfg_gen.all, cnfg_bioz.all);

    writeRegister(MAX30001_CNFG_GEN, cnfg_gen.all);
    writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  
  }
  
  
