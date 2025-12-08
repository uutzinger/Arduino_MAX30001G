/******************************************************************************************************/
// MAX30001G ECG and BIOZ Driver
/******************************************************************************************************/
// Driver for the MAX30001G Bio Potential and Impedance Front End by Maxim Integrated (now Analog Devices)                                                
//
// This driver attempts to be "complete" and offers access to all features of the MAX30001G           
// Urs Utzinger, July 2024, Spring 2025                                                             
/******************************************************************************************************/

#include "logger.h"
#include "max30001g.h"

/******************************************************************************************************/
// Hardware Description
/******************************************************************************************************/
/* 

  ECG Input Stage 
  ---------------
  MUX connects ecg pads or calibration signals to AFE
    EMI and ESD protection is built in
    Calibration signal generator
    Leads on/off detection
  Instrumentation Amplifier
  Anti aliasing filter
  Programmable Gain Amplifier
  18 bit Sigma Delta ADC

  ESD protection
    +/- 9kV contact discharge
    +/- 15kV air discharge

  DC Lead Off detection
    ECG channels need to be powered on for lead detection
    Uses programmable current sources to detect lead off conditions. 
    0,5,10,20,50,100nA are selectable
    if voltage on lead is above or below threshold, lead on/off is detected 
    VMID +/- 300 (default), 400, 450, 500 mV options
    Lead on/off can trigger interrupt

  Lead On check (ultra low power)
    Channels need to be powered off !!
    ECGN is pulled high and ECGP is pulled low with pull up/down resistor, comparator checks if both electrodes are attached

    currently not implemented as it would require to wait for lead-on event and then to startup the system

  Polarity
    ECGN and ECGP polarity can be switched internally

  Lead Bias
    Internal or external lead bias can be enabled 500, 100, 200 MOhm to meet common mode range requirements
    MediBrick has external bias resistor on VCM and can used 3rd electrode for bias

  Calibration Generator
    +/- 0.25, 0.5 mV uni or bi polar, 1/64 to 256 Hz 
    pulsewidth 0.03ms to 62ms or 50% duty cycle

  Amplifier
    MediBrick external filter for differential DC rejection is 10 microF resulting in a corner frequency of 0.05Hz 
    allowing best ECG quality but has higher susceptibility to motion artifacts
    20,40,80,160V/V over all gain selectable

  Filters
    ...

  We have external RC on ECG N and ECG P 47nF and 51k.
  We have external RC on VCM 47nF and 51k
  
  We might want to exchange external resistor on VCM to 200k
  
  We have external capacitor on CAPN/P of 10uF, therfore our high pass filter is 0.05Hz

  BIOZ Hardware
  -------------

NEED TO COMPLETE BIOZ DESCRIPTION HERE

*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

// Create the AFE device ******************************************************************************/

MAX30001G::MAX30001G(int csPin): ECG_data(128), BIOZ_data(128), _csPin(csPin) { 

  // assign CS pin
  LOGln("AFE: Pins CS, %u", _csPin);
  pinMode(_csPin,  OUTPUT);        // chip select
  LOGln("AFE: CS pin High");
  digitalWrite(_csPin, HIGH); 

  // SPI 
  LOGln("AFE: SPI start");
  SPI.begin();
  
  LOG("AFE: Checking communication: ");
  if (spiCheck()) { 
    LOGln("passed"); 
  } else { 
    LOGln("failed"); 
    return();
  }

  LOGln("AFE Reset");
  swReset();

  // SetFMSTR
  // 
  // 0 0b00 = 32768 Hz    = FLCK
  // 1 0b01 = 32000 Hz    = FLCK*625/640
  // 2 0b10 = 32000 Hz    = FLCK*625/640
  // 3 0b11 = 31968.78 Hz = FLCK*640/656
  //
  // this also updates global variables that depend on the FMSTR
  LOGln("AFE: Setting the clock");
  setFMSTR(0);

  LOGln("AFE: Reading all registers");
  readAllRegisters(); // read all registers
  dumpRegisters();    // reporting register content

}

/******************************************************************************************************/
// Setting Up the AFE
/******************************************************************************************************/
// setupECG
// setupECGcalibration
// setupBIOZ
// setupBIOZCalibrationInternal
// setupBIOZCalibrationExternal
// scanBIOZ
// setupECGandBIOZ
/******************************************************************************************************/

void MAX30001G::setupECG(uint8_t speed, uint8_t gain, bool threeleads){
/******************************************************************************************************/
/*
 * Initialize AFE for ECG and RtoR detection
 * speed 
 *   0 ~125 sps
 *   1 ~256 sps
 *   2 ~512 sps
 * gain 
 *   0  20 V/V
 *   1  40 V/V
 *   2  80 V/V
 *   3 160 V/V
 * three leads
 *  true  3 lead ECG (with ground on RL or LL)
 *  false 2 lead ECG (with RA and LA only), should use internal leads bias
 */
/******************************************************************************************************/
  
  swReset(); // Reset the AFE

  // ECG sampling rate
  //   0 low 125sps
  //   1 medium 256sps
  //   2 high 512sps  
  setECGSamplingRate(speed);

  // ECG low pass and high pass filters
  //   digital lpf for the AFE  
  //   0 bypass ~  0 Hz
  //   1 low    ~ 40 Hz
  //   2 medium ~100 Hz
  //   3 high   ~150 Hz
  // 
  //   digital hpf for the AFE  
  //   0         bypass
  //   1         0.5 Hz 
  //
  // Fixed external analog HPF on C_HPF: 
  //    0.1 uF   5.00 Hz (best motion artifact suppression)
  //    1.0 uF   0.50 Hz
  //   10.0 uF   0.05 Hz (MediBrick, highest signal quality preference)

  switch (speed) {
    case 0:
      setECGfilter(1, 0); // 40Hz and 0.Hz
      break;
    case 1:
      setECGfilter(2, 0); // 100Hz and 0.Hz
      break;
    case 2:
      setECGfilter(3, 0); // 150Hz and 0.Hz
      break;
  } 

  // ECG gain
  //   0  20 V/V
  //   1  40 V/V
  //   2  80 V/V
  //   3 160 V/V
  setECGgain(gain);

  // ECG offset recovery 
  //   set to ~ 98% of max signal
  setECGAutoRecovery(0.98 * V_ref / gain);

  // ECG lead polarity
  //   inverted 
  //     0,false: regular
  //     1,true:  swap ECG_N and ECG_P connection to AFE
  //   open 
  //     0,false: connected to AFE
  //     1,true:  isolated, use for internal calibration
  setECGLeadPolarity(false, false);

  // Disable ECG & BIOZ calibration
  setDefaultNoCalibration();
  
  // Disable BIOZ Impedance Test
  setDefaultNoBIOZImpedanceTest();

  // R to R
  setDefaultRtoR()

  // Interrupt Clearing Behaviour
  //   Clear RR_INT on read
  //   Clear fast recovery on read
  //   Clear self clear sync pulse
  //   Clear sync pulse on every sample instant
  setDefaultInterruptClearing();

  // Trigger FIFO full interrupt after 32 ECG samples and 8 BIOZ samples (max number of values)
  setFIFOInterruptThreshold(32,8);                   

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(true,  false, false, false,   false,    false); //  for ECG
  setInterrupt2(false, false, true,  false,   true,     false); //  for R to R

  // Enable ECG and RtoR
  // --------------------
  // ecg, bioz, rtor
  enableAFE(true, false, true);

  // These functions need to run after the ECG mode is enabled
  // ---------------------------------------------------------

  // Leads bias
  //   enable/disable
  //   resistance (0,50,100,200)
  // resistance: 
  //     0 use external lead bias, (ECG 3rd electrode, in BIOZ mode only available for high current mode)
  // >=150 internal lead bias 200M Ohm 
  // >  75 internal lead bias 100M Ohm
  //  else internal lead bias  50M Ohm
  //
  // MediBrick has 200k to ground lead and 10uF to ground on VCM, 3rd electrode can be used.
  //
  if (threeleads){
    setLeadBias(true,   0); // use external leads bias
  } else {
    setLeadBias(true, 200); // use internal leads bias
  }

  // ECG lead off and lead on detection
  // enable: 
  //  - enable/disable leads-off detection.
  // bioz_4: 
  //  - true  = 4 wire BIOZ 
  //  - false = 2 wire BIOZ 
  // electrode_impedance: 
  //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
  //  - used to determine current magnitude, wet electrodes need higher current for lead on detection
  setLeadsOffDetection(true, false, 2);
  setLeadsOnDetection(false); // not used
  
  delay(100);
  synch();
  delay(100);

}

void MAX30001G::setupECGcalibration(speed, gain){
/******************************************************************************************************/
/*
 * Initialize AFE for 
 * - ECG 
 * - connect internal calibration signal
 * - no RtoR
 *
 * speed 
 *   0 ~125 sps
 *   1 ~256 sps
 *   2 ~512 sps

 * gain 
 *   0  20 V/V
 *   1  40 V/V
 *   2  80 V/V
 *   3 160 V/V
 */
/******************************************************************************************************/

  // Pre start configure
  // -------------------

  setECGSamplingRate(speed);

  switch (speed) {
    case 0:
      setECGfilter(1, 1); // 40Hz and 0.5Hz
      break;
    case 1:
      setECGfilter(2, 1); // 100Hz and 0.5Hz
      break;
    case 2:
      setECGfilter(3, 1); // 150Hz and 0.5Hz
      break;
  } 

  setECGgain(gain);

  setECGNormalRecovery(); // No recovery

  setECGLeadPolarity(false, true); // normal not inverted, open for calibration
  // 0: normal, 1: inverted
  // 0: closed to measure samples, 1: open for calibration

  // Default ECG Calibration
  setDefaultECGCalibration();


  // Disable BIOZ Impedance Test
  setDefaultNoBIOZImpedanceTest(); // disable BIOZ impedance test

  // R to R
  setDefaultNoRtoR()

  // Interrupt Clearing Behaviour
  setDefaultInterruptClearing();

  // Trigger Interrupt after max values
  setFIFOInterruptThreshold(32,8);                   

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(true,  false, false, false,   false, false);
  setInterrupt2(false, false, false, false,   false, false);

  // Enable only ECG
  // ----------------
  // ecg, bioz, rtor
  enableAFE(true,false,false);

  // These functions need to run after the ECG is enabled
  // -----------------------------------------------------

  // Leads bias
  //   enable/disable
  //   resistance (0,50,100,200)
  setLeadBias(false, 100);

  // ECG lead off and lead on detection
  // enable: 
  //  - enable/disable leads-off detection.
  // bioz_4: 
  //  - true  = 4 wire BIOZ 
  //  - false = 2 wire BIOZ 
  // electrode_impedance: 
  //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
  //  - used to determine current magnitude, wet electrodes need higher current for lead on detection
  setLeadsOffDetection(false, false, 0);
  setLeadsOnDetection(false); // not used
  
  delay(100);
  synch();
  delay(100);

}


void MAX30001G::setupBIOZ(
  uint8_t speed, uint8_t gain, 
  uint8_t ahpf, uint8_t dlpf, uint8_t dhpf, 
  uint16_t frequency, uint16_t current, float phase, 
  bool leadbias, bool leadsoffdetect, bool fourleads
) {

/******************************************************************************************************/
/*
 * Initialize AFE  for BIO Impedance Measurement 

 * speed 
 *   0 approx 30 sps (25-32)
 *   1 approx 60 sps (50-64)
 
 * gain 
 *   0  10 V/V
 *   1  20 V/V
 *   2  40 V/V
 *   3  80 V/V

 * ahpf (should be lower than modulation frequency)
 *   0    60 Hz
 *   1   150 Hz
 *   2   500 Hz
 *   3  1000 Hz
 *   4  2000 Hz
 *   5  4000 Hz
 * >=6  bypass

 * dlpf (set higher for fast changing signals, but more noise)
 *   0   0 Hz
 *   1   4 Hz **
 *   2   8 Hz
 *   3  16 Hz

 * dhpf (bypass or filter out static content to visualize cardiac induced changes)
 *   0   bypass **
 *   1   0.05Hz
 *   2   0.5Hz

 * frequency (historically data was recorded at 50KHz, any frequency from 125Hz to 128kHz is available)
 *   128,000
 *    80,000
 *    40,000 **
 *    18,000
 *     8,000
 *     4,000
 *     2,000
 *     1,000
 *       500
 *       250
 *       125

 * current in nanoAmps (8uA works with all settings, higher current is better for measuring low impedance)
 * 50..96,000 nano Amps
 *     8,000 **
 
 * phase delay between current and voltage measurement 
 *  for accurate phase and impedance reading at leas two phase settings 
 *  need to be measured, default is no phase shift, just measure real component
 * phase in degrees
 *    <80kHz  80kHz 128kHz
 *  0    0.00   0.0   0.0 **
 *  1   11.25  22.5  45.0
 *  2   22.50  45.0  90.0
 *  3   33.75  67.5 135.0
 *  4   45.00  90.0 
 *  5   56.25 112.5
 *  6   67.50 125.0
 *  7   78.75 147.5
 *  8   90.00 170.0
 *  9  101.25
 * 10  112.50
 * 11  123.75
 * 12  135.00
 * 13  146.25
 * 14  157.50
 * 15  168.75

 * leads bias
 *  enable external lead bias
 *  disable lead bias
 
 * leads on detection
 *   off
 
 * leads off detection
 *   2 leads_off_detected
 *   4 bioz with 4 wire
 * 
 * Electrode impedance in MOhm
 * <=  0  
 * <=  2, ECG & BIOZ
 * <=  4, ECG
 * <= 10, ECG & BIOZ
 * <= 20, ECG & BIOZ
 * 
 */
/******************************************************************************************************/

  swReset(); // Reset the AFE

  if (speed >1) { speed = 1; }
  setBIOZSamplingRate(speed);                       // Set BIOZ sampling rate

  if (ahpf > 6) { ahpf = 6; }
  if (dlpf > 3) { dlpf = 3; }
  if (dhpf > 2) { dhpf = 2; }
  setBIOZfilter(ahpf, dlpf, dhpf);                  // ahpf 150H

  if (gain > 3) { gain = 3; }
  setBIOZgain(gain, true);                          // Set gain to 20 V/V, enable low noise INA mode

  setBIOZmodulation(2);                             // Set BIOZ modulation type to chopped with low pass filter    
  /* 0 = Unchopped Sources with Low Pass Filter * default (higher noise, excellent 50/60Hz rejection, recommended for ECG, BioZ applications)
   * 1 = Chopped Sources without Low Pass Filter          (low noise, no 50/60Hz rejection, recommended for BioZ applications with digital LPF, possibly battery powered ECG, BioZ applications)
   * 2 = Chopped Sources with Low Pass Filter             (low noise, excellent 50/60Hz rejection)
   * 3 = Chopped Sources with Resistive CM                (Not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance) 
   * */
  setBIOZModulationFrequencybyFrequency(frequency); // Set BIOZ modulation frequency

  if (current > 96000) { current = 96000; }
  if (current < 55) { current = 55; }
  setBIOZmag(current);                              // Set Current to 8 micro Amps (8000 nano Amps, lowest option for high current, can also use 48uA)
                                                    //   also sets external BIAS resistor when high current mode is selected
  setBIOZPhaseOffsetbyPhase(phase);                 // Set the phase offset

  // Disable ECG calibration
  setDefaultNoCalibration();

  // Disable Impedance Test
  setDefaultNoBIOZImpedanceTest();

  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour

  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  setInterrupt1(false, true,  false, false, false); // eanble BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
    
  // Enable BIOZ
  // ------------
  enableAFE(false,true,false);

  // These functions need to run after the BIOZ channel is enabled
  // -------------------------------------------------------------

  // Lead bias
  if (leadbias){
    // resistance
    // 50 MOhm
    // 100 MOhm default
    // 200 MOhm
    if (current >= 8000){
      setLeadBias(true,   0); // high current BIOZ external leadbias
    } else {
      setLeadBias(true, 100); // internal lead bias
    }
  } else {
    setLeadBias(false,    0); // no lead bias
  }
  
  // Current Monitor
  // Is set in LeadsOffDetection, not needed here
  // if (fourleads){
  //  setBIOZCurrentMonitor(true); // Reflected in status bit
  // }

  // Lead off detection
  // enable: 
  //  - enable/disable leads-off detection.
  // bioz_4: 
  //  - true  = 4 wire BIOZ 
  //  - false = 2 wire BIOZ 
  // electrode_impedance: 
  //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
  //  - used to determine current magnitude, wet electrodes need higher current for lead on detection
  setLeadsOffDetection(leadsoffdetect, fourleads, 2);
  setLeadsOnDetection(false); // not used

  delay(100);
  synch();
  delay(100);
  
} // initialize driver for BIOZ

void MAX30001G::setupBIOZSignalCalibration(  
  uint8_t speed, uint8_t gain
){

/******************************************************************************************************/
/*
 * BIOZ voltage calibration test
 *  provides low frequency signal to ADC and BIOX FIFO
 * 
 * Relevant Parameters
 *   speed, gain
 *   INA Noise
 *   calibration
 *   interrupts
 * Not relevant Parameters
 *   BIOZ frequency, phase and current not relevant
 *   BIOZ analog highpass and digital lowpass and digital highpass not relevant
 *   Leads bias
 *   Leads on/off detection
 * 
 */
/******************************************************************************************************/

  swReset(); // Reset the AFE

  if (speed >1) { speed = 1; }
  setBIOZSamplingRate(speed);                       // Set BIOZ sampling rate

  if (gain > 3) { gain = 3; }
  setBIOZgain(gain, true);                          // Set gain to 20 V/V, enable low noise INA mode

  // None relevant parameters
  uint8_t ahpf = 0; // lowest value (60Hz)
  uint8_t dlpf = 3; // highest value (16Hz)
  uint8_t dhpf = 0; // bypass (0Hz)
  uint16_t frequency = 40000; 
  uint16_t current = 8000;
  uint8_t phase = 0;
  setBIOZfilter(ahpf, dlpf, dhpf);                  // ahpf 150H
  setBIOZmodulation(0);                             // Set BIOZ modulation type to default
  setBIOZModulationFrequencybyFrequency(frequency); // Set BIOZ modulation frequency
  setBIOZmag(current);                              // Set Current to 8 micro Amps (8000 nano Amps, lowest option for high current, can also use 48uA)
  setBIOZPhaseOffsetbyIndex(phase);                        // Set the phase offset

  // Enable BIOZ calibration (unipolar, 0.5mV, 1Hz)
  setDefaultBIOZCalibration();

  // Disable Impedance Test
  setDefaultNoBIOZImpedanceTest();

  // Interrupts
  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour
  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(false, true,  false, false, false); // eanble BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
   
  // Enable BIOZ
  // ------------
  enableAFE(false,true,false);

  // These functions need to run after the BIOZ channel is enabled
  // -------------------------------sto------------------------------

  // Lead bias
  setLeadBias(false, 0); // no lead bias
 
  // Lead off detection
  setLeadsOffDetection(false, false, 0); // all off
  setLeadsOnDetection(false); // not used

  delay(100);
  synch();
  delay(100);
 
}

void MAX30001G::setupBIOZImpedanceCalibrationInternal(
  uint8_t speed, uint8_t gain, 
  uint8_t ahpf, uint8_t dlpf, uint8_t dhpf, 
  uint16_t frequency, uint16_t current, float phase, 
  uint32_t resistance, uint8_t modulation, uint8_t modulation_frequency
)
{
  /******************************************************************************************************/
  /*
  * BIOZ Internal Resistance Calibration
  *   provides internal modulated or constant resistor to BIOZ leads_on_detected
  *
  * speed 
  *   0 approx 30 sps (25-32)
  *   1 approx 60 sps (50-64)
  
  * gain 
  *   0  10 V/V
  *   1  20 V/V
  *   2  40 V/V
  *   3  80 V/V

  * ahpf (should be lower than modulation frequency)
  *   0    60 Hz
  *   1   150 Hz
  *   2   500 Hz
  *   3  1000 Hz
  *   4  2000 Hz
  *   5  4000 Hz
  * >=6  bypass

  * dlpf (set higher for fast changing signals, but more noise)
  *   0   0 Hz
  *   1   4 Hz **
  *   2   8 Hz
  *   3  16 Hz

  * dhpf (bypass or filter out static content to visualize fast changes such as cardiac induced changes)
  *   0   bypass **
  *   1   0.05Hz
  *   2   0.5Hz

  * frequency [Hz]
  *   128,000
  *    80,000
  *    40,000 **
  *    18,000
  *     8,000
  *     4,000
  *     2,000
  *     1,000
  *       500
  *       250
  *       125

  * current [nanoAmps] (8uA works with all settings, higher current is better for measuring low impedances)
  * 50..96,000 nano Amps
  *     8,000 **
  
  * phase delay between current and voltage measurement 
  *  phase in degrees (float)
  *  lower  0.. 11.25 ..168.75  
  *  80kHz  0.. 22.5  ..157.50  
  * 128kHz  0.. 45.0  ..135     
  *
  
  * resistance 
  * R_Nominal  Modulation Option
  *            3   2   1    0 (DC only)
  * 1029000    0   0   0    0
  *  487000    0   0   0    0
  *  108000    0   0   0    0
  *   27000    0   0   0    0
  *    5000 2961 981 248    0
  *    2500  740 245  62    0
  *    1667  329 109  28    0
  *    1250  185  61   0    0
  *    1000  119  39   0    0
  *     833   82  27   0    0
  *     714   61  20   0    0
  *     625   46  15   0    0
  * 
  * modulation frequency:
  *   4 = 4 Hz
  *   3 = 1 Hz
  *   2 = 0.25 Hz 
  *   1 = 0.0625 Hz
  *   0 = no modulation (DC)
  * 
  */
  /******************************************************************************************************/

  swReset(); // Reset the AFE

  if (speed >1) { speed = 1; }
  setBIOZSamplingRate(speed);                       // Set BIOZ sampling rate

  if (gain > 3) { gain = 3; }
  setBIOZgain(gain, true);                          // Set gain to 20 V/V, enable low noise INA mode

  if (ahpf > 6) { ahpf = 6; }
  if (dlpf > 3) { dlpf = 3; }
  if (dhpf > 2) { dhpf = 2; }
  setBIOZfilter(ahpf, dlpf, dhpf);                  // ahpf 150H

  setBIOZmodulation(1);                             // Set BIOZ modulation low noise, no low pass as internal measurements
  setBIOZModulationFrequencybyFrequency(frequency); // Set BIOZ modulation frequency

  if (current > 96000) { current = 96000; }
  if (current < 55)    { current = 55; }
  setBIOZmag(current);                              // Set Current to 8 micro Amps (8000 nano Amps, lowest option for high current, can also use 48uA)
                                                    //   also sets external BIAS resistor when high current mode is selected

  setBIOZPhaseOffsetbyPhase(uint16_t(frequency), phase);                        // Set the phase offset

  // Disable BIOZ calibration
  setDefaultNoCalibration();

  // Impedance Test
  bool useHighResistance = (resistance > 5000);
  bool enable = true;
  bool enableModulation = !useHighResistance && (modulation > 0);

  uint8_t rnomValue = 0;
  uint8_t rmodValue = 0;
  uint8_t modFreq   = 0;

 /* Resistance, Modulation (values are inverted)
  *     Ohm    3   2   1    0
  * 1029000    0   0   0    0
  *  487000    0   0   0    0
  *  108000    0   0   0    0
  *   27000    0   0   0    0
  *    5000 2961 981 248    0
  *    2500  740 245  62    0
  *    1667  329 109  28    0
  *    1250  185  61   0    0
  *    1000  119  39   0    0
  *     833   82  27   0    0
  *     714   61  20   0    0
  *     625   46  15   0    0
  */ 

  // Lookup modulation value
  auto getRmodValue = [](uint8_t rnom, uint8_t mod) -> uint8_t {
    // 0: 5.0kΩ
    // 1: 2.5kΩ
    // 2: 1.667kΩ
    // 3: 1.25kΩ
    if (mod > 3) {
      return 0;       // no modulation for high resistance
    } else {
      return 3 - mod; // shared pattern for valid rnom values
    }
  };

  // Assign rnomValue based on resistance
  if (useHighResistance) {
    if      (resistance > 1000000) rnomValue = 3; // 1029kΩ
    else if (resistance >  400000) rnomValue = 2; // 487kΩ
    else if (resistance >  100000) rnomValue = 1; // 108kΩ
    else                           rnomValue = 0; // fallback
  } else {
    if      (resistance > 2500) rnomValue = 0; // 5kΩ
    else if (resistance > 1667) rnomValue = 1; // 2.5kΩ
    else if (resistance > 1250) rnomValue = 2; // 1.667kΩ
    else if (resistance > 1000) rnomValue = 3; // 1.25kΩ
    else if (resistance >  833) rnomValue = 4; // 1kΩ
    else if (resistance >  714) rnomValue = 5; // 0.833kΩ
    else if (resistance >  625) rnomValue = 6; // 0.714kΩ
    else                        rnomValue = 7; // 0.625kΩ
  }

  // Set rmodValue if modulation is enabled
  if (enableModulation) {
    rmodValue = getRmodValue(rnomValue, modulation);
  }

  // Set modulation frequency
  switch (modulation_frequency) {
    case 1: modFreq = 3; break; // 0.0625Hz
    case 2: modFreq = 2; break; // 0.25Hz
    case 3: modFreq = 1; break; // 1Hz
    case 4: modFreq = 0; break; // 4Hz
    default: modFreq = 0; break; // 4Hz or fallback
  }

  setBIOZImpedanceTest(
    enable, useHighResistance, 
    enableModulation, 
    rnomValue, rmodValue, modFreq);

  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour

  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(false, true,  false, false, false); // eanble BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
    
  // Enable BIOZ
  // ------------
  // ecg, bioz, rtor
  enableAFE(false,true,false);

  // These functions need to run after the BIOZ channel is enabled
  // -------------------------------------------------------------

  // Lead bias
  setLeadBias(false,    0); // no lead bias
  
  // Leads off detection
  // enable: 
  //  - enable/disable leads-off detection.
  // bioz_4: 
  //  - true  = 4 wire BIOZ 
  //  - false = 2 wire BIOZ 
  // electrode_impedance: 
  //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
  //  - used to determine current magnitude, wet electrodes need higher current for lead on detection
  setLeadsOffDetection(false, false, 0);
  setLeadsOnDetection(false); // not used

  delay(100);
  synch();
  delay(100);
  
}

void MAX30001G::setupECGandBIOZ(
  uint8_t ecg_speed, uint8_t ecg_gain, bool ecg_threeleads,
  uint8_t bioz_speed, uint8_t bioz_gain, 
  uint8_t bioz_dlpf, uint8_t bioz_dhpf, 
  uint16_t bioz_frequency, uint16_t bioz_current, float bioz_phase, 
  bool leadbias, bool leadsoffdetect, bool bioz_fourleads
) {
/******************************************************************************************************/
/*
 Enable BIO Impedance and ECG measurement simultanously

  * ECG speed 
  *   0 ~125 sps
  *   1 ~256 sps
  *   2 ~512 sps
  * 
  * ECG gain 
  *   0  20 V/V
  *   1  40 V/V
  *   2  80 V/V
  *   3 160 V/V
  * 
  * ECG three leads
  *  true  3 lead ECG (with ground on RL or LL)
  *  false 2 lead ECG (with RA and LA only), will use internal leads bias
  * 
  * BIOZ speed 
  *   0 approx 30 sps (25-32)
  *   1 approx 60 sps (50-64)
  *
  * BIOZ gain 
  *   0  10 V/V
  *   1  20 V/V **
  *   2  40 V/V
  *   3  80 V/V
  *
  * BIOZ dlpf (set higher for fast changing signals, but more noise)
  *   0   0 Hz
  *   1   4 Hz **
  *   2   8 Hz
  *   3  16 Hz
  *
  * BIOZ dhpf (bypass or filter out static content to visualize cardiac induced changes)
  *   0   bypass **
  *   1   0.05Hz
  *   2   0.5Hz (**)
  *
  * BIOZ frequency (historically data was recorded at 50KHz)
  *   128,000
  *    80,000
  *    40,000 **
  *    18,000
  *     8,000 ** Proto
  *     4,000
  *     2,000
  *     1,000
  *       500 ** only with lowspeed ECG
  *       250 ** only with lowspeed ECG
  *       125 ** should not be used
  *
  * BIOZ current in nanoAmps (8uA works with all settings, higher current is better for measuring low impedance)
  * 50..96,000 nano Amps
  *     8,000 **
  *
  * BIOZ phase delay between current and voltage measurement 
  *  for accurate phase and impedance reading at leas two phase settings 
  *  need to be measured, default is no phase shift, just measure real component
  * phase in degrees (float)
  *    <80kHz  80kHz 128kHz
  *  0    0.00   0.0   0.0 **
  *  1   11.25  22.5  45.0
  *  2   22.50  45.0  90.0
  *  3   33.75  67.5 135.0
  *  4   45.00  90.0 
  *  5   56.25 112.5
  *  6   67.50 125.0
  *  7   78.75 147.5
  *  8   90.00 170.0
  *  9  101.25
  * 10  112.50
  * 11  123.75
  * 12  135.00
  * 13  146.25
  * 14  157.50
  * 15  168.75
  * 
  *
  * leads bias
  *  enable/disable
  *
  * leads on detection
  *   off
  *
  * leads off detection
  *   2 leads_off_detected
  *   4 bioz with 4 wire
  * 
  * Electrode impedance in MOhm
  * <=  0  
  * <=  2, ECG & BIOZ
  * <=  4, ECG
  * <= 10, ECG & BIOZ
  * <= 20, ECG & BIOZ
  * 
*/
/******************************************************************************************************/

  swReset(); // Reset the AFE

  setECGSamplingRate(ecg_speed);

    // ECG low pass and high pass filters
  //   digital lpf for the AFE  
  //   0 bypass ~  0 Hz
  //   1 low    ~ 40 Hz **
  //   2 medium ~100 Hz
  //   3 high   ~150 Hz
  // 
  //   digital hpf for the AFE  
  //   0         bypass
  //   1         0.5 Hz 
  //
  // Fixed external analog HPF on C_HPF: 
  //    0.1 uF   5.00 Hz (best motion artifact suppression)
  //    1.0 uF   0.50 Hz
  //   10.0 uF   0.05 Hz (MediBrick, highest signal quality preference)

  switch (ecg_speed) {
    case 0:
      setECGfilter(1, 0); // 40Hz and 0.Hz
      break;
    case 1:
      setECGfilter(2, 0); // 100Hz and 0.Hz
      break;
    case 2:
      setECGfilter(3, 0); // 150Hz and 0.Hz
      break;
  } 

  // ECG gain
  //   0  20 V/V
  //   1  40 V/V
  //   2  80 V/V
  //   3 160 V/V
  setECGgain(ecg_gain);

  // ECG offset recovery 
  //   set to ~ 98% of max signal
  setECGAutoRecovery(0.98 * V_ref / ecg_gain);

  // ECG lead polarity
  //   inverted 
  //     0,false: regular
  //     1,true:  swap ECG_N and ECG_P connection to AFE
  //   open 
  //     0,false: connected to AFE
  //     1,true:  isolated, use for internal calibration
  setECGLeadPolarity(false, false);

  // R to R
  setDefaultRtoR()

  // BIOZ
  if (bioz_speed >1) { bioz_speed = 1; }
  setBIOZSamplingRate(bioz_speed);                       // Set BIOZ sampling rate

  // AHPF based on ECG speed
  // The ECG lowpass filter needs to be lower than the BIOZ ahpf filter
  // A good separation is 4x
  /* ECG speed 
   *   0 ~125 sps, ecg low pass:  40Hz -> ahpf  150Hz: 1
   *   1 ~256 sps, ecg low pass: 100Hz -> ahpf  500Hz: 2
   *   2 ~512 sps, ecg low pass: 150Hz -> ahpf 1000Hz: 3
   * 
   *  BIOZ ahpf options are (should be lower than modulation frequency)
   *   0    60 Hz
   *   1   150 Hz
   *   2   500 Hz
   *   3  1000 Hz
   *   4  2000 Hz
   *   5  4000 Hz
   * >=6  bypass
   */

  if        ecg_speed == 0 {
    bioz_ahpf = 1; // 150Hz
  } else if ecg_speed == 1 {
    bioz_ahpf = 2; // 500Hz
  } else if ecg_speed == 2 {
    bioz_ahpf = 3; // 1000Hz
  } else {
    bioz_ahpf = 0; // 60Hz
  }

  if (bioz_dlpf > 3) { bioz_dlpf = 3; }
  if (bioz_dhpf > 2) { bioz_dhpf = 2; }
  setBIOZfilter(bioz_ahpf, bioz_dlpf, bioz_dhpf);        // ahpf 150H

  if (bioz_gain > 3) { bioz_gain = 3; }
  setBIOZgain(bioz_gain, true);                          // Set gain to 20 V/V, enable low noise INA mode

  setBIOZmodulation(2);                                  // Set BIOZ modulation type to chopped with low pass filter    
  // 0 = Unchopped Sources with Low Pass Filter * default (higher noise, excellent 50/60Hz rejection, recommended for ECG, BioZ applications)
  // 1 = Chopped Sources without Low Pass Filter          (low noise, no 50/60Hz rejection, recommended for BioZ applications with digital LPF, possibly battery powered ECG, BioZ applications)
  // 2 = Chopped Sources with Low Pass Filter             (low noise, excellent 50/60Hz rejection)
  // 3 = Chopped Sources with Resistive CM                (not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance)

  setBIOZModulationFrequencybyFrequency(bioz_frequency); // Set BIOZ modulation frequency

  if (bioz_current > 96000) { bioz_current = 96000; }
  if (bioz_current < 55) { bioz_current = 55; }
  setBIOZmag(bioz_current);                              // Set Current to 8 micro Amps (8000 nano Amps, lowest option for high current, can also use 48uA)
                                                         //   also sets external BIAS resistor when high current mode is selected
  setBIOZPhaseOffsetbyPhase(bioz_phase);                 // Set the phase offset

  // Disable ECG & BIOZ calibration
  setDefaultNoCalibration();

  // Disable BIOZ Impedance Test
  setDefaultNoBIOZImpedanceTest();
  
  // Interrupt Clearing Behaviour
  //   Clear RR_INT on read
  //   Clear fast recovery on read
  //   Clear self clear sync pulse
  //   Clear sync pulse on every sample instant
  setDefaultInterruptClearing();

  // Trigger FIFO full interrupt after 32 ECG samples and 8 BIOZ samples (max number of values)
  setFIFOInterruptThreshold(32,8);                   

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(true,  false, true,  false,   false,    false); //  for ECG
  setInterrupt2(false, true,  false, false,   true,     bioz_fourleads); //  for R to R

  // Enable BIOZ
  // ------------
  enableAFE(true,true,true);

  // These functions need to run after the BIOZ channel is enabled
  // -------------------------------------------------------------

  // Lead bias
  // When ECG and BIOZ is enabled, only ECG leadbias will be activated
  // If three leads ecg, use external thrid lead bias by setting resistance to 0
  // If two leads ecg, use internal lead bias (50, 100**, 200MOhm)
  if (leadbias) {
    if (ecg_three_leads){
        setLeadBias(true,   0); // high current BIOZ external leadbias
      } else {
        setLeadBias(true, 100); // internal lead bias
      }
  } else {
    setLeadBias(false,    0); // no lead bias
  }
  
  // Lead off detection
  // enable: 
  //  - enable/disable leads-off detection.
  // bioz_4: 
  //  - true  = 4 wire BIOZ 
  //  - false = 2 wire BIOZ 
  // electrode_impedance: 
  //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
  //  - used to determine current magnitude, wet electrodes need higher current for lead on detection
  setLeadsOffDetection(leadsoffdetect, fourleads, 2);
  setLeadsOnDetection(false); // not used

  delay(100);
  synch();
  delay(100);

} // end initialize driver for ECG and BIOZ

void MAX30001G::scanBIOZ(uint8_t avg, bool fast, bool fourleads) {
/******************************************************************************************************/
/* 
 * IMPEDANCE SPECTROSCOPY
 * ======================
 * 
 * Measures BIOZ by scanning over all valid frequencies and phase offsets:
 *
 * We measure at 8 frequencies and 16 phase offsets:
 * Frequency:  128,0000, 80,000, 40,000, 17,780, 8,000, 4,000, 2,000, 1,000, (500, 250, 125)
 * Phase:      0..11.25..168.75 degrees
 * We will not measure below 1kHz to enable sufficient 60Hz noise suppression by the ananlog high pass filter
 * 
 * After measurements are completed, magnitude and phase are extracted at each frequency
 * 
 * Result will be placed in 
 *   impedance_magnitude
 *   impedance_phase
 *   impedance_frequency
 * 
 * Magnitude and phase will be 0.0 if measurement was unsuccessful.
 * 
 * To utilize full dynamic range. measurements start a 8 micro Ampere and current is decreased 
 * if signal is saturated or increased if signal is below 10% of the ADC range.
 * Since low frequencies have a reduced current range, current adjusment is limited. 
 *
 *  BIOZ samplinng rate low (30sps)
 *  BIOZ gain 20V/V, low noise mode
 *  BIOZ ahpf 60Hz
 * 
 */  

  // high current
  // ---------------------------------------------------------------------
  // BioZ Current Generator Magnitude: 
  // cnfg_bioz.cgmag
  // 000 = Off (DRVP and DRVN floating, Current Generators Off)
  // 001 = 8µA (also use this setting when BIOZ_HI_LOB = 0)
  // 010 = 16µA
  // 011 = 32µA
  // 100 = 48µA
  // 101 = 64µA
  // 110 = 80µA
  // 111 = 96µA
  //
  // The following currents can be achied based on the selected modulation frequency: 

  // BIOZ CURRENT GENERATOR MODULATION FREQUENCY (Hz)                                     CNFG_BIOZ
  // BIOZ_FCGEN[3:0]  FMSTR[1:0] = 00 FMSTR[1:0] = 01  FMSTR[1:0] = 10  FMSTR[1:0] = 11   CGMAG allowed
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

  // low current
  // ---------------------------------------------------------------------
  //   cmag == 001 
  //   cnfg_bioz_lc.bit.hi_lob == 0 
  //   cnfg_bioz_lc.bit.lc2x == 0 55-1100nA
  //   cnfg_bioz_lc.bit.lc2x == 1 110-1100nA
  //   set the common mode resistance as recommended in BIOZ_CMRES.
  //
  // BIOZ Low Current Generator Magnitude: 
  // cnfg_bioz_lc.bit.cmag_lc
  //         LC2X = 0 LC2X = 1
  // 0000    0        0
  // 0001    55nA     110nA
  // 0010    110nA    220nA
  // 0011    220nA    440nA
  // 0100    330nA    660nA
  // 0101    440nA    880nA
  // 0110    550nA    1100nA
/******************************************************************************************************/

  float frequency[8];      // Array for modulation frequencies
  int32_t current[8];      // Array for current used at each frequency
  float impedance[8][16];  // 2D array for impedance magnitudes
  float phase[8][16];      // 2D array for phase values

  int32_t threshold_min = 524288 * 0.1; // Minimum magnitude threshold to trigger current increase
  int32_t threshold_max = 524288 * 0.9; // Maximum magnitude threshold to trigger current decrease
  int32_t adc_target    = 524288 * 0.6; // Desired ADC signal level

  uint8_t num_phase_measurements = 0; // number of phase offsets available

  if avg > 8 {
    avg = 8; // max number of samples to average
    LOGE("avg > 8, set to 8.");
  }

  // Initialize arrays
  for (int i = 0; i < 11; i++) {
    for (int j = 0; j < 16; j++) {
      impedance[i][j] = NAN; // unmeasured, set to not a finite number
      phase[i][j] = -1.0;    // unmeasured, set to invalid phase
    }
    frequency[i] = -1.0; // unmeasured, set to invalid frequency
    current[i] = 8000;   // 8 microAmpere, set to default current
  }

  swReset(); // Reset the AFE

  if (!fast) {
    setBIOZSamplingRate(0);                           // Set BIOZ sampling rate Low (30sps)
  } else {
    setBIOZSamplingRate(1);                           // Set BIOZ sampling rate high (60sps)
  }
  setBIOZgain(1, true);                             // Set gain to 20 V/V, enable low noise INA mode
  setBIOZmodulation(2);                             // Set BIOZ modulation type to chopped with low pass filter 
  // AHPF - Discussio
  // In order to measure BIOZ at 125Hz analog highpass should be set to 60Hz 
  //   however that will introduce 60Hz noise from AC main into the BIOZ signal. 
  // The default setting for the analog highpass filter is 500Hz cut on. 
  // If we choose the following cut on frequencies 60Hz suppression will be:
  //  60Hz cut on: 60Hz is supressed by -3dB  or reduced to 70%
  // 150Hz cut on: 60Hz is supressed by -10dB or reduced to 30%
  // 500Hz cut on: 60Hz is supressed by -20dB or reduced to 10%
  // Therefore to have 60Hz supression at -20dB we should not measure below 1kHz and use ahpf 500Hz
  //
  // If 60Hz noise is making it on to the BIOZ signal, after demodulation that noise will appear
  //   at 60Hz and also at modulation_frequency +/- 60Hz. The digial low pass filter will 
  //   remove that noise but only if the 60Hz noise is significanlty lower in power than 
  //   the BIOZ signal and not saturating the BIOZ channel.
  //
  // At later time we can experiement with 60Hz or 0Hz analog high pass and verify if we have same
  // readings at high modulation frequencies.
  setBIOZfilter(0, 1, 0);                           // Example: AHPF = 60Hz, LPF = 4Hz, HPF = 0. (bypass)
  setDefaultNoCalibration();                        // Disable ECG calibration
  setDefaultNoBIOZImpedanceTest();                  // Disable Impedance Test
  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour
  
  // Set Initial Configuration  
  setFIFOInteruptThreshold(1,avg);                    // trigger interrupt after avg values
  setInterrupt1(false, true,  false, false, false); // eanble BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2


  enableAFE(false,true,false);         // Enable BIOZ readings

  // Iterate over all valid frequencies 
  // 128kHz (0b0000) to 
  //   1kHz (0b0111)
  //and all phases
  // Total of 108 measurements


  for (uint8_t freq = 0; freq <= 0b0111; freq++) {
    float max_magnitude = 0.0;

    setBIOZModulationFrequencybyIndex(freq); // Set BIOZ modulation frequency
    frequency[freq] = BIOZ_frequency ;       // Update frequency, global BIOZ-frequecy was updated in previous statemet

    if freq == 0b0000 {
      num_phase_measurements = 4; // 4 offsets, 128kHz
    } else if (freq == 0b0001) {
      num_phase_measurements = 8; // 8 offsets, 80kHz
    } else if (freq == 0b0010) {
      num_phase_measurements = 16; // 16 offsets 40kHz and lower
    }

    // First Pass
    // ----------------------------------------------------------------
    // Handels saturation

    do {
      bool phaseSweepRestart = false; // Flag to indicate if we need to restart the phase sweep

      setBIOZmag(current[freq]); // Set Current
      current[freq] = BIOZ_cgmag; // Update current based on the actual set value

      // Iterate over all 16 phase offsets (0 to 15)
      for (uint8_t phase_selector = 0; phase_selector < num_phase_measurements; phase_selector++) {
        setBIOZPhaseOffset(phase_selector);  // Set the phase offset
        synch();                             // Synchronize the AFE clock
        // FIFOReset();                         // Reset the FIFO
        bioz_available = false;              // Reset BIOZ_available

        // Poll until BIOZ data is available
        // bioz_available is updated in the interrupt handler
        while (!bioz_available) {
          delay(1);  // Short delay to prevent blocking
        }
        bioz_available = false;             // Reset the flag

        // Save the measured FIFO data
        BIOZ_data.clear(); // Clear the BIOZ data ringbuffer
        readBIOZ_FIFO(true);

        // Error checking
        if (!EOF_data_detected)     {
          LOGE("Should have received End of FIFO tag.");
        }
        if (over_voltage_detected)  {
          LOGE("Over-voltage detected.");
          over_voltage_detected = false; // Reset the flag
        }
        if (under_voltage_detected) {
          LOGE("Under-voltage detected.");
          under_voltage_detected = false; // Reset the flag
        }
        if (!valid_data_detected) {
          LOGE("No valid data detected.");
          valid_data_detected = false; // Reset the flag
        }

        unit8_t num_meas = BIOZ_data.available()
        for (unit8_t n = 0; n < num_meas; n++) {
          // Read calibrated data
          float mag = BIOZ_data.pop; // Read the calibrated data from the FIFO

          if (abs(mag) > threshold_max) {
            // Stop sweep and reduce current, restart the phase sweep
            int32_t new_current = closestCurrent(current[freq] / 2);
            if new_current != current[freq] {
              current[freq] = new_current; // Update current with the new value
              phaseSweepRestart = true;
              break; // Exit phase loop to restart with adjusted current
            }
          } 

          if (abs(mag) < threshold_min) {
            // Stop sweep and reduce current, restart the phase sweep
            current[freq] = closestCurrent(current[freq] * 2);
            phaseSweepRestart = true;
            break; // Exit phase loop to restart with adjusted current
          } 

          // Store the impedance magnitude and phase
          impedance[freq][phase_selector] = mag;
          phase[freq][phase_selector] = BIOZ_phase;

          // Track maximum magnitude for this frequency
          if (abs(mag) > max_magnitude) {
            max_magnitude = abs(mag);
          }

          LOGD("Frequency: %.2f Hz, Phase: %.2f degrees, Impedance: %.1f, Current: %d", 
                frequency[freq], phase[freq][phase_selector], impedance[freq][phase_selector], current[freq]);
          } else { 
            // no valid data
            impedance[freq][phase_selector] = NAN;
            phase[freq][phase_selector] = BIOZ_phase;
            LOGE("No valid data detected.");
          }
        } // end of measurement loop
        if (phaseSweepRestart) {
          // Break out of the phase loop to restart with adjusted current
          break;
        }
      } // end of phase loop
    } while(phaseSweepRestart);

    // Second Pass
    // ----------------------------------------------------------------
    // Handels low magnitude

    if (max_magnitude == 0.0) { LOGE("Max impedance magnitude is zero."); }

    bool remeasured = false;

    // Remasure with more current if the magnitude is too low
    if ((max_magnitude < threshold_min) && (max_magnitude > 0.0)) {
      // Increase the current
      int32_t current_temp = current[freq]; // previous current

      current[freq] = closestCurrent((current[freq] * adc_target) / max_magnitude); // desired new current
      setBIOZmag(current[freq]); // Set new current
      current[freq] = BIOZ_cgmag; // Update current with what we were able to set

      if curren[freq] != current_temp {
        // There is valid new current setting, therefore remeasure
        remeasured = true;
        // Measure over all phases
        for (uint8_t phase_selector = 0; phase_selector < num_phase_measurements; phase_selector++) {
          setBIOZPhaseOffset(phase_selector);
          // enableAFE(false,true,false);

          // Reset the FIFO and synchronize the AFE clock
          FIFOReset(); // Reset the FIFO
          synch(); // Synchronize the AFE clock, might also reset FIFO

          // Poll until BIOZ data is available
          BIOZ_available = false; // there should be no value in the FIFO
          while (!BIOZ_available) {
            delay(1);  // Short delay to prevent blocking
          }
          BIOZ_available = false; // Reset the flag

          // Save the measured FIFO data
          BIOZ_data.clear(); // Clear the BIOZ data ringbuffer
          readBIOZ_FIFO(false);

          // Read all the measurements and average if multiple samples are available
          BIOZ_data.available()
          impedance[freq][phase_selector] =  // Read calibrated data
          phase[freq][phase_selector] = BIOZ_phase;

          LOGD("Frequency: %.2f Hz, Phase: %.2f degrees, Impedance: %.2f Ohm, Current: %d nA", 
                frequency[freq], phase[freq][phase_selector], impedance[freq][phase_selector], current[freq]);

          if (!EOF_data_detected)     {LOGE("Should have received End of FIFO tag.");}
          if (over_voltage_detected)  {LOGE("Over-voltage detected.");}
          if (under_voltage_detected) {LOGE("Under-voltage detected.");}
          if (!valid_data_detected)   {LOGE("No valid data detected.");}
        } // end of phase loop
      } // end of remeasure
    } // end of adjusted current measurement 

    if (remeasured == false) {
      // We did not remeasure
      // Calibrate previous readings and store the impedance magnitudes and phases
      for (uint8_t phase_selector = 0; phase_selector < num_phase_measurements; phase_selector++) {
        impedance[freq][phase_selector] = (impedance[freq][phase_selector] * V_ref * 1e9) / float(524288 * BIOZ_cgmag * BIOZ_gain);
        phase[freq][phase_selector] = BIOZ_phase;  // Phase set by setBIOZPhaseOffset
        LOGD("Frequency: %.2f Hz, Phase: %.2f degrees, Impedance: %.2f Ohm, Current: %d nA", 
              frequency[freq], phase[freq][phase_selector], impedance[freq][phase_selector], current[freq]);
      }
    }
      
  } // end of frequency loop
  
  // Fit the measurements to magnitude and phase
  // Iterate over all valid frequencies (0b0000 to 0b0111)
  for (uint8_t freq = 0; freq <= 0b0111; freq++) {

    if freq == 0b0000 {
      num_phase_measurements = 4; // 4 offsets
    } else if (freq == 0b0001) {
      num_phase_measurements = 8; // 9 offsets
    } else if (freq == 0b0010) {
      num_phase_measurements = 16; // 16 offsets
    }

    ImpedanceModel result = fitImpedance(phase[freq][0], impedance[freq][0], num_phase_measurements);
    impedance_magnitude[freq] = result.magnitude;
    impedance_phase[freq] = result.phase;
    impedance_frequency[freq] = frequency[freq];
  }

}

int32_t closestCurrent(int32_t input) {
// Returns the current matching requested current with least error
/*
 * Input current in naoamps
 */

  // The list of selectable values sorted in ascending order
  int32_t currentValues[] = {0, 55, 110, 220, 330, 440, 550, 660, 880, 1100, 8000, 16000, 32000, 48000, 64000, 80000, 96000};
  int size = sizeof(currentValues) / sizeof(currentValues[0]);

  // If the input is smaller than or equal to the smallest element, return 0
  if (input <= currentValues[0]) {
    return currentValues[0];
  }

  // Iterate over the array to find the closest smaller or equal value
  for (int i = 1; i < size; i++) {
    if (currentValues[i] >= input) {
      if ((input - currentValues[i - 1]) < (currentValues[i] - input)) {
        return currentValues[i - 1];
      } else {
        return currentValues[i];
      }
    } 
  }

  // If input is larger than the largest value, return the largest value
  return currentValues[size - 1];
}
  
ImpedanceModel fitImpedance(const float* phase_offsets, const float* measurements, int num_phase_measurements) {
/*
 * Fit the measured impedance data (measurements, phase_offsetes) to a model of the form:
 * 
 * Z = A * cos(theta) + B * sin(theta)
 * 
 * Measurements represent: V/I = |Z| cos(theta - phase_offset) and are in Ohms
 * 
 * V:             voltage measured by the AFE (BIOZ signal)
 * I:             current set for the AFE current generator
 * phase_offset:  phase offset of the demodulator compared to the current generator 
 * Z:             impedance computed
 * |Z|:           magntidue of impedance
 * theta:         phase computed
 * A:             resistance computed
 * B:             reactance comuted
 * 
 * The function returns the magnitude (|Z|) and phase (theta) of the impedance model via a last squares fitting approach.
 * 
 * This function handles all impedance measurement approaches:
 * - 2 phase offset of 0 and 90 degrees (quadrature demodulation) 
 *     [sin_sin, cos_cos terms = 1, cos_sin, sin_cos terms = 0]
 * - phase offsets at uniform intervals covering 0 to 360 degrees 
 *     [cos_sin, sin_cos terms = 0]
 * - phase offsets at arbitrary intervals
 * 
 * Urs Utzinger and ChatGPT
 */
    // Initialize sums for least squares
    float sum_cos_cos = 0.0f;
    float sum_sin_sin = 0.0f;
    float sum_cos_sin = 0.0f;
    float sum_measurements_cos = 0.0f;
    float sum_measurements_sin = 0.0f;
    int num_valid_points = 0;
    
    for (int i = 0; i < num_phase_measurements; ++i) {
        if (isfinite(measurements[i])) {
            num_valid_points++;
            float cos_theta = cos(phase_offsets[i] * PI / 180.0f);
            float sin_theta = sin(phase_offsets[i] * PI / 180.0f);
            sum_cos_cos += cos_theta * cos_theta;
            sum_sin_sin += sin_theta * sin_theta;
            sum_cos_sin += cos_theta * sin_theta;
            sum_measurements_cos += measurements[i] * cos_theta;
            sum_measurements_sin += measurements[i] * sin_theta;
        }
    }
    
    if (num_valid_points == 0) {
        return {0.0f, 0.0f};  // Return default values if no valid points
    }
    
    // Compute the determinant
    float denom = sum_cos_cos * sum_sin_sin - sum_cos_sin * sum_cos_sin;
    if (denom == 0.0f) {
        // Handle singularity (e.g., insufficient variation in phase offsets)
        return {0.0f, 0.0f};
    }
    
    // Solve for A and B
    float A = (sum_measurements_cos * sum_sin_sin - sum_measurements_sin * sum_cos_sin) / denom;
    float B = (sum_measurements_sin * sum_cos_cos - sum_measurements_cos * sum_cos_sin) / denom;
    
    // Compute magnitude and phase
    float magnitude = sqrt(A * A + B * B);
    float phase = atan2(B, A);  // Phase in radians
    
    ImpedanceModel result = {magnitude, phase};
    return result;
}

/******************************************************************************************************/
// Enable ECG, BIOZ, RtoR
/******************************************************************************************************/

void MAX30001G::enableAFE(bool enableECG, bool enableBIOZ, bool enableRtoR) {
  cnfg_gen.all   = readRegister24(MAX30001_CNFG_GEN);
  cnfg_rtor1.all = readRegister24(MAX30001_CNFG_RTOR1);

  cnfg_gen.bit.en_ecg    = enableECG ? 1 : 0;
  cnfg_gen.bit.en_bioz   = enableBIOZ ? 1 : 0;  
  cnfg_rtor1.bit.en_rtor = enableRtoR ? 1 : 0;

  if (enableRtoR && !enableECG) {
    cnfg_gen.bit.en_ecg  = 1;
    LOGW("To enable RtoR you must enable ECG. ECG enabled.");
  }
  writeRegister(MAX30001_CNFG_GEN, cnfg_gen.all);
  writeRegister(MAX30001_CNFG_RTOR1, cnfg_rtor1.all);
}

