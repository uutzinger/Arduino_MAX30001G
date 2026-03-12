/******************************************************************************************************/
// MAX30001G ECG and BIOZ Driver
/******************************************************************************************************/
// Driver for the MAX30001G Bio Potential and Impedance Front End by Maxim Integrated (now Analog Devices)                                                
//
// This driver attempts to be "complete" and offers access to all features of the MAX30001G           
// Urs Utzinger, July 2024 - present
// GPT-5.3 assisted development, review and testing, Spring 2026                                                             
/******************************************************************************************************/

#include "logger.h"
#include "max30001g.h"

#define DEBUG_MAX30001G

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

MAX30001G::MAX30001G(uint8_t csPin, int intPin1, int intPin2)
  : _csPin(csPin),
    _intPin1(intPin1),
    _intPin2(intPin2),
    _profile(PROFILE_NONE),
    _configured(false),
    _running(false),
    _useECG(false),
    _useBIOZ(false),
    _useRTOR(false),
    _drainECGOnUpdate(false),
    _drainBIOZOnUpdate(false),
    _readRTOROnUpdate(false),
    _applyLeadSettingsOnStart(false),
    _leadBiasEnable(false),
    _leadBiasResistance(0U),
    _leadOffEnable(false),
    _leadOffBioz4(false),
    _leadOffElectrodeImpedance(0U),
    _leadOnEnable(false),
    _savedConfig{},
    _scanCurrentProfileValid(false),
    _scanProfileFreqStart(0),
    _scanProfileFreqEnd(7),
    _scanProfileFast(false),
    _scanRuntimeConfig(),
    _scanReuseCurrents(false),
    _scanInProgress(false),
    _scanCompleted(false),
    _BIOZScanState(BIOZ_SCAN_IDLE),
    _scanFreqIndex(0U),
    _scanPhaseIndex(0U),
    _scanAttempt(0U),
    _scanNumPhaseMeasurements(0U),
    _scanWaitDeadlineMs(0U),
    _scanThresholdMin(0.0f),
    _scanThresholdMax(0.0f),
    _scanAdcTarget(0.0f),
    _scanSawValid(false),
    _scanSawInvalidOrRange(false),
    _scanAnyAboveTarget(false),
    _scanAnyInTarget(false),
    _scanMaxAbsRaw(0.0f) {

  // assign CS pin
  LOGI("AFE: CS pin = %u", _csPin);
  pinMode(_csPin, OUTPUT); // chip select
  digitalWrite(_csPin, HIGH);

  // SPI
  SPI.begin();

  LOGI("AFE: Checking communication");
  if (!spiCheck()) {
    LOGE("AFE: SPI communication check failed.");
    return;
  }
  LOGI("AFE: SPI communication check passed.");

  LOGI("AFE: Reset");
  swReset();

  // Set FMSTR
  //
  // 0 0b00 = 32768 Hz    = FCLK
  // 1 0b01 = 32000 Hz    = FCLK*625/640
  // 2 0b10 = 32000 Hz    = FCLK*625/640
  // 3 0b11 = 31968.78 Hz = FCLK*640/656
  //
  // This also updates global variables that depend on FMSTR.
  setFMSTR(0);

  readAllRegisters(); // read all registers

  #ifdef DEBUG_MAX30001G
    printAllRegisters(); // print all registers to log
  #endif

  for (uint8_t i = 0U; i < MAX30001_BIOZ_NUM_FREQUENCIES; ++i) {
    _scanCurrentProfile[i] = 8000;
    _scanCurrent[i] = 8000;
    _scanFrequency[i] = 0.0f;
    for (uint8_t j = 0U; j < MAX30001_BIOZ_NUM_PHASES; ++j) {
      _scanImpedance[i][j] = NAN;
      _scanPhaseDeg[i][j] = NAN;
    }
  }
  _savedConfig.valid = false;

  if (_intPin1 >= 0) {
    pinMode(_intPin1, INPUT_PULLUP);
    const int irq1 = digitalPinToInterrupt(_intPin1);
    if (irq1 != NOT_AN_INTERRUPT) {
      attachInterrupt(irq1, onAFE_IRQ1, FALLING);
      LOGI("AFE: INTB attached on pin %d", _intPin1);
    } else {
      LOGW("AFE: pin %d does not support external interrupts; INTB not attached.", _intPin1);
    }
  }

  if ((_intPin2 >= 0) && (_intPin2 != _intPin1)) {
    pinMode(_intPin2, INPUT_PULLUP);
    const int irq2 = digitalPinToInterrupt(_intPin2);
    if (irq2 != NOT_AN_INTERRUPT) {
      attachInterrupt(irq2, onAFE_IRQ2, FALLING);
      LOGI("AFE: INT2B attached on pin %d", _intPin2);
    } else {
      LOGW("AFE: pin %d does not support external interrupts; INT2B not attached.", _intPin2);
    }
  } else if ((_intPin2 >= 0) && (_intPin2 == _intPin1)) {
    LOGW("AFE: INT2B pin equals INTB pin (%d); INT2B attachment skipped.", _intPin2);
  }

}

void MAX30001G::onAFE_IRQ1() {
  afe_irq_pending = true;
  afe_irq1_pending = true;  // ISR only sets a flag
}

void MAX30001G::onAFE_IRQ2() {
  afe_irq_pending = true;
  afe_irq2_pending = true;  // ISR only sets a flag
}

void MAX30001G::begin(){
  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  clearLatchedStatusFlags();
  valid_data_detected = false;
  EOF_detected = false;
  rtor_counter = 0;
  _running = false;

  if (!spiCheck()) {
    LOGE("AFE: begin() failed SPI self-check.");
  }

  readAllRegisters();
  refreshTimingGlobals();
}

void MAX30001G::start() {
  if (!_configured) {
    LOGW("AFE: start() ignored because no setup profile has been configured.");
    return;
  }

  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  clearLatchedStatusFlags();
  valid_data_detected = false;
  EOF_detected = false;
  rtor_counter = 0;
  _scanCompleted = false;

  if (_profile == PROFILE_BIOZ_SCAN) {
    startBIOZScan();
    return;
  }

  enableAFE(_useECG, _useBIOZ, _useRTOR);

  if (_applyLeadSettingsOnStart) {
    setLeadsBias(_leadBiasEnable, _leadBiasResistance);
    setLeadsOffDetection(_leadOffEnable, _leadOffBioz4, _leadOffElectrodeImpedance);
    setLeadsOnDetection(_leadOnEnable);
  }

  FIFOReset();
  delay(100);
  synch();
  delay(100);
  _running = true;
}

void MAX30001G::stop() {
  if (_profile == PROFILE_BIOZ_SCAN) {
    stopBIOZScan();
  }

  enableAFE(false, false, false);

  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  clearLatchedStatusFlags();
  valid_data_detected = false;
  EOF_detected = false;
  rtor_counter = 0;
  _running = false;
}

bool MAX30001G::update(bool reportRaw) {
  if (!_running) {
    return false;
  }

  if (_profile == PROFILE_BIOZ_SCAN) {
    const BiozScanState previousState = _BIOZScanState;
    const bool wasCompleted = _scanCompleted;
    stepBIOZScan();
    return (previousState != _BIOZScanState) || (!wasCompleted && _scanCompleted);
  }

  bool handled = false;
  bool serviced = servicePendingInterrupts();

  // Poll STATUS when no hardware interrupt pins are wired.
  if (!serviced && (_intPin1 < 0) && (_intPin2 < 0)) {
    serviceAllInterrupts();
    serviced = true;
  }
  handled = serviced;

  if (_drainECGOnUpdate && ecg_available) {
    readECG_FIFO(reportRaw);
    ecg_available = false;
    handled = true;
  }

  if (_drainBIOZOnUpdate && bioz_available) {
    readBIOZ_FIFO(reportRaw);
    bioz_available = false;
    handled = true;
  }

  if (_readRTOROnUpdate && rtor_available) {
    readRTOR();
    handled = true;
  }

  if (ecg_overflow_occurred || bioz_overflow_occurred) {
    FIFOReset();
    ecg_overflow_occurred = false;
    bioz_overflow_occurred = false;
    handled = true;
  }

  return handled;
}

void MAX30001G::end(){
  stop();
  setInterrupt1(false, false, false, false, false, false);
  setInterrupt2(false, false, false, false, false, false);
  clearAllInterruptCallbacks();

  if (_intPin1 >= 0) {
    const int irq1 = digitalPinToInterrupt(_intPin1);
    if (irq1 != NOT_AN_INTERRUPT) {
      detachInterrupt(irq1);
    }
  }
  if ((_intPin2 >= 0) && (_intPin2 != _intPin1)) {
    const int irq2 = digitalPinToInterrupt(_intPin2);
    if (irq2 != NOT_AN_INTERRUPT) {
      detachInterrupt(irq2);
    }
  }
}

/******************************************************************************************************/
// Setting Up the AFE with the following utility functions:
/******************************************************************************************************/
// setupECG
// setupBIOZ
// setupECGandBIOZ
// setupECGSignalCalibration
// setupBIOZSignalCalibration
// setupBIOZImpedanceCalibration
// setupBIOZExternalImpedanceCalibration
// scanBIOZ
/******************************************CalibrationEx************************************************************/

/******************************************************************************************************/
// ECG
/******************************************************************************************************/

void MAX30001G::setupECG(uint8_t speed, uint8_t gain, bool threeleads) {
/******************************************************************************************************/
/*
 * Initialize AFE for ECG and RtoR detection
 * speed 
 *   0 ~125 sps
 *   1 ~256 sps * default
 *   2 ~512 sps
 * gain 
 *   0  20 V/V
 *   1  40 V/V
 *   2  80 V/V * default
 *   3 160 V/V
 * three leads
 *  true  3 lead ECG (with ground on RL or LL) * default
 *  false 2 lead ECG (with RA and LA only), should use internal leads bias
 *
 * ECG low pass and high pass filters
 *   digital lpf for the AFE  
 *   0 bypass ~  0 Hz * for speed 0
 *   1 low    ~ 40 Hz * for speed 1
 *   2 medium ~100 Hz * for speed 2
 *   3 high   ~150 Hz - not used
 * 
 *   digital hpf for the AFE  
 *   0         bypass * default for all speeds
 *   1         0.5 Hz 
 *
 * fixed external analog HPF on C_HPF: 
 *    0.1 uF   5.00 Hz (best motion artifact suppression)
 *    1.0 uF   0.50 Hz
 *   10.0 uF   0.05 Hz (MediBrick, highest signal quality preference)
 *
 * polarity to regular and connected to AFE
 * no calibration
 * no BIOZ impedance test
 * enables RtoR
 * enable ecgFifo interrupt
 * enable lead off detection with 2MOhm threshold
 * no lead on detection
 * lead bias 0 if three leads operation otherwise 200MOhm internal
 */
/******************************************************************************************************/

  // Lifecycle profile: ECG + RTOR with FIFO drain in update().
  _profile = PROFILE_ECG;
  _configured = true;
  _running = false;
  _useECG = true;
  _useBIOZ = false;
  _useRTOR = true;
  _drainECGOnUpdate = true;
  _drainBIOZOnUpdate = false;
  _readRTOROnUpdate = true;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = true;
  _leadBiasResistance = threeleads ? 0U : 200U;
  _leadOffEnable = true;
  _leadOffBioz4 = false;
  _leadOffElectrodeImpedance = 2U;
  _leadOnEnable = false;

  swReset(); // Reset the AFE to a known baseline.

  // Ensure global timing variables are refreshed after reset.
  setFMSTR(0);

  setECGSamplingRate(speed);

  // ECG low pass and high pass filters
  switch (speed) {
    case 0:
      setECGfilter(1, 0); // 40Hz and 0Hz
      break;
    case 1:
      setECGfilter(2, 0); // 100Hz and 0Hz
      break;
    case 2:
      setECGfilter(3, 0); // 150Hz and 0Hz
      break;
    default:
      setECGfilter(3, 0); // safe default
      break;
  }

  setECGgain(gain);

  // ECG offset recovery 
  //   Set threshold to ~98% of max ECG input range.
  //   setECGAutoRecovery expects threshold in mV.
  if (ECG_gain > 0) {
    const int recovery_threshold_mV = static_cast<int>((0.98f * V_ref) / static_cast<float>(ECG_gain));
    setECGAutoRecovery(recovery_threshold_mV);
  } else {
    setECGNormalRecovery();
    LOGW("ECG gain not initialized correctly; fast auto-recovery disabled.");
  }

  // ECG lead polarity
  //   inverted 
  //     0,false: regular
  //     1,true:  swap ECG_N and ECG_P connection to AFE
  //   open 
  //     0,false: connected to AFE
  //     1,true:  isolated, use for internal calibration
  setECGLeadPolarity(false, false);

  // Disable ECG & BIOZ test signals
  setDefaultNoTestSignal();
  
  // Disable BIOZ Impedance Test
  setDefaultNoBIOZTestImpedance();

  // R to R
  setDefaultRtoR();

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

  // Start-time actions (EN_ECG/EN_RTOR, lead bias/off, FIFO reset, SYNCH) are applied in start().
}

void MAX30001G::setupECGSignalCalibration(uint8_t speed, uint8_t gain) {
/******************************************************************************************************/
/*
 * Initialize AFE for 
 * - ECG 
 * - connect internal calibration signal
 * - no RtoR
 *
 * Uses default setup values:
 * speed
 * 0 low    ~125sps
 * 1 medium ~256sps
 * 2 high   ~512sps * default
 * 
 * gain
 *   0  20 V/V
 *   1  40 V/V
 *   2  80 V/V
 *   3 160 V/V
 */
/******************************************************************************************************/

  _profile = PROFILE_ECG_CAL;
  _configured = true;
  _running = false;
  _useECG = true;
  _useBIOZ = false;
  _useRTOR = false;
  _drainECGOnUpdate = true;
  _drainBIOZOnUpdate = false;
  _readRTOROnUpdate = false;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = false;
  _leadBiasResistance = 100U;
  _leadOffEnable = false;
  _leadOffBioz4 = false;
  _leadOffElectrodeImpedance = 0U;
  _leadOnEnable = false;

  swReset();   // Reset the AFE to a known baseline.
  setFMSTR(0); // Ensure global timing variables are refreshed after reset.
  
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
    default:
      setECGfilter(3, 1);
      break;
  }

  setECGgain(gain);

  setECGNormalRecovery(); // No recovery

  setECGLeadPolarity(false, true); // normal not inverted, open for calibration
  // 0: normal, 1: inverted
  // 0: closed to measure samples, 1: open for calibration

  // Default ECG Voltage Calibration
  setDefaultECGTestSignal();

  // Disable BIOZ Impedance Test
  setDefaultNoBIOZTestImpedance(); // disable BIOZ impedance test

  // R to R
  setDefaultNoRtoR();

  // Interrupt Clearing Behaviour
  setDefaultInterruptClearing();

  // Trigger Interrupt after max values
  setFIFOInterruptThreshold(32,8);                   

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(true,  false, false, false,   false, false);
  setInterrupt2(false, false, false, false,   false, false);

  // Start-time actions (EN_ECG, lead settings, FIFO reset, SYNCH) are applied in start().
}

/******************************************************************************************************/
// BIOZ
/******************************************************************************************************/

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

 * ahpf (should be lower than selected modulation frequency)
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
 * 55..96,000 nano Amps
 *     8,000 **
 
 * phase delay between current and voltage measurement
 *  for accurate phase and impedance reading at least two phase settings
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

  _profile = PROFILE_BIOZ;
  _configured = true;
  _running = false;
  _useECG = false;
  _useBIOZ = true;
  _useRTOR = false;
  _drainECGOnUpdate = false;
  _drainBIOZOnUpdate = true;
  _readRTOROnUpdate = false;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = leadbias;
  if (leadbias) {
    _leadBiasResistance = (current >= 8000U) ? 0U : 100U;
  } else {
    _leadBiasResistance = 0U;
  }
  _leadOffEnable = leadsoffdetect;
  _leadOffBioz4 = fourleads;
  _leadOffElectrodeImpedance = 2U;
  _leadOnEnable = false;

  swReset(); // Reset the AFE
  setFMSTR(0); // Keep setup scripts on a known timing base and refresh dependent globals.

  if (speed > 1) { speed = 1; }
  if (gain > 3) { gain = 3; }
  if (ahpf > 6) { ahpf = 6; }
  if (dlpf > 3) { dlpf = 3; }
  if (dhpf > 2) { dhpf = 2; }
  if (current > 96000) { current = 96000; }
  if (current < 55) { current = 55; }

  setBIOZSamplingRate(speed); // Set BIOZ sampling rate
  setBIOZgain(gain, true);    // Set gain and enable low-noise INA mode

  // 0 = Unchopped sources with LPF (required for low-current mode)
  // 1 = Chopped sources without LPF
  // 2 = Chopped sources with LPF
  // 3 = Chopped sources with resistive CM (not for drive currents >32uA)
  const uint8_t modulation_mode = (current < 8000U) ? 0U : 2U;
  setBIOZmodulation(modulation_mode);
  setBIOZModulationFrequencyByFrequency(frequency); // Set BIOZ modulation frequency first.

  // Ensure AHPF corner is below actual selected modulation frequency.
  // When requested AHPF is too high, clamp to the highest valid code below fmod.
  if (ahpf <= 5U) {
    static const float ahpf_hz[6] = {60.0f, 150.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
    const float fmod_hz = BIOZ_frequency;

    uint8_t max_valid_ahpf = 6U; // no valid non-bypass option yet
    for (uint8_t i = 0; i < 6U; ++i) {
      if (ahpf_hz[i] < fmod_hz) {
        max_valid_ahpf = i;
      }
    }

    if (max_valid_ahpf == 6U) {
      LOGW("No AHPF corner is below %.0f Hz modulation; forcing AHPF bypass.", fmod_hz);
      ahpf = 6U;
    } else if (ahpf > max_valid_ahpf) {
      LOGW("Requested AHPF %.0f Hz is not below %.0f Hz modulation; using %.0f Hz.",
           ahpf_hz[ahpf], fmod_hz, ahpf_hz[max_valid_ahpf]);
      ahpf = max_valid_ahpf;
    }
  }
  setBIOZfilter(ahpf, dlpf, dhpf);

  setBIOZmag(current);              // Set drive current and required current-feedback configuration.
  setBIOZPhaseOffsetbyPhase(phase); // Set demodulation phase offset.

  // Disable ECG/BIOZ calibration
  setDefaultNoTestSignal();

  // Disable Impedance Test
  setDefaultNoBIOZTestImpedance();
  
  // Disable R-to-R when running BIOZ-only setup
  setDefaultNoRtoR();

  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour

  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  setInterrupt1(false, true,  false, false, false); // enable BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
    
  // Start-time actions (EN_BIOZ, lead settings, FIFO reset, SYNCH) are applied in start().
  
} // initialize driver for BIOZ

void MAX30001G::setupBIOZSignalCalibration(  
  uint8_t speed, 
  uint8_t gain
) {

/******************************************************************************************************/
/*
 * BIOZ VCAL signal calibration test
 *  Provides an internal low-frequency voltage test signal to the BIOZ channel and FIFO.
 *  This routine validates the BIOZ analog/ADC/FIFO signal path using VCAL.
 *  It does NOT exercise current injection + demodulation phase behavior.
 * 
 * Relevant Parameters
 *   speed, gain
 *   INA Noise
 *   calibration
 *   interrupts
 * Not relevant Parameters
 *   BIOZ frequency, phase and current-generator magnitude
 *   Lead bias
 *   Leads on/off detection
 *
 * Fixed for this setup:
 *   AHPF  = bypass (required so ~1Hz VCAL is not attenuated)
 *   DLPF  = 16Hz
 *   DHPF  = bypass
 *   CGMAG = 0 (current generator off)
 *   BMUX_CG_MODE = 0
 *
 * For demodulator/current-generator path validation use setupBIOZImpedanceCalibration().
 *
 * AFE and global timing base are reset to defaults.
 * 
 */
/******************************************************************************************************/

  _profile = PROFILE_BIOZ_CAL;
  _configured = true;
  _running = false;
  _useECG = false;
  _useBIOZ = true;
  _useRTOR = false;
  _drainECGOnUpdate = false;
  _drainBIOZOnUpdate = true;
  _readRTOROnUpdate = false;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = false;
  _leadBiasResistance = 0U;
  _leadOffEnable = false;
  _leadOffBioz4 = false;
  _leadOffElectrodeImpedance = 0U;
  _leadOnEnable = false;

  swReset(); // Reset the AFE
  setFMSTR(0); // Keep setup scripts on a known timing base and refresh dependent globals.

  if (speed > 1) { speed = 1; }
  setBIOZSamplingRate(speed);                       // Set BIOZ sampling rate

  if (gain > 3) { gain = 3; }
  setBIOZgain(gain, true);                          // Set gain to 20 V/V, enable low noise INA mode

  // VCAL path setup: keep generator path idle.
  // AHPF must be bypassed for low-frequency VCAL (default is ~1Hz).
  const uint8_t ahpf = 6; // bypass
  const uint8_t dlpf = 3; // ~16Hz
  const uint8_t dhpf = 0; // bypass
  setBIOZfilter(ahpf, dlpf, dhpf);
  setBIOZmodulation(0);   // required-safe mode when generator is idle/low-current path
  setBIOZmag(0);          // disable BIOZ current generator

  // Enable BIOZ calibration (unipolar, 0.5mV, 1Hz)
  setDefaultBIOZTestSignal();

  // Disable Impedance Test
  setDefaultNoBIOZTestImpedance();
  setDefaultNoRtoR();

  // Interrupts
  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour
  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(false, true,  false, false, false); // enable BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
   
  // Start-time actions (EN_BIOZ, lead settings, FIFO reset, SYNCH) are applied in start().
 
}

void MAX30001G::setupBIOZImpedanceCalibration(
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

  * ahpf (should be lower than selected BIOZ modulation frequency)
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
  * 55..96,000 nano Amps
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

  _profile = PROFILE_BIOZ_IMP_CAL;
  _configured = true;
  _running = false;
  _useECG = false;
  _useBIOZ = true;
  _useRTOR = false;
  _drainECGOnUpdate = false;
  _drainBIOZOnUpdate = true;
  _readRTOROnUpdate = false;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = false;
  _leadBiasResistance = 0U;
  _leadOffEnable = false;
  _leadOffBioz4 = false;
  _leadOffElectrodeImpedance = 0U;
  _leadOnEnable = false;

  swReset(); // Reset the AFE
  setFMSTR(0); // Keep setup scripts on a known timing base and refresh dependent globals.

  if (speed > 1) { speed = 1; }
  if (gain > 3) { gain = 3; }
  if (ahpf > 6) { ahpf = 6; }
  if (dlpf > 3) { dlpf = 3; }
  if (dhpf > 2) { dhpf = 2; }
  if (current > 96000) { current = 96000; }
  if (current < 55)    { current = 55; }
  if (modulation > 3) { modulation = 3; }
  if (modulation_frequency > 4) { modulation_frequency = 4; }

  setBIOZSamplingRate(speed); // Set BIOZ sampling rate
  setBIOZgain(gain, true);    // Set gain and enable low-noise INA mode

  setBIOZModulationFrequencyByFrequency(frequency); // Set BIOZ modulation frequency first.
  setBIOZmag(current); // Set BIOZ current magnitude and current-feedback mode.

  // Low-current BIOZ (55..1100nA) requires BMUX_CG_MODE=0; use mode 1 for high-current only.
  const uint8_t bioz_modulation_mode = (current < 8000U) ? 0U : 1U;
  setBIOZmodulation(bioz_modulation_mode);

  // Ensure AHPF corner is below actual selected modulation frequency.
  if (ahpf <= 5U) {
    static const float ahpf_hz[6] = {60.0f, 150.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
    const float fmod_hz = BIOZ_frequency;

    uint8_t max_valid_ahpf = 6U; // no valid non-bypass option yet
    for (uint8_t i = 0; i < 6U; ++i) {
      if (ahpf_hz[i] < fmod_hz) {
        max_valid_ahpf = i;
      }
    }

    if (max_valid_ahpf == 6U) {
      LOGW("No AHPF corner is below %.0f Hz modulation; forcing AHPF bypass.", fmod_hz);
      ahpf = 6U;
    } else if (ahpf > max_valid_ahpf) {
      LOGW("Requested AHPF %.0f Hz is not below %.0f Hz modulation; using %.0f Hz.",
           ahpf_hz[ahpf], fmod_hz, ahpf_hz[max_valid_ahpf]);
      ahpf = max_valid_ahpf;
    }
  }
  setBIOZfilter(ahpf, dlpf, dhpf);

  setBIOZPhaseOffsetbyPhase(phase); // Set phase offset for selected BIOZ modulation frequency.

  // Disable ECG/BIOZ calibration (VCAL must be disabled for BIOZ impedance test).
  setDefaultNoTestSignal();

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
  auto getRmodValue = [](uint8_t mod) -> uint8_t {
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
    rmodValue = getRmodValue(modulation);
  }

  // Set modulation frequency
  switch (modulation_frequency) {
    case 1: modFreq = 3; break; // 0.0625Hz
    case 2: modFreq = 2; break; // 0.25Hz
    case 3: modFreq = 1; break; // 1Hz
    case 4: modFreq = 0; break; // 4Hz
    default: modFreq = 0; break; // 4Hz or fallback
  }

  setBIOZTestImpedance(
    enable, useHighResistance,
    enableModulation,
    rnomValue, rmodValue, modFreq);

  setDefaultNoRtoR();

  setDefaultInterruptClearing();                    // Interrupt Clearing Behaviour

  setFIFOInterruptThreshold(32,8);                  // trigger interrupt after max values

  // Enable Interrupts
  //            ecg,   bioz,  rtor,  leadson, leadsoff, bioz_fourwire
  setInterrupt1(false, true,  false, false, false); // enable BIOZ interrupt on interrupt 1
  setInterrupt2(false, false, false, false, false); // disable interrupt 2
    
  // Start-time actions (EN_BIOZ, lead settings, FIFO reset, SYNCH) are applied in start().
  
}

void MAX30001G::setupBIOZExternalImpedanceCalibration(uint16_t frequency, float phase)
{
  /*
   * Configure BIOZ path to measure an external 100 Ohm calibration resistor on PCB.
   * Uses normal BIOZ measurement path (no VCAL and no internal BIST resistor).
   * This is configure-only; call start() to begin conversions.
   */
  setupBIOZ(
    0,         // speed selector (low-rate mode)
    3,         // gain = 80V/V for low-impedance measurement
    6,         // ahpf bypass
    1,         // dlpf 4Hz
    0,         // dhpf bypass
    frequency, // modulation frequency
    8000,      // drive current [nA]
    phase,     // demodulation phase
    false,     // lead bias off for fixed external resistor path
    false,     // no lead-off detection
    false      // 2-wire BIOZ path
  );
}

/******************************************************************************************************/
// ECG & BIOZ simultanously
/******************************************************************************************************/

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
  * 55..96,000 nano Amps
  *     8,000 **
  *
  * BIOZ phase delay between current and voltage measurement 
  *  for accurate phase and impedance reading at least two phase settings 
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

  _profile = PROFILE_ECG_AND_BIOZ;
  _configured = true;
  _running = false;
  _useECG = true;
  _useBIOZ = true;
  _useRTOR = true;
  _drainECGOnUpdate = true;
  _drainBIOZOnUpdate = true;
  _readRTOROnUpdate = true;
  _applyLeadSettingsOnStart = true;
  _leadBiasEnable = leadbias;
  if (leadbias) {
    _leadBiasResistance = ecg_threeleads ? 0U : 100U;
  } else {
    _leadBiasResistance = 0U;
  }
  _leadOffEnable = leadsoffdetect;
  _leadOffBioz4 = bioz_fourleads;
  _leadOffElectrodeImpedance = 2U;
  _leadOnEnable = false;

  swReset(); // Reset the AFE
  setFMSTR(0); // Keep setup scripts on a known timing base and refresh dependent globals.

  // Input range checks
  if (ecg_speed > 2) { ecg_speed = 2; }
  if (ecg_gain > 3) { ecg_gain = 3; }
  if (bioz_speed > 1) { bioz_speed = 1; }
  if (bioz_gain > 3) { bioz_gain = 3; }
  if (bioz_dlpf > 3) { bioz_dlpf = 3; }
  if (bioz_dhpf > 2) { bioz_dhpf = 2; }
  if (bioz_current > 96000) { bioz_current = 96000; }
  if (bioz_current < 55) { bioz_current = 55; }

  // ECG
  setECGSamplingRate(ecg_speed);
  switch (ecg_speed) {
    case 0:
      setECGfilter(1, 0); // 40Hz and 0Hz
      break;
    case 1:
      setECGfilter(2, 0); // 100Hz and 0Hz
      break;
    case 2:
    default:
      setECGfilter(3, 0); // 150Hz and 0Hz
      break;
  }
  setECGgain(ecg_gain);
  if (ECG_gain > 0) {
    const int recovery_threshold_mV = static_cast<int>((0.98f * V_ref) / static_cast<float>(ECG_gain));
    setECGAutoRecovery(recovery_threshold_mV);
  } else {
    setECGNormalRecovery();
    LOGW("ECG gain not initialized correctly; fast auto-recovery disabled.");
  }
  setECGLeadPolarity(false, false);
  setDefaultRtoR(); // Keep R-to-R active in combined mode.

  // BIOZ
  setBIOZSamplingRate(bioz_speed);
  setBIOZgain(bioz_gain, true);
  setBIOZModulationFrequencyByFrequency(bioz_frequency);
  setBIOZmag(bioz_current);

  // 0 = Unchopped with LPF (required for low-current mode)
  // 2 = Chopped with LPF (default for high-current combined mode)
  const uint8_t bioz_modulation_mode = (bioz_current < 8000U) ? 0U : 2U;
  setBIOZmodulation(bioz_modulation_mode);

  // AHPF based on ECG speed to keep BIOZ channel above ECG baseband.
  // speed 0 -> 150Hz, speed 1 -> 500Hz, speed 2 -> 1000Hz
  uint8_t bioz_ahpf = 0U;
  if (ecg_speed == 0U) {
    bioz_ahpf = 1U;
  } else if (ecg_speed == 1U) {
    bioz_ahpf = 2U;
  } else {
    bioz_ahpf = 3U;
  }

  // Ensure AHPF is below selected BIOZ modulation frequency.
  {
    static const float ahpf_hz[6] = {60.0f, 150.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
    const float fmod_hz = BIOZ_frequency;

    uint8_t max_valid_ahpf = 6U;
    for (uint8_t i = 0; i < 6U; ++i) {
      if (ahpf_hz[i] < fmod_hz) {
        max_valid_ahpf = i;
      }
    }

    if (max_valid_ahpf == 6U) {
      LOGW("No AHPF corner is below %.0f Hz modulation; forcing AHPF bypass.", fmod_hz);
      bioz_ahpf = 6U;
    } else if (bioz_ahpf > max_valid_ahpf) {
      LOGW("Requested AHPF %.0f Hz is not below %.0f Hz modulation; using %.0f Hz.",
           ahpf_hz[bioz_ahpf], fmod_hz, ahpf_hz[max_valid_ahpf]);
      bioz_ahpf = max_valid_ahpf;
    }
  }

  setBIOZfilter(bioz_ahpf, bioz_dlpf, bioz_dhpf);
  setBIOZPhaseOffsetbyPhase(bioz_phase);

  // Disable calibration and internal impedance-test modes
  setDefaultNoTestSignal();
  setDefaultNoBIOZTestImpedance();

  // Interrupt behavior
  setDefaultInterruptClearing();
  setFIFOInterruptThreshold(32,8);

  // INT1: ECG + BIOZ FIFO/events
  setInterrupt1(true, true, false, false, false, false);
  // INT2: RtoR and optional lead-off/current-monitor events
  setInterrupt2(false, false, true, false, leadsoffdetect, bioz_fourleads);

  // Start-time actions (EN_ECG/EN_BIOZ/EN_RTOR, lead settings, FIFO reset, SYNCH)
  // are applied in start().

} // end initialize driver for ECG and BIOZ

/******************************************************************************************************/
// Impedance Spectroscopy
/******************************************************************************************************/
/*
 * Measures BIOZ by scanning over valid modulation frequencies and phase offsets:
 *
 * Up to 11 frequencies and 16 phase offsets are supported:
 * 
 * Frequency:  128,000, 80,000, 40,000, 17,780, 8,000, 4,000, 2,000, 1,000, (500, 250, 125)
 * Phase:      0..11.25..168.75 degrees
 * 
 * Default range is 128kHz..1kHz.
 * Full range includes also 500/250/125Hz.
 *
 * After measurements are completed, magnitude and phase are extracted at each frequency
 * 
 * Magnitude and phase will be 0.0 if measurement was unsuccessful.
 * 
 * To utilize full dynamic range. measurements start a 8 micro Ampere and current is decreased 
 * if signal is saturated or increased if signal is below 10% of the ADC range.
 * Since low frequencies have a reduced current range, current adjusment is limited.
 *
 * BIOZ sampling rate is 30sps or 60sps (when fast=true)
 * BIOZ gain 20V/V, low noise mode
 * BIOZ ahpf is auto-selected from the lowest scanned modulation frequency.
 * 
 * For each phase point, multiple FIFO readings are averaged with simple outlier rejection.
 * It should be assumed that the first few readings after a frequency or phase change are not stable.
 *
 * BIOZ analog HPF is selected automatically from datasheet cutoffs:
 * 60, 150, 500, 1000, 2000, 4000Hz (or bypass).
 * Selection rule:
 *   choose highest available cutoff that is strictly below the lowest scanned modulation frequency.
 *   Example: 500Hz -> AHPF 150Hz; 125Hz -> AHPF 60Hz.
 *
 * If we choose the following cut on frequencies for AHPF, suppresion of 60Hz noise will be:
 *   60Hz cut on:  -3dB or reduced to 70%
 *  150Hz cut on: -10dB or reduced to 30%
 *  500Hz cut on: -20dB or reduced to 10%
 *  Therefore to have 60Hz AC noise suppression near -20dB, use modulation >=1kHz and AHPF=500Hz.
 *  If 60Hz noise is making it on to the BIOZ signal, after demodulation that noise will appear
 *    at 60Hz and at modulation_frequency +/- 60Hz. The digial low pass filter will 
 *    remove that noise but only if the 60Hz noise is significanlty lower in power than 
 *    the BIOZ signal and not saturating the BIOZ channel.
 * 
 * At later time we can experiement with 60Hz or 0Hz analog high pass and verify if we have same
 * readings at high modulation frequencies.
 *
 * Expected scan time
 * ------------------
 * Points per sweep:
 *  - index 0..7  (128kHz..1kHz): 108 phase points
 *  - index 0..10 (128kHz..125Hz): 156 phase points
 * Approx acquisition time per attempt:
 * T ≈ points_per_sweep * (avg / BIOZ_samplingRate) (+ reconfig overhead).
 * Rough examples:
 *  avg=8, 30 sps: ~29 s acquisition per attempt plus overhead.
 *  avg=8, 60 sps: ~14–20 s per attempt.
 *  avg=2, 30 sps: ~7–10 s per attempt.
 * With retries (max_retries), worst-case can be much longer.
 *
 * high current
 * ---------------------------------------------------------------------
 * BioZ Current Generator Magnitude: 
 * cnfg_bioz.cgmag
 * 000 = Off (DRVP and DRVN floating, Current Generators Off)
 * 001 = 8µA (also use this setting when BIOZ_HI_LOB = 0)
 * 010 = 16µA
 * 011 = 32µA
 * 100 = 48µA
 * 101 = 64µA
 * 110 = 80µA
 * 111 = 96µA
 *
 * The following currents can be achied based on the selected modulation frequency: 
 *
 * BIOZ CURRENT GENERATOR MODULATION FREQUENCY (Hz)                                     CNFG_BIOZ
 * BIOZ_FCGEN[3:0]  FMSTR[1:0] = 00 FMSTR[1:0] = 01  FMSTR[1:0] = 10  FMSTR[1:0] = 11   CGMAG allowed
 *                  fMSTR= 32,768Hz fMSTR = 32,000Hz fMSTR = 32,000Hz fMSTR = 31,968Hz  
 * 0000             131,072         128,000          128,000         127,872            all
 * 0001              81,920          80,000           80,000          81,920            all
 * 0010              40,960          40,000           40,000          40,960            all
 * 0011              18,204          17,780           17,780          18,204            all
 * 0100               8,192           8,000            8,000           7,992            not 111
 * 0101               4,096           4,000            4,000           3,996            000 to 011
 * 0110               2,048           2,000            2,000           1,998            000 to 010
 * 0111               1,024           1,000            1,000             999            000, 001
 * 1000                 512             500              500             500            000, 001
 * 1001                 256             250              250             250            000, 001
 * 101x,11xx            128             125              125             125            000, 001
 *
 * low current
 * ---------------------------------------------------------------------
 *   cmag == 001 
 *   cnfg_bioz_lc.bit.hi_lob == 0 
 *   cnfg_bioz_lc.bit.lc2x == 0 55-1100nA
 *   cnfg_bioz_lc.bit.lc2x == 1 110-1100nA
 *   set the common mode resistance as recommended in BIOZ_CMRES.
 *
 * BIOZ Low Current Generator Magnitude: 
 * cnfg_bioz_lc.bit.cmag_lc
 *         LC2X = 0 LC2X = 1
 * 0000    0        0
 * 0001    55nA     110nA
 * 0010    110nA    220nA
 * 0011    220nA    440nA
 * 0100    330nA    660nA
 * 0101    440nA    880nA
 * 0110    550nA    1100nA
 */
/******************************************************************************************************/

void MAX30001G::setupBIOZScan(uint8_t avg, bool fast, bool fourleads, bool fullRange) {
  BIOZScanConfig config;
  config.avg = avg;
  config.fast = fast;
  config.fourleads = fourleads;
  config.freq_end_index = fullRange ? (MAX30001_BIOZ_NUM_FREQUENCIES - 1U) : 7U;
  setupBIOZScan(config, false);
}

void MAX30001G::setupBIOZScan(const BIOZScanConfig& config, bool reuseCurrents) {
/*
 * BIOZ Scan Configuration
 * avg:       number of samples to average at each frequency/phase point (1-8)
 * fast:      if true, use 60sps BIOZ sampling rate; otherwise use 30sps
 * fourleads: if true, use 4-wire BIOZ configuration; otherwise use 2-wire
 * use_internal_resistor: if true, scan the internal calibration resistor instead of external electrodes
 * internal_resistor_ohm: nominal internal resistor, nearest supported value is selected
 * max_retries: number of retries per frequency/phase point if measurement is unsuccessful (0-3)
 * low_target_fraction: signal is low if below 10% of ADC range for current adjustment
 * high_target_fraction: signal is high if above 90% of ADC range for current adjustment
 * target_fraction: signal is ok if above 60% of ADC range for current adjustment
 * outlier_sigma: number of standard deviations for outlier rejection when averaging multiple readings at each point
 * timeout_margin_ms: margin in milliseconds for FIFO read timeout when waiting for samples at each point
 * freq_start_index: index of first modulation frequency to scan (0-10, corresponding to 128kHz..125Hz)
 * freq_end_index: index of last modulation frequency to scan (0-10, corresponding to 128kHz..125Hz)
 * initial_current_nA: initial current magnitude in nanoAmps (55..96,000)
 * 
 */

  BIOZScanConfig sanitized_config = config;

  if (sanitized_config.avg < 1U) { sanitized_config.avg = 1U; }
  if (sanitized_config.avg > 8U) { sanitized_config.avg = 8U; }

  if (sanitized_config.max_retries < 1U) { sanitized_config.max_retries = 1U; }
  if (sanitized_config.max_retries > 10U) { sanitized_config.max_retries = 10U; }

  if (sanitized_config.low_target_fraction < 0.02f) { sanitized_config.low_target_fraction = 0.02f; }
  if (sanitized_config.low_target_fraction > 0.80f) { sanitized_config.low_target_fraction = 0.80f; }
  if (sanitized_config.high_target_fraction < 0.20f) { sanitized_config.high_target_fraction = 0.20f; }
  if (sanitized_config.high_target_fraction > 0.98f) { sanitized_config.high_target_fraction = 0.98f; }
  if (sanitized_config.high_target_fraction <= sanitized_config.low_target_fraction) {
    sanitized_config.high_target_fraction = sanitized_config.low_target_fraction + 0.05f;
    if (sanitized_config.high_target_fraction > 0.98f) {
      sanitized_config.high_target_fraction = 0.98f;
    }
  }
  if (sanitized_config.target_fraction < sanitized_config.low_target_fraction) {
    sanitized_config.target_fraction = sanitized_config.low_target_fraction;
  }
  if (sanitized_config.target_fraction > sanitized_config.high_target_fraction) {
    sanitized_config.target_fraction = sanitized_config.high_target_fraction;
  }

  if (sanitized_config.outlier_min_samples < 3U) { sanitized_config.outlier_min_samples = 3U; }
  if (sanitized_config.outlier_min_samples > MAX30001_BIOZ_NUM_PHASES) {
    sanitized_config.outlier_min_samples = MAX30001_BIOZ_NUM_PHASES;
  }
  if (sanitized_config.outlier_sigma < 0.5f) { sanitized_config.outlier_sigma = 0.5f; }
  if (sanitized_config.outlier_sigma > 6.0f) { sanitized_config.outlier_sigma = 6.0f; }

  if (sanitized_config.freq_start_index >= MAX30001_BIOZ_NUM_FREQUENCIES) {
    sanitized_config.freq_start_index = 0U;
  }
  if (sanitized_config.freq_end_index >= MAX30001_BIOZ_NUM_FREQUENCIES) {
    sanitized_config.freq_end_index = MAX30001_BIOZ_NUM_FREQUENCIES - 1U;
  }
  if (sanitized_config.freq_start_index > sanitized_config.freq_end_index) {
    const uint8_t tmp = sanitized_config.freq_start_index;
    sanitized_config.freq_start_index = sanitized_config.freq_end_index;
    sanitized_config.freq_end_index = tmp;
  }
  if (sanitized_config.initial_current_nA < 55) { sanitized_config.initial_current_nA = 55; }
  if (sanitized_config.initial_current_nA > 96000) { sanitized_config.initial_current_nA = 96000; }
  sanitized_config.initial_current_nA = closestCurrent(sanitized_config.initial_current_nA);

  if (sanitized_config.internal_resistor_ohm < 625U) { sanitized_config.internal_resistor_ohm = 625U; }
  if (sanitized_config.internal_resistor_ohm > 5000U) { sanitized_config.internal_resistor_ohm = 5000U; }

  _profile = PROFILE_BIOZ_SCAN;
  _configured = true;
  _running = false;
  _useECG = false;
  _useBIOZ = false;
  _useRTOR = false;
  _drainECGOnUpdate = false;
  _drainBIOZOnUpdate = false;
  _readRTOROnUpdate = false;
  _applyLeadSettingsOnStart = false;
  _leadBiasEnable = false;
  _leadBiasResistance = 0U;
  _leadOffEnable = false;
  _leadOffBioz4 = false;
  _leadOffElectrodeImpedance = 0U;
  _leadOnEnable = false;

  _scanRuntimeConfig = sanitized_config;
  _scanReuseCurrents = reuseCurrents;
  _scanInProgress = false;
  _scanCompleted = false;
  _BIOZScanState = BIOZ_SCAN_IDLE;
  _scanFreqIndex = 0U;
  _scanPhaseIndex = 0U;
  _scanAttempt = 0U;
  _scanNumPhaseMeasurements = 0U;
  _scanWaitDeadlineMs = 0U;
}

/*
 * Starts the BIOZ scan process.
 */
void MAX30001G::startBIOZScan() {
  if (!_configured || (_profile != PROFILE_BIOZ_SCAN)) {
    LOGW("AFE: startBIOZScan() ignored because BIOZ scan profile is not configured.");
    return;
  }
  _BIOZScanState = BIOZ_SCAN_INIT;
  _scanInProgress = true;
  _scanCompleted = false;
  _running = true;
}

/*
 * Stops the BIOZ scan process.
 */
void MAX30001G::stopBIOZScan() {
  enableAFE(false, false, false);
  _scanInProgress = false;
  _scanCompleted = false;
  _running = false;
  _BIOZScanState = BIOZ_SCAN_IDLE;
}

/*
 * Returns the number of phases to scan for a given frequency index.
 */
uint8_t MAX30001G::biozScanPhaseCountForFreq(uint8_t freq_idx) const {
  if (freq_idx == 0U) { return 4U; }
  if (freq_idx == 1U) { return 8U; }
  return MAX30001_BIOZ_NUM_PHASES;
}

/* 
 * Selects the appropriate AHpf value based on the lowest frequency measured in the BIOZ scan.
 */
uint8_t MAX30001G::biozScanSelectAHpf(float lowest_frequency_hz) const {
  static const float kAhpfCutoffHz[6] = {60.0f, 150.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
  uint8_t selected = 0U;
  for (uint8_t i = 0U; i < 6U; ++i) {
    if (kAhpfCutoffHz[i] < lowest_frequency_hz) {
      selected = i;
    }
  }
  return selected;
}

/*
 * Calculates the robust mean of samples in the BIOZ_data RingBuffer, excluding outliers.
 */
float MAX30001G::biozScanRobustMeanFromBuffer(uint8_t outlier_min_samples, float outlier_sigma, bool& hasSamples) {
  float samples[32];
  uint8_t n = 0U;
  while ((BIOZ_data.available() > 0U) && (n < 32U)) {
    float value = 0.0f;
    if (BIOZ_data.pop(value) != 1U) {
      break;
    }
    samples[n++] = value;
  }

  hasSamples = (n > 0U);
  if (!hasSamples) {
    return NAN;
  }

  float mean = 0.0f;
  for (uint8_t i = 0U; i < n; ++i) {
    mean += samples[i];
  }
  mean /= static_cast<float>(n);

  if (n < outlier_min_samples) {
    return mean;
  }

  float var = 0.0f;
  for (uint8_t i = 0U; i < n; ++i) {
    const float d = samples[i] - mean;
    var += d * d;
  }
  var /= static_cast<float>(n);
  const float std = sqrtf(var);
  if (std <= 0.0f) {
    return mean;
  }

  const float reject = outlier_sigma * std;
  float filtered_sum = 0.0f;
  uint8_t filtered_n = 0U;
  for (uint8_t i = 0U; i < n; ++i) {
    if (fabsf(samples[i] - mean) <= reject) {
      filtered_sum += samples[i];
      filtered_n++;
    }
  }
  if (filtered_n == 0U) {
    return mean;
  }
  return filtered_sum / static_cast<float>(filtered_n);
}

/*************************************************
 * This is the state machine for performing the BIOZ scan. 
 * It is called repeatedly in the main loop through  update function
 * and moves the scan through its the scan process.
 * The states are: 
 *  - initializing the scan, 
 *  - configuring the frequency
 *  - setting current generator for current frequency
 *  - configuring the phase offset
 *  - awaiting samples
 *  - process samples which includes calculating the mean and outlier rejection
 *  - avaluating all phase measuremetns at given frequency and deciding if we need to adjust current and retry or move on to next frequency
 *  - fit impedance model to phase measurements at current frequency
 *  - finalize and push spectrum to RingBuffer when all frequencies are completed or stop if error or user stopped the scan.
 */

void MAX30001G::stepBIOZScan() {
  if (!_scanInProgress) {
    return;
  }

  constexpr float kNominalFreqHz[MAX30001_BIOZ_NUM_FREQUENCIES] = {
    128000.0f, 80000.0f, 40000.0f, 17780.0f, 8000.0f, 4000.0f,
    2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f
  };

  const uint8_t freq_start_index = _scanRuntimeConfig.freq_start_index;
  const uint8_t freq_end_index = _scanRuntimeConfig.freq_end_index;
  const uint8_t avg = _scanRuntimeConfig.avg;
  const bool fast = _scanRuntimeConfig.fast;
  const bool fourleads = _scanRuntimeConfig.fourleads;
  const bool use_internal_resistor = _scanRuntimeConfig.use_internal_resistor;

  switch (_BIOZScanState) {
    case BIOZ_SCAN_INIT: {
      _scanThresholdMin = 524288.0f * _scanRuntimeConfig.low_target_fraction;
      _scanThresholdMax = 524288.0f * _scanRuntimeConfig.high_target_fraction;
      _scanAdcTarget = 524288.0f * _scanRuntimeConfig.target_fraction;

      for (uint8_t i = 0U; i < MAX30001_BIOZ_NUM_FREQUENCIES; ++i) {
        _scanFrequency[i] = 0.0f;
        _scanCurrent[i] = _scanRuntimeConfig.initial_current_nA;
        impedance_magnitude[i] = 0.0f;
        impedance_phase[i] = 0.0f;
        impedance_frequency[i] = 0.0f;
        for (uint8_t j = 0U; j < MAX30001_BIOZ_NUM_PHASES; ++j) {
          _scanImpedance[i][j] = NAN;
          _scanPhaseDeg[i][j] = NAN;
        }
      }

      const bool reuseProfile = _scanReuseCurrents &&
                                _scanCurrentProfileValid &&
                                (_scanProfileFreqStart == freq_start_index) &&
                                (_scanProfileFreqEnd == freq_end_index) &&
                                (_scanProfileFast == fast);
      if (reuseProfile) {
        for (uint8_t i = freq_start_index; i <= freq_end_index; ++i) {
          _scanCurrent[i] = _scanCurrentProfile[i];
        }
        LOGI("scanBIOZ: reusing previously optimized current profile.");
      }

      swReset();
      setFMSTR(0);

      setBIOZSamplingRate(fast ? 1U : 0U);
      setBIOZgain(1U, true);
      setBIOZmodulation(2U);

      const float lowest_nominal_freq_hz = kNominalFreqHz[freq_end_index];
      const uint8_t ahpf = biozScanSelectAHpf(lowest_nominal_freq_hz);
      setBIOZfilter(ahpf, 1U, 0U);
      if (freq_end_index == 9U) {
        LOGI("scanBIOZ: 250Hz sweep uses AHPF=150Hz (no 125Hz AHPF option in MAX30001).");
      }

      setDefaultNoTestSignal();
      if (use_internal_resistor) {
        const uint16_t resistor = _scanRuntimeConfig.internal_resistor_ohm;
        uint8_t rnom_value = 0U;
        if      (resistor > 2500U) { rnom_value = 0U; }
        else if (resistor > 1667U) { rnom_value = 1U; }
        else if (resistor > 1250U) { rnom_value = 2U; }
        else if (resistor > 1000U) { rnom_value = 3U; }
        else if (resistor >  833U) { rnom_value = 4U; }
        else if (resistor >  714U) { rnom_value = 5U; }
        else if (resistor >  625U) { rnom_value = 6U; }
        else                       { rnom_value = 7U; }
        setBIOZTestImpedance(true, false, false, rnom_value, 0U, 0U);
      } else {
        setDefaultNoBIOZTestImpedance();
      }
      setDefaultNoRtoR();
      setDefaultInterruptClearing();

      setFIFOInterruptThreshold(1U, avg);
      setInterrupt1(false, true, false, false, false);
      setInterrupt2(false, false, false, false, false);

      enableAFE(false, true, false);
      setLeadsBias(false, 0U);
      setLeadsOffDetection(false, fourleads, 0U);
      setLeadsOnDetection(false);

      FIFOReset();
      synch();

      _scanFreqIndex = freq_start_index;
      _BIOZScanState = BIOZ_SCAN_CONFIG_FREQ;
      return;
    }

    case BIOZ_SCAN_CONFIG_FREQ: {
      if (_scanFreqIndex > freq_end_index) {
        _BIOZScanState = BIOZ_SCAN_FINISH;
        return;
      }

      setBIOZModulationFrequencybyIndex(_scanFreqIndex);
      _scanFrequency[_scanFreqIndex] = BIOZ_frequency;
      _scanNumPhaseMeasurements = biozScanPhaseCountForFreq(_scanFreqIndex);
      _scanAttempt = 0U;
      _BIOZScanState = BIOZ_SCAN_PREPARE_ATTEMPT;
      return;
    }

    case BIOZ_SCAN_PREPARE_ATTEMPT: {
      setBIOZmag(_scanCurrent[_scanFreqIndex]);
      _scanCurrent[_scanFreqIndex] = BIOZ_cgmag;

      _scanMaxAbsRaw = 0.0f;
      _scanSawValid = false;
      _scanSawInvalidOrRange = false;
      _scanAnyAboveTarget = false;
      _scanAnyInTarget = false;

      _scanPhaseIndex = 0U;
      _BIOZScanState = BIOZ_SCAN_CONFIG_PHASE;
      return;
    }

    case BIOZ_SCAN_CONFIG_PHASE: {
      if (_scanPhaseIndex >= _scanNumPhaseMeasurements) {
        _BIOZScanState = BIOZ_SCAN_EVALUATE_ATTEMPT;
        return;
      }

      setBIOZPhaseOffsetbyIndex(_scanPhaseIndex);
      _scanPhaseDeg[_scanFreqIndex][_scanPhaseIndex] = BIOZ_phase;

      FIFOReset();
      synch();

      BIOZ_data.clear();
      bioz_available = false;
      afe_irq_pending = false;
      afe_irq1_pending = false;
      afe_irq2_pending = false;
      valid_data_detected = false;
      over_voltage_detected = false;
      under_voltage_detected = false;

      const float sps = (BIOZ_samplingRate > 1.0f) ? BIOZ_samplingRate : 1.0f;
      const uint32_t timeout_ms = static_cast<uint32_t>(
        (1000.0f * (static_cast<float>(avg) + 2.0f) / sps) +
        static_cast<float>(_scanRuntimeConfig.timeout_margin_ms)
      );
      _scanWaitDeadlineMs = millis() + timeout_ms;
      _BIOZScanState = BIOZ_SCAN_WAIT_DATA;
      return;
    }

    case BIOZ_SCAN_WAIT_DATA: {
      bool serviced = servicePendingInterrupts();
      if (!serviced && (_intPin1 < 0) && (_intPin2 < 0)) {
        serviceAllInterrupts();
      }

      if (bioz_available) {
        bioz_available = false;
        readBIOZ_FIFO(true);
        _BIOZScanState = BIOZ_SCAN_PROCESS_PHASE;
        return;
      }

      if (static_cast<int32_t>(millis() - _scanWaitDeadlineMs) >= 0) {
        LOGW("scanBIOZ: timeout waiting for BIOZ data (phase idx=%u).", _scanPhaseIndex);
        _scanSawInvalidOrRange = true;
        _scanImpedance[_scanFreqIndex][_scanPhaseIndex] = NAN;
        _scanPhaseIndex++;
        _BIOZScanState = BIOZ_SCAN_CONFIG_PHASE;
      }
      return;
    }

    case BIOZ_SCAN_PROCESS_PHASE: {
      bool hasSamples = false;
      const float raw_mean = biozScanRobustMeanFromBuffer(
        _scanRuntimeConfig.outlier_min_samples, _scanRuntimeConfig.outlier_sigma, hasSamples
      );
      const bool got_sample = hasSamples && isfinite(raw_mean);
      const bool range_or_invalid = (!valid_data_detected ||
                                     over_voltage_detected ||
                                     under_voltage_detected ||
                                     !isfinite(raw_mean));

      if (!got_sample) {
        _scanSawInvalidOrRange = true;
        _scanImpedance[_scanFreqIndex][_scanPhaseIndex] = NAN;
      } else {
        _scanSawValid = true;
        if (range_or_invalid) {
          _scanSawInvalidOrRange = true;
        }

        const float abs_raw = fabsf(raw_mean);
        if (abs_raw > _scanMaxAbsRaw) {
          _scanMaxAbsRaw = abs_raw;
        }
        if (abs_raw > _scanThresholdMax) {
          _scanAnyAboveTarget = true;
        } else if (abs_raw >= _scanThresholdMin) {
          _scanAnyInTarget = true;
        }

        const float denom = 524288.0f * static_cast<float>(BIOZ_cgmag) * static_cast<float>(BIOZ_gain) * 1e-9f;
        _scanImpedance[_scanFreqIndex][_scanPhaseIndex] =
            (denom > 0.0f) ? (raw_mean * static_cast<float>(V_ref)) / denom : NAN;

        LOGD("scanBIOZ: f=%.1fHz, phase=%.2fdeg, raw=%.1f, Z=%.2fohm, I=%ldnA",
             _scanFrequency[_scanFreqIndex], _scanPhaseDeg[_scanFreqIndex][_scanPhaseIndex], raw_mean,
             _scanImpedance[_scanFreqIndex][_scanPhaseIndex], static_cast<long>(_scanCurrent[_scanFreqIndex]));
      }

      _scanPhaseIndex++;
      _BIOZScanState = BIOZ_SCAN_CONFIG_PHASE;
      return;
    }

    case BIOZ_SCAN_EVALUATE_ATTEMPT: {
      bool accepted = false;

      if (!_scanSawValid) {
        LOGW("scanBIOZ: no valid samples at frequency %.1fHz.", _scanFrequency[_scanFreqIndex]);
      } else if (_scanMaxAbsRaw <= 0.0f) {
        LOGW("scanBIOZ: zero raw magnitude at frequency %.1fHz.", _scanFrequency[_scanFreqIndex]);
      } else {
        const bool reduce_current = _scanSawInvalidOrRange || _scanAnyAboveTarget;
        const bool increase_current = (!reduce_current && !_scanAnyInTarget);

        if (reduce_current || increase_current) {
          int32_t desired_current = _scanCurrent[_scanFreqIndex];

          if (reduce_current) {
            desired_current = closestCurrent(static_cast<int32_t>(
              static_cast<float>(_scanCurrent[_scanFreqIndex]) * (_scanAdcTarget / _scanMaxAbsRaw)
            ));
            if (desired_current >= _scanCurrent[_scanFreqIndex]) {
              desired_current = closestCurrent(_scanCurrent[_scanFreqIndex] / 2);
            }
          } else if (increase_current) {
            desired_current = closestCurrent(static_cast<int32_t>(
              static_cast<float>(_scanCurrent[_scanFreqIndex]) * (_scanAdcTarget / _scanMaxAbsRaw)
            ));
          }

          if ((desired_current != _scanCurrent[_scanFreqIndex]) &&
              ((_scanAttempt + 1U) < _scanRuntimeConfig.max_retries)) {
            _scanCurrent[_scanFreqIndex] = desired_current;
            _scanAttempt++;
            _BIOZScanState = BIOZ_SCAN_PREPARE_ATTEMPT;
            return;
          }
        }

        accepted = true;
      }

      if (!accepted) {
        LOGW("scanBIOZ: using best-effort data at frequency %.1fHz.", _scanFrequency[_scanFreqIndex]);
      }
      _BIOZScanState = BIOZ_SCAN_FINALIZE_FREQ;
      return;
    }

    case BIOZ_SCAN_FINALIZE_FREQ: {
      const ImpedanceModel result = fitImpedance(
        _scanPhaseDeg[_scanFreqIndex], _scanImpedance[_scanFreqIndex], _scanNumPhaseMeasurements
      );
      impedance_magnitude[_scanFreqIndex] = result.magnitude;
      impedance_phase[_scanFreqIndex] = result.phase;
      impedance_frequency[_scanFreqIndex] = _scanFrequency[_scanFreqIndex];
      _scanCurrentProfile[_scanFreqIndex] = _scanCurrent[_scanFreqIndex];

      _scanFreqIndex++;
      _BIOZScanState = BIOZ_SCAN_CONFIG_FREQ;
      return;
    }

    case BIOZ_SCAN_FINISH: {
      _scanCurrentProfileValid = true;
      _scanProfileFreqStart = freq_start_index;
      _scanProfileFreqEnd = freq_end_index;
      _scanProfileFast = fast;

      ImpedanceSpectrum spectrum;
      for (uint8_t i = 0U; i < MAX30001_BIOZ_NUM_FREQUENCIES; ++i) {
        spectrum.frequency[i] = impedance_frequency[i];
        spectrum.magnitude[i] = impedance_magnitude[i];
        spectrum.phase[i] = impedance_phase[i];
      }
      if (BIOZ_spectrum.push(spectrum, true) != 1U) {
        LOGW("BIOZ spectrum ring buffer push failed.");
      }

      _scanInProgress = false;
      _scanCompleted = true;
      _running = false;
      _BIOZScanState = BIOZ_SCAN_IDLE;
      enableAFE(false, false, false);
      return;
    }

    case BIOZ_SCAN_IDLE:
    default:
      _scanInProgress = false;
      _running = false;
      return;
  }
}

void MAX30001G::scanBIOZ(uint8_t avg, bool fast, bool fourleads, bool fullRange) {
  BIOZScanConfig config;
  config.avg = avg;
  config.fast = fast;
  config.fourleads = fourleads;
  config.freq_end_index = fullRange ? (MAX30001_BIOZ_NUM_FREQUENCIES - 1U) : 7U;
  scanBIOZ(config, true);
}

void MAX30001G::scanBIOZ(const BIOZScanConfig& config, bool reuseCurrents) {

  constexpr uint8_t kNumFreq = MAX30001_BIOZ_NUM_FREQUENCIES; // FCGEN 0000..1010 -> 128k..125Hz
  constexpr uint8_t kMaxPhase = MAX30001_BIOZ_NUM_PHASES;     // max supported phase offsets
  constexpr float kNominalFreqHz[kNumFreq] = {
    128000.0f, 80000.0f, 40000.0f, 17780.0f, 8000.0f, 4000.0f,
    2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f
  };

  uint8_t avg = config.avg;
  bool fast = config.fast;
  bool fourleads = config.fourleads;
  uint8_t max_retries = config.max_retries;
  float low_target_fraction = config.low_target_fraction;
  float high_target_fraction = config.high_target_fraction;
  float target_fraction = config.target_fraction;
  uint8_t outlier_min_samples = config.outlier_min_samples;
  float outlier_sigma = config.outlier_sigma;
  uint16_t timeout_margin_ms = config.timeout_margin_ms;
  uint8_t freq_start_index = config.freq_start_index;
  uint8_t freq_end_index = config.freq_end_index;
  int32_t initial_current_nA = config.initial_current_nA;
  bool use_internal_resistor = config.use_internal_resistor;
  uint16_t internal_resistor_ohm = config.internal_resistor_ohm;

  if (avg < 1U) { avg = 1U; }
  if (avg > 8U) { avg = 8U; }

  if (max_retries < 1U) { max_retries = 1U; }
  if (max_retries > 10U) { max_retries = 10U; }

  if (low_target_fraction < 0.02f) { low_target_fraction = 0.02f; }
  if (low_target_fraction > 0.80f) { low_target_fraction = 0.80f; }
  if (high_target_fraction < 0.20f) { high_target_fraction = 0.20f; }
  if (high_target_fraction > 0.98f) { high_target_fraction = 0.98f; }
  if (high_target_fraction <= low_target_fraction) {
    high_target_fraction = low_target_fraction + 0.05f;
    if (high_target_fraction > 0.98f) { high_target_fraction = 0.98f; }
  }
  if (target_fraction < low_target_fraction) { target_fraction = low_target_fraction; }
  if (target_fraction > high_target_fraction) { target_fraction = high_target_fraction; }

  if (outlier_min_samples < 3U) { outlier_min_samples = 3U; }
  if (outlier_min_samples > kMaxPhase) { outlier_min_samples = kMaxPhase; }
  if (outlier_sigma < 0.5f) { outlier_sigma = 0.5f; }
  if (outlier_sigma > 6.0f) { outlier_sigma = 6.0f; }

  if (freq_start_index >= kNumFreq) { freq_start_index = 0U; }
  if (freq_end_index >= kNumFreq) { freq_end_index = kNumFreq - 1U; }
  if (freq_end_index < freq_start_index) {
    const uint8_t t = freq_start_index;
    freq_start_index = freq_end_index;
    freq_end_index = t;
  }

  if (initial_current_nA < 55) { initial_current_nA = 55; }
  if (initial_current_nA > 96000) { initial_current_nA = 96000; }
  initial_current_nA = closestCurrent(initial_current_nA);
  if (internal_resistor_ohm < 625U) { internal_resistor_ohm = 625U; }
  if (internal_resistor_ohm > 5000U) { internal_resistor_ohm = 5000U; }

  // Raw ADC target window for current adaptation.
  const float threshold_min = 524288.0f * low_target_fraction;
  const float threshold_max = 524288.0f * high_target_fraction;
  const float adc_target    = 524288.0f * target_fraction;

  float frequency[kNumFreq];
  int32_t current[kNumFreq];
  float impedance[kNumFreq][kMaxPhase];
  float phase[kNumFreq][kMaxPhase];

  if (config.avg > 8U) {
    LOGW("scanBIOZ: avg > 8, clamped to 8.");
  }

  // Initialize results to 0 or NAN as appropriate.
  for (uint8_t i = 0; i < kNumFreq; ++i) {
    frequency[i] = 0.0f;
    current[i] = initial_current_nA; // configurable initial seed
    impedance_magnitude[i] = 0.0f;
    impedance_phase[i] = 0.0f;
    impedance_frequency[i] = 0.0f;
    for (uint8_t j = 0; j < kMaxPhase; ++j) {
      impedance[i][j] = NAN;
      phase[i][j] = NAN;
    }
  }

  const bool reuseProfile = reuseCurrents &&
                            _scanCurrentProfileValid &&
                            (_scanProfileFreqStart == freq_start_index) &&
                            (_scanProfileFreqEnd == freq_end_index) &&
                            (_scanProfileFast == fast);
  if (reuseProfile) {
    for (uint8_t i = freq_start_index; i <= freq_end_index; ++i) {
      current[i] = _scanCurrentProfile[i];
    }
    LOGI("scanBIOZ: reusing previously optimized current profile.");
  }

  swReset();   // Reset AFE to known baseline.
  setFMSTR(0); // Keep timing-dependent globals consistent.

  setBIOZSamplingRate(fast ? 1U : 0U);  // 30 or 60 sps
  setBIOZgain(1U, true);                // 20V/V, low-noise
  setBIOZmodulation(2U);                // chopped + LPF

  const float lowest_nominal_freq_hz = kNominalFreqHz[freq_end_index];
  auto selectAHpfFromLowestFrequency = [](float lowest_frequency_hz) -> uint8_t {
    // AHPF codes 0..5 map to 60,150,500,1000,2000,4000Hz; 6/7 = bypass.
    // Use highest cutoff strictly below the lowest modulation frequency.
    constexpr float kAhpfCutoffHz[6] = {60.0f, 150.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
    uint8_t selected = 0U; // default to 60Hz when no lower option exists.
    for (uint8_t i = 0U; i < 6U; ++i) {
      if (kAhpfCutoffHz[i] < lowest_frequency_hz) {
        selected = i;
      }
    }
    return selected;
  };
  const uint8_t ahpf = selectAHpfFromLowestFrequency(lowest_nominal_freq_hz);
  setBIOZfilter(ahpf, 1U, 0U);          // DLPF=4Hz, DHPF=bypass
  if (freq_end_index == 9U) {
    LOGI("scanBIOZ: 250Hz sweep uses AHPF=150Hz (no 125Hz AHPF option in MAX30001).");
  }

  setDefaultNoTestSignal();            // Disable ECG/BIOZ calibration (VCAL must be disabled for BIOZ impedance test).
  if (use_internal_resistor) {
    uint8_t rnom_value = 0U;
    if      (internal_resistor_ohm > 2500U) { rnom_value = 0U; }
    else if (internal_resistor_ohm > 1667U) { rnom_value = 1U; }
    else if (internal_resistor_ohm > 1250U) { rnom_value = 2U; }
    else if (internal_resistor_ohm > 1000U) { rnom_value = 3U; }
    else if (internal_resistor_ohm >  833U) { rnom_value = 4U; }
    else if (internal_resistor_ohm >  714U) { rnom_value = 5U; }
    else if (internal_resistor_ohm >  625U) { rnom_value = 6U; }
    else                                    { rnom_value = 7U; }
    setBIOZTestImpedance(true, false, false, rnom_value, 0U, 0U);
  } else {
    setDefaultNoBIOZTestImpedance();      // Disable BIOZ impedance test mode.
  }
  setDefaultNoRtoR();                   // Disable R-to-R (not needed for BIOZ scan, and may cause unwanted interrupts if enabled).
  setDefaultInterruptClearing();        // Clear on read for all interrupts, no FIFO count-based clearing.

  setFIFOInterruptThreshold(1U, avg);               // BIOZ FIFO interrupt after avg samples, turn off for ECG
  setInterrupt1(false, true, false, false, false);  // BIOZ IRQ on INT1
  setInterrupt2(false, false, false, false, false); // INT2 unused for scan

  enableAFE(false, true, false);          // BIOZ only, ECG and RtoR off
  setLeadsBias(false, 0);                 // no lead bias during scan
  setLeadsOffDetection(false, fourleads, 0); // no lead off detection during scan
  setLeadsOnDetection(false);             // no lead on detection during scan 

  FIFOReset();
  synch();

  auto phaseCountForFreq = [](uint8_t freq_idx) -> uint8_t {
    if (freq_idx == 0U) {
      return 4U;  // 128kHz
    }
    if (freq_idx == 1U) {
      return 8U;  // 80kHz
    }
    return kMaxPhase;   // 40kHz and below
  };

  auto waitForBiozData = [&](uint32_t timeout_ms) -> bool {
    // Wait for BIOZ FIFO data to be available, with a timeout. 
    // Read registers until bioz_available is true.  
    const uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
      if (bioz_available) {
        return true;
      }
      if (afe_irq_pending || afe_irq1_pending || afe_irq2_pending) {
        afe_irq_pending = false;
        afe_irq1_pending = false;
        afe_irq2_pending = false;
        serviceAllInterrupts();
      } else {
        // Fallback polling path if IRQ was not wired by the application.
        serviceAllInterrupts();
        delay(1);
      }
    }
    return false;
  };

  auto robustMeanFromBiozBuffer = [&]() -> float {
    // Compute a robust mean of the samples in the BIOZ FIFO buffer 
    // with simple outlier rejection.
    float samples[32];
    uint8_t n = 0U;
    while ((BIOZ_data.available() > 0) && (n < 32U)) {
      float sample = 0.0f;
      if (BIOZ_data.pop(sample) != 1U) {
        break;
      }
      samples[n++] = sample;
    }
    if (n == 0U) {
      return NAN;
    }

    // compute mean
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
      mean += samples[i];
    }
    mean /= static_cast<float>(n);

    if (n < outlier_min_samples) {
      return mean; // Too few samples for meaningful outlier filtering.
    }

    // compute standard deviation
    float var = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
      const float d = samples[i] - mean;
      var += d * d;
    }
    var /= static_cast<float>(n);
    const float std = sqrtf(var);
    if (std <= 0.0f) {
      return mean;
    }

    // reject outliers more than 2.5 standard deviations from the mean 
    // and compute mean of remaining samples
    const float reject = outlier_sigma * std;
    float filtered_sum = 0.0f;
    uint8_t filtered_n = 0U;
    for (uint8_t i = 0; i < n; ++i) {
      if (fabsf(samples[i] - mean) <= reject) {
        filtered_sum += samples[i];
        filtered_n++;
      }
    }
    if (filtered_n == 0U) {
      return mean;
    }
    return filtered_sum / static_cast<float>(filtered_n);
  };

  auto measurePhaseRaw = [&](uint8_t phase_selector, float &raw_mean, bool &range_or_invalid) -> bool {
    // Measure raw BIOZ ADC values at the given phase offset index, and compute a robust mean.
    // Returns true if a valid mean was computed, false if no valid data was detected or an error occurred.
    // Range_or_invalid is set to true if the data is suspected to be out of range or invalid, which can occur if the signal is saturating or too low.
    
    setBIOZPhaseOffsetbyIndex(phase_selector); // set phase offset for this measurement
    FIFOReset();
    synch();

    BIOZ_data.clear();
    bioz_available = false;
    afe_irq_pending = false;
    afe_irq1_pending = false;
    afe_irq2_pending = false;

    const float sps = (BIOZ_samplingRate > 1.0f) ? BIOZ_samplingRate : 1.0f;
    const uint32_t timeout_ms = static_cast<uint32_t>(
      (1000.0f * (static_cast<float>(avg) + 2.0f) / sps) + static_cast<float>(timeout_margin_ms)
    );
    if (!waitForBiozData(timeout_ms)) {
      LOGW("scanBIOZ: timeout waiting for BIOZ data (phase idx=%u).", phase_selector);
      raw_mean = NAN;
      range_or_invalid = true;
      return false;
    }

    bioz_available = false;
    readBIOZ_FIFO(true); // raw ADC counts

    raw_mean = robustMeanFromBiozBuffer();
    range_or_invalid = (!valid_data_detected || over_voltage_detected || under_voltage_detected || !isfinite(raw_mean));
    return isfinite(raw_mean);
  };

  /// Main scan loop: iterate over frequencies, adapt current, and measure all phases.
  // ----------------------------------------------------------------------------------

  //over all frequencies --------
  for (uint8_t freq_idx = freq_start_index; freq_idx <= freq_end_index; ++freq_idx) {
    setBIOZModulationFrequencybyIndex(freq_idx);
    frequency[freq_idx] = BIOZ_frequency;
    const uint8_t num_phase_measurements = phaseCountForFreq(freq_idx);

    bool accepted = false;
    for (uint8_t attempt = 0U; attempt < max_retries; ++attempt) {

      setBIOZmag(current[freq_idx]);
      current[freq_idx] = BIOZ_cgmag; // actual value after device quantization/limits

      float max_abs_raw = 0.0f;
      bool saw_invalid_or_range = false;
      bool saw_valid = false;
      bool any_above_target = false;
      bool any_in_target = false;

      // over all phases --------
      for (uint8_t phase_selector = 0U; phase_selector < num_phase_measurements; ++phase_selector) {
        float raw_mean = NAN;
        bool range_or_invalid = false;
        const bool got_sample = measurePhaseRaw(phase_selector, raw_mean, range_or_invalid);
        phase[freq_idx][phase_selector] = BIOZ_phase; // degree value set by phase selector routine

        if (!got_sample) {
          saw_invalid_or_range = true;
          impedance[freq_idx][phase_selector] = NAN;
          continue;
        }

        saw_valid = true;
        if (range_or_invalid) {
          saw_invalid_or_range = true;
        }

        const float a = fabsf(raw_mean);
        if (a > max_abs_raw) {
          max_abs_raw = a;
        }
        if (a > threshold_max) {
          any_above_target = true;
        } else if (a >= threshold_min) {
          any_in_target = true;
        }

        // Convert raw ADC code to Ohms for fitting.
        const float denom = 524288.0f * static_cast<float>(BIOZ_cgmag) * static_cast<float>(BIOZ_gain) * 1e-9f;
        impedance[freq_idx][phase_selector] = (denom > 0.0f)
                                              ? (raw_mean * static_cast<float>(V_ref)) / denom
                                              : NAN;

        LOGD("scanBIOZ: f=%.1fHz, phase=%.2fdeg, raw=%.1f, Z=%.2fohm, I=%ldnA",
             frequency[freq_idx], phase[freq_idx][phase_selector], raw_mean,
             impedance[freq_idx][phase_selector], static_cast<long>(current[freq_idx]));
      } // all phases -------

      if (!saw_valid) {
        LOGW("scanBIOZ: no valid samples at frequency %.1fHz.", frequency[freq_idx]);
        break;
      }

      if (max_abs_raw <= 0.0f) {
        LOGW("scanBIOZ: zero raw magnitude at frequency %.1fHz.", frequency[freq_idx]);
        break;
      }

      const bool reduce_current = saw_invalid_or_range || any_above_target;
      const bool increase_current = (!reduce_current && !any_in_target);

      if (reduce_current || increase_current) {
        int32_t desired_current = current[freq_idx];

        if (reduce_current) {
          // Any clipped/ranged/invalid phase => reduce current.
          desired_current = closestCurrent(static_cast<int32_t>(static_cast<float>(current[freq_idx]) * (adc_target / max_abs_raw)));
          if (desired_current >= current[freq_idx]) {
            desired_current = closestCurrent(current[freq_idx] / 2);
          }
        } else if (increase_current) {
          // Increase only when no phase reached the target range.
          desired_current = closestCurrent(static_cast<int32_t>(static_cast<float>(current[freq_idx]) * (adc_target / max_abs_raw)));
        }

        if (desired_current != current[freq_idx]) {
          current[freq_idx] = desired_current;
          continue; // remeasure all phases at adjusted current
        }
      }

      accepted = true;
      break;
    } // try again for current adjustment ---------

    if (!accepted) {
      LOGW("scanBIOZ: using best-effort data at frequency %.1fHz.", frequency[freq_idx]);
    }

    const ImpedanceModel result = fitImpedance(phase[freq_idx], impedance[freq_idx], num_phase_measurements);
    impedance_magnitude[freq_idx] = result.magnitude;
    impedance_phase[freq_idx] = result.phase;
    impedance_frequency[freq_idx] = frequency[freq_idx];
    _scanCurrentProfile[freq_idx] = current[freq_idx];
  }

  _scanCurrentProfileValid = true;
  _scanProfileFreqStart = freq_start_index;
  _scanProfileFreqEnd = freq_end_index;
  _scanProfileFast = fast;

}

int32_t MAX30001G::closestCurrent(int32_t input) {
/* Returns the current matching requested current with least error
 * Input current in nanoamps
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

float MAX30001G::impedancemodel(float phase_offset_deg, float magnitude_ohm, float phase_deg) {
  /*
    Model used by demodulation-based BIOZ measurement at a fixed injection frequency:
      measurement = |Z| * cos(phase - phase_offset)
    All phase arguments are in degrees.
  */
  const float deg_to_rad = static_cast<float>(PI) / 180.0f;
  return magnitude_ohm * cosf((phase_deg - phase_offset_deg) * deg_to_rad);
}

ImpedanceModel MAX30001G::fitImpedance(const float* phase_offsets, const float* measurements, int num_phase_measurements) {
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
 * The function returns the magnitude (|Z|) and phase (theta) of the impedance model via a least squares fitting approach.
 * The returned phase is in degrees.
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
    if ((phase_offsets == nullptr) || (measurements == nullptr) || (num_phase_measurements < 2)) {
      return {0.0f, 0.0f};
    }

    // Initialize sums for least squares
    float sum_cos_cos = 0.0f;
    float sum_sin_sin = 0.0f;
    float sum_cos_sin = 0.0f;
    float sum_measurements_cos = 0.0f;
    float sum_measurements_sin = 0.0f;
    int num_valid_points = 0;
    const float deg_to_rad = static_cast<float>(PI) / 180.0f;

    for (int i = 0; i < num_phase_measurements; ++i) {
      if (isfinite(measurements[i]) && isfinite(phase_offsets[i])) {
        num_valid_points++;
        const float phi = phase_offsets[i] * deg_to_rad;
        const float cos_theta = cosf(phi);
        const float sin_theta = sinf(phi);
        sum_cos_cos += cos_theta * cos_theta;
        sum_sin_sin += sin_theta * sin_theta;
        sum_cos_sin += cos_theta * sin_theta;
        sum_measurements_cos += measurements[i] * cos_theta;
        sum_measurements_sin += measurements[i] * sin_theta;
      }
    }

    if (num_valid_points < 2) {
      return {0.0f, 0.0f}; // Need at least two valid points.
    }

    // Compute the determinant
    const float denom = sum_cos_cos * sum_sin_sin - sum_cos_sin * sum_cos_sin;
    if (fabsf(denom) < 1.0e-6f) {
      // Handle singularity (e.g., insufficient variation in phase offsets)
      return {0.0f, 0.0f};
    }

    // Solve for A and B
    const float A = (sum_measurements_cos * sum_sin_sin - sum_measurements_sin * sum_cos_sin) / denom;
    const float B = (sum_measurements_sin * sum_cos_cos - sum_measurements_cos * sum_cos_sin) / denom;

    // Compute magnitude and phase
    const float magnitude = sqrtf(A * A + B * B);
    const float phase_deg = atan2f(B, A) * (180.0f / static_cast<float>(PI));

    return {magnitude, phase_deg};
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

