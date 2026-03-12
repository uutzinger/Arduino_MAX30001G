/******************************************************************************************************/
// Configure BIOZ
/******************************************************************************************************/
#include <math.h>

#include "logger.h"                // Logging
#include "max30001g.h"

namespace {

struct BiozFreqEntry {
  uint32_t hz[4]; // FMSTR[1:0] = 00,01,10,11
};

// Table 42 (FCGEN 0000..1010; 101x/11xx collapse to 125/128Hz bucket).
constexpr BiozFreqEntry kBiozFreqTable[11] = {
  {{131072, 128000, 128000, 127872}}, // 0000
  {{ 81920,  80000,  80000,  81920}}, // 0001
  {{ 40960,  40000,  40000,  40960}}, // 0010
  {{ 18204,  17780,  17780,  18204}}, // 0011
  {{  8192,   8000,   8000,   7992}}, // 0100
  {{  4096,   4000,   4000,   3996}}, // 0101
  {{  2048,   2000,   2000,   1998}}, // 0110
  {{  1024,   1000,   1000,    999}}, // 0111
  {{   512,    500,    500,    500}}, // 1000
  {{   256,    250,    250,    250}}, // 1001
  {{   128,    125,    125,    125}}, // 1010 (also used for 101x/11xx class)
};

// CGMAG code to current magnitude in nA for high-current range.
constexpr uint32_t kHighCurrentNA[8] = {
  0, 8000, 16000, 32000, 48000, 64000, 80000, 96000
};

struct LowCurrentOption {
  uint16_t current_nA;
  uint8_t lc2x;
  uint8_t cmag_lc;
};

// Valid low-current combinations from Table 45.
constexpr LowCurrentOption kLowCurrentOptions[] = {
  {  55, 0, 0b0001},
  { 110, 0, 0b0010},
  { 220, 0, 0b0011},
  { 330, 0, 0b0100},
  { 440, 0, 0b0101},
  { 550, 0, 0b0110},
  { 660, 1, 0b0100},
  { 880, 1, 0b0101},
  {1100, 1, 0b0110},
};

struct CmresOption {
  uint8_t code;
  float mohm;
  uint32_t kohm;
};

// Valid CMRES selections (skip OFF codes).
constexpr CmresOption kCmresOptions[] = {
  {0b1111,   5.0f,   5000},
  {0b1110,   5.5f,   5500},
  {0b1101,   6.5f,   6500},
  {0b1100,   7.5f,   7500},
  {0b1011,  10.0f,  10000},
  {0b1010,  12.5f,  12500},
  {0b1001,  20.0f,  20000},
  {0b1000,  40.0f,  40000},
  {0b0111,  80.0f,  80000},
  {0b0101, 100.0f, 100000},
  {0b0011, 160.0f, 160000},
  {0b0001, 320.0f, 320000},
};

uint8_t fmstrIndex(uint8_t fmstr_bits) {
  return static_cast<uint8_t>(fmstr_bits & 0x03U);
}

uint32_t frequencyFromFcgen(uint8_t fcgen, uint8_t fmstr_idx) {
  if (fcgen > 0x0A) {
    fcgen = 0x0A;
  }
  if (fmstr_idx > 3U) {
    fmstr_idx = 0U;
  }
  return kBiozFreqTable[fcgen].hz[fmstr_idx];
}

uint8_t maxCgmagForFcgen(uint8_t fcgen) {
  if (fcgen <= 0b0011) {
    return 0b0111; // all 8uA..96uA
  }
  if (fcgen == 0b0100) {
    return 0b0110; // up to 80uA
  }
  if (fcgen == 0b0101) {
    return 0b0011; // up to 32uA
  }
  if (fcgen == 0b0110) {
    return 0b0010; // up to 16uA
  }
  return 0b0001;   // up to 8uA for 0111,1000,1001,101x,11xx
}

uint8_t nearestFcgenForFrequency(uint32_t requested_hz, uint8_t fmstr_idx) {
  uint8_t best_idx = 0;
  uint32_t best_err = 0xFFFFFFFFUL;

  for (uint8_t idx = 0; idx <= 0x0A; ++idx) {
    const uint32_t f = frequencyFromFcgen(idx, fmstr_idx);
    const uint32_t err = (f > requested_hz) ? (f - requested_hz) : (requested_hz - f);
    if (err < best_err) {
      best_err = err;
      best_idx = idx;
    }
  }

  return best_idx;
}

uint8_t nearestHighCgmag(uint32_t requested_nA, uint8_t max_code) {
  if (max_code < 1U) {
    return 0;
  }
  if (max_code > 7U) {
    max_code = 7U;
  }

  uint8_t best_code = 1U;
  uint32_t best_err = 0xFFFFFFFFUL;
  for (uint8_t code = 1U; code <= max_code; ++code) {
    const uint32_t v = kHighCurrentNA[code];
    const uint32_t err = (v > requested_nA) ? (v - requested_nA) : (requested_nA - v);
    if (err < best_err) {
      best_err = err;
      best_code = code;
    }
  }
  return best_code;
}

LowCurrentOption nearestLowCurrent(uint32_t requested_nA) {
  LowCurrentOption best = kLowCurrentOptions[0];
  uint32_t best_err = 0xFFFFFFFFUL;
  for (const LowCurrentOption &opt : kLowCurrentOptions) {
    const uint32_t err = (opt.current_nA > requested_nA) ? (opt.current_nA - requested_nA) : (requested_nA - opt.current_nA);
    if (err < best_err) {
      best_err = err;
      best = opt;
    }
  }
  return best;
}

CmresOption selectCmresForCurrent(uint32_t drive_current_nA) {
  // Datasheet Table 45 guidance: CMRES(Mohm) ~= 5 / I(uA).
  const float desired_mohm = 5000.0f / static_cast<float>(drive_current_nA);

  CmresOption best = kCmresOptions[0];
  float best_err = fabsf(best.mohm - desired_mohm);
  for (const CmresOption &opt : kCmresOptions) {
    const float err = fabsf(opt.mohm - desired_mohm);
    if (err < best_err) {
      best_err = err;
      best = opt;
    }
  }
  return best;
}

void phaseResolutionFromFcgen(uint8_t fcgen, float &step_deg, uint8_t &max_selector) {
  if (fcgen == 0b0000) {
    step_deg = 45.0f;
    max_selector = 3U;
  } else if (fcgen == 0b0001) {
    step_deg = 22.5f;
    max_selector = 7U;
  } else {
    step_deg = 11.25f;
    max_selector = 15U;
  }
}

} // namespace

void MAX30001G::setBIOZSamplingRate(uint8_t speed_select) {
  /*
    Set BIOZ sample-rate selector:
      speed_select=0 -> low-rate setting 25-32 Hz* default
      speed_select=1 -> high-rate setting 50-64 Hz
    Actual SPS depend on FMSTR and are updated in global BIOZ_samplingRate.
  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_gen.all  = readRegister24(MAX30001_CNFG_GEN);

  if (speed_select == 0U) {
    cnfg_bioz.bit.rate = 1U; // low rate
  } else if (speed_select == 1U) {
    cnfg_bioz.bit.rate = 0U; // high rate
  } else {
    LOGE("Invalid BIOZ speed selector: %u (valid: 0 low, 1 high).", speed_select);
    return;
  }

  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  updateGlobalBIOZ_samplingRate();
  LOGI("BIOZ sampling rate set to %.2f sps", BIOZ_samplingRate);
}

void MAX30001G::setBIOZgain(uint8_t gain, bool lowNoise) {
  /*
    BIOZ gain:
      0=10V/V, * default
      1=20V/V, 
      2=40V/V, 
      3=80V/V
    INA mode:
      lowNoise=false -> low-power * default
      lowNoise=true  -> low-noise
  */

  if (gain > 3U) {
    LOGE("Invalid BIOZ gain selector: %u (valid 0..3).", gain);
    return;
  }

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz.bit.gain = gain;
  cnfg_bioz.bit.ln_bioz = lowNoise ? 1U : 0U;
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

  switch (gain) {
    case 0:  BIOZ_gain = 10; break;
    case 1:  BIOZ_gain = 20; break;
    case 2:  BIOZ_gain = 40; break;
    case 3:  BIOZ_gain = 80; break;
    default: BIOZ_gain = 10; break;
  }

  LOGI("BIOZ gain set to %d V/V (%s mode)", BIOZ_gain, lowNoise ? "low-noise" : "low-power");
}

void MAX30001G::setBIOZModulationFrequencyByFrequency(uint16_t frequency_hz) {
  /*
    Select nearest valid BIOZ FCGEN option from a target frequency in Hz.
    FCGEN is discrete (Table 42), so this helper maps by nearest value.


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
        0          OFF               OFF   this is not allowed

    */

  if (frequency_hz == 0U) {
    LOGE("BIOZ modulation frequency must be > 0 Hz.");
    return;
  }

  cnfg_gen.all = readRegister24(MAX30001_CNFG_GEN);
  const uint8_t idx = nearestFcgenForFrequency(frequency_hz, fmstrIndex(cnfg_gen.bit.fmstr));
  setBIOZModulationFrequencybyIndex(idx);
}

void MAX30001G::setBIOZModulationFrequencybyIndex(uint8_t frequency_selector) {
  /*
    Set BIOZ modulation frequency by FCGEN code:
      0..10 map to ~128kHz..125Hz (Table 42).
  */

  if (frequency_selector > 0x0AU) {
    LOGW("FCGEN selector %u invalid; clamping to 10 (~125Hz).", frequency_selector);
    frequency_selector = 0x0AU;
  }

  cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);
  cnfg_gen.all     = readRegister24(MAX30001_CNFG_GEN);

  cnfg_bioz.bit.fcgen = frequency_selector;
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

  BIOZ_frequency = static_cast<float>(frequencyFromFcgen(frequency_selector, fmstrIndex(cnfg_gen.bit.fmstr)));

  const uint8_t max_code = maxCgmagForFcgen(frequency_selector);
  const uint32_t max_current = kHighCurrentNA[max_code];
  if ((cnfg_bioz_lc.bit.hi_lob == 1U) && (BIOZ_cgmag > static_cast<int>(max_current))) {
    LOGW("Current setting %d nA exceeds FCGEN limit (%lu nA). Re-run setBIOZmag().",
         BIOZ_cgmag, static_cast<unsigned long>(max_current));
  }

  LOGI("BIOZ modulation frequency set to %.0f Hz (FCGEN=%u)", BIOZ_frequency, frequency_selector);
}


void MAX30001G::setBIOZmag(uint32_t current_nA) {
  /*
    Set BIOZ current magnitude in nA.
    - 0 disables the drive current (CGMAG=000).
    - 55..1100nA uses low-current mode (BIOZ_HI_LOB=0, CGMAG=001 + LC controls).
    - >=8uA uses high-current mode (BIOZ_HI_LOB=1, CGMAG=001..111), constrained by FCGEN.

    Common-mode feedback handling:
    - High-current mode: enable EXT_RBIAS for external resistive bias.
    - Low-current mode: select CMRES per Table 45 guidance.


    high current 
    --------------------------------------------------------------------
     cmag >= 001
     and cnfg_bioz_lc.bit.hi_lob == 1 
  
     BioZ Current Generator Magnitude: cnfg_bioz.cgmag
     000 = Off (DRVP and DRVN floating, Current Generators Off)
     001 = 8µA (also use this setting when BIOZ_HI_LOB = 0)
     010 = 16µA
     011 = 32µA
     100 = 48µA
     101 = 64µA
     110 = 80µA
     111 = 96µA

     low current
     ---------------------------------------------------------------------
     cmag == 001 
     and cnfg_bioz_lc.bit.hi_lob == 0 
     cnfg_bioz_lc.bit.lc2x == 0 55-1100nA
     cnfg_bioz_lc.bit.lc2x == 1 110-1100nA
     set the common mode resistance as recommended in BIOZ_CMRES.
  
     BIOZ Low Current Generator Magnitude: cnfg_bioz_lc.bit.cmag_lc
             LC2X = 0 LC2X = 1
     0000    0        0
     0001    55nA     110nA
     0010    110nA    220nA
     0011    220nA    440nA
     0100    330nA    660nA
     0101    440nA    880nA
     0110    550nA    1100nA

    Following currents can be achieved with selected momdulation frequency

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

    Adjust Common Mode Current Feedback 
    -----------------------------------
    
    There is feedback option for high current mode
    There is feedback option for low current mode
    BIOZ_cmag is in [nA]
    
    High Current 8000 - 96000 nA
    ----------------------------
    active and passive(resistive) current mode feedback is vailable (BMUX_CG_MODE)
    
    Options for current feedback:
    Passive:
      1) external 324kOhm resistor on R_BIAS to A_GND, EXT_RBIAS = on
        MediBrick board has resistor in place
      2) BISTR [0] 27k, [1] 108k, [2] 487k, [3] 1029k, EN_BISTR = on
        This is usually used for calibration
    Active:
      3) no external no internal, uses active
    
    Low Current 55 - 1100 nA
    ------------------------
    only passive(resistive) common mode current feedback 
    
    cnfg_bioz_lc.cmres needs to be matched to current magnitude
    It should be approximately  5000 / (drive current [µA]) [kΩ]
    available are: 5, 5.5, 6.5, 7.5, 10, 12.5, 20, 40, 80, 100, 160, 320, MOhm

  */

  cnfg_bioz.all    = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);

  if (current_nA == 0U) {
    cnfg_bioz.bit.cgmag = 0b000;
    BIOZ_cgmag = 0;
    writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
    LOGI("BIOZ current generator disabled.");
    return;
  }

  if (current_nA >= 8000U) {
    // High-current mode with FCGEN-dependent CGMAG ceiling (Table 43).
    const uint8_t max_code = maxCgmagForFcgen(cnfg_bioz.bit.fcgen);
    const uint8_t chosen = nearestHighCgmag(current_nA, max_code);
    const uint32_t limited = kHighCurrentNA[chosen];

    if (current_nA > kHighCurrentNA[max_code]) {
      LOGW("Requested current %lu nA exceeds FCGEN limit; using %lu nA.",
           static_cast<unsigned long>(current_nA),
           static_cast<unsigned long>(limited));
    }

    cnfg_bioz_lc.bit.hi_lob = 1U;
    cnfg_bioz.bit.cgmag = chosen;
    cnfg_bioz.bit.ext_rbias = 1U;

    BIOZ_cgmag = static_cast<int>(limited);
    BIOZ_cmres = 0U;
  } else {
    // Low-current mode (Table 45).
    const LowCurrentOption opt = nearestLowCurrent(current_nA);
    const CmresOption cmres = selectCmresForCurrent(opt.current_nA);

    // Datasheet Table 37: BMUX_CG_MODE must be 00 in low-current mode.
    cnfg_bmux.all = readRegister24(MAX30001_CNFG_BMUX);
    if (cnfg_bmux.bit.cg_mode != 0U) {
      LOGW("Low-current BIOZ requires BMUX_CG_MODE=0. Forcing mode 0.");
      cnfg_bmux.bit.cg_mode = 0U;
      writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
    }

    cnfg_bioz_lc.bit.hi_lob = 0U;
    cnfg_bioz.bit.cgmag = 0b001;        // required in low-current range
    cnfg_bioz_lc.bit.lc2x = opt.lc2x;
    cnfg_bioz_lc.bit.cmag_lc = opt.cmag_lc;
    cnfg_bioz_lc.bit.cmres = cmres.code;
    cnfg_bioz.bit.ext_rbias = 0U;

    BIOZ_cgmag = opt.current_nA;
    BIOZ_cmres = cmres.kohm;
  }

  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  writeRegister(MAX30001_CNFG_BIOZ_LC, cnfg_bioz_lc.all);

  LOGI("BIOZ drive current set to %d nA", BIOZ_cgmag);
  if (cnfg_bioz_lc.bit.hi_lob == 0U) {
    LOGI("BIOZ CMRES set to %lu kOhm (low-current mode)", static_cast<unsigned long>(BIOZ_cmres));
  } else {
    LOGI("BIOZ using external RBIAS feedback (high-current mode)");
  }
}


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
  * 
  **/

void MAX30001G::setBIOZPhaseOffsetbyPhase(uint16_t frequency_hz, float phase_deg) {
  // Convenience API for "value-based" scan workflows: select frequency then phase.
  setBIOZModulationFrequencyByFrequency(frequency_hz);
  setBIOZPhaseOffsetbyPhase(phase_deg);
}

void MAX30001G::setBIOZPhaseOffsetbyPhase(float phase_deg) {
  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);

  float step_deg = 11.25f;
  uint8_t max_selector = 15U;
  phaseResolutionFromFcgen(cnfg_bioz.bit.fcgen, step_deg, max_selector);

  const float max_phase = static_cast<float>(max_selector) * step_deg;
  if (phase_deg < 0.0f) {
    phase_deg = 0.0f;
  }
  if (phase_deg > max_phase) {
    phase_deg = max_phase;
    LOGW("Phase limited to %.2f degrees for current FCGEN.", max_phase);
  }

  uint8_t selector = static_cast<uint8_t>((phase_deg / step_deg) + 0.5f); // nearest code
  if (selector > max_selector) {
    selector = max_selector;
  }

  setBIOZPhaseOffsetbyIndex(selector);
}

void MAX30001G::setBIOZPhaseOffsetbyIndex(uint8_t selector) {
  /*
    FCGEN-dependent PHOFF resolution:
      FCGEN=0000  -> PHOFF[3:2]*45deg    (0..135deg)
      FCGEN=0001  -> PHOFF[3:1]*22.5deg  (0..157.5deg)
      FCGEN>=0010 -> PHOFF[3:0]*11.25deg (0..168.75deg)
  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);

  float step_deg = 11.25f;
  uint8_t max_selector = 15U;
  phaseResolutionFromFcgen(cnfg_bioz.bit.fcgen, step_deg, max_selector);

  if (selector > max_selector) {
    LOGW("Phase selector %u invalid for current FCGEN; clamping to %u.", selector, max_selector);
    selector = max_selector;
  }

  cnfg_bioz.bit.phoff = selector;
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);

  BIOZ_phase = static_cast<float>(selector) * step_deg;
  LOGI("BIOZ phase set to %.2f degrees", BIOZ_phase);
}

void MAX30001G::setBIOZfilter(uint8_t ahpf, uint8_t lpf, uint8_t hpf) {
  /*
    BIOZ_AHPF:
      0:60Hz 
      1:150Hz 
      2:500Hz * default
      3:1000Hz 
      4:2000Hz 
      5:4000Hz 
      6/7:bypass
    BIOZ_DLPF:
      0:bypass 
      1:4Hz * default
      2:8Hz 
      3:16Hz (some rate/fmstr combinations internally map to 4Hz)
    BIOZ_DHPF:
      0:bypass * default
      1:0.05Hz 
      2/3:0.5Hz
  */

  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);

  // Digital HPF
  if (hpf == 0U) {
    cnfg_bioz.bit.dhpf = 0b00;
    BIOZ_dhpf = 0.0f;
  } else if (hpf == 1U) {
    cnfg_bioz.bit.dhpf = 0b01;
    BIOZ_dhpf = 0.05f;
  } else {
    cnfg_bioz.bit.dhpf = 0b10; // 1x => 0.5Hz
    BIOZ_dhpf = 0.5f;
  }

  // Digital LPF
  if (lpf > 3U) {
    LOGW("Invalid BIOZ DLPF selector %u; clamping to 3.", lpf);
    lpf = 3U;
  }
  cnfg_bioz.bit.dlpf = lpf;

  // Analog HPF
  if (ahpf <= 5U) {
    cnfg_bioz.bit.ahpf = ahpf;
    switch (ahpf) {
      case 0: BIOZ_ahpf = 60.0f; break;
      case 1: BIOZ_ahpf = 150.0f; break;
      case 2: BIOZ_ahpf = 500.0f; break;
      case 3: BIOZ_ahpf = 1000.0f; break;
      case 4: BIOZ_ahpf = 2000.0f; break;
      case 5: BIOZ_ahpf = 4000.0f; break;
      default: BIOZ_ahpf = 0.0f; break;
    }
  } else {
    cnfg_bioz.bit.ahpf = 0b110; // bypass
    BIOZ_ahpf = 0.0f;
  }

  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  updateGlobalBIOZ_dlpf(); // keeps effective LPF consistent with FMSTR/rate rules

  LOGI("BIOZ filter set: AHPF=%.2f Hz, DLPF=%.3f Hz, DHPF=%.2f Hz",
       BIOZ_ahpf, BIOZ_dlpf, BIOZ_dhpf);
}

void MAX30001G::setBIOZmodulation(uint8_t mode) {
  /*
    BMUX_CG_MODE:
      00 = Unchopped sources with LPF * default
      01 = Chopped sources without LPF
      10 = Chopped sources with LPF
      11 = Chopped sources with resistive CM (not recommended >32uA)

    Datasheet note: use mode 00 if BIOZ_HI_LOB=0 (low-current range).

    0 = (higher noise, excellent 50/60Hz rejection, recommended for ECG, BioZ applications)
    1 = (low noise, no 50/60Hz rejection, recommended for BioZ applications with digital LPF, possibly battery powered ECG, BioZ applications)
    2 = (low noise, excellent 50/60Hz rejection)
    3 = (not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance)
  */

  if (mode > 3U) {
    LOGE("Invalid BIOZ modulation mode %u (valid 0..3).", mode);
    return;
  }

  cnfg_bmux.all    = readRegister24(MAX30001_CNFG_BMUX);
  cnfg_bioz_lc.all = readRegister24(MAX30001_CNFG_BIOZ_LC);

  if ((cnfg_bioz_lc.bit.hi_lob == 0U) && (mode != 0U)) {
    LOGW("Low-current mode requires BMUX_CG_MODE=0. Forcing mode 0.");
    mode = 0U;
  }

  cnfg_bmux.bit.cg_mode = mode;
  writeRegister(MAX30001_CNFG_BMUX, cnfg_bmux.all);
  LOGI("BIOZ modulation mode set to %u", mode);
}

void MAX30001G::setBIOZCurrentMonitor(bool enable) {
  cnfg_bioz.all = readRegister24(MAX30001_CNFG_BIOZ);
  cnfg_bioz.bit.cgmon = enable ? 1U : 0U;
  writeRegister(MAX30001_CNFG_BIOZ, cnfg_bioz.all);
  LOGI("BIOZ current monitor %s", enable ? "enabled" : "disabled");
}
