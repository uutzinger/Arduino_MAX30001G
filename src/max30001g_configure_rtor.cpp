/******************************************************************************************************/
// Configure RtoR
/******************************************************************************************************/
#include "logger.h"      // Logging 
#include "max30001g_globals.h"
#include "max30001g_defs.h" // MAX30001G
#include "max30001g_comm.h" // SPI communication
#include "max30001g_configure_rtor.h"

/******************************************************************************************************/

void MAX30001G::setDefaultRtoR() {
  /*
    Sets default values for R-to-R detection.
    - enable: true
    - ptsf: 0b0011   (4) (4/16) 
      This is the fraction of the Peak Average value used in the Threshold computation.
    - ptsf: 0b0110   (6) is also a common value for the threshold.
    - pavg: 0b10     (8) default but 2 might also be used, how many past peak values are used to update peak thershold
      This is the weighting factor for the current R-to-R peak observation vs. past peak observations when determining peak thresholds.
    - gain: 0b1111   (Auto-Scale)
    - wndw: 0b011    12 (96ms)
    - hoff: 0b100000 (32)
    - ravg: 0b10     (8)
    - rhsf: 0b100    (4/8)
  */
  setRtoR(
      true,      // enable: Enable R-to-R detection
      0b0011,    // ptsf: Peak Threshold Scaling Factor (4/16)
      0b10,      // pavg: Peak Averaging Weight Factor (8)
      0b1111,    // gain: Auto-Scale
      0b011,     // wndw: Window width (96ms)
      0b100000,  // hoff: Minimum Hold Off
      0b10,      // ravg: Interval Averaging Weight Factor (8)
      0b100      // rhsf: Interval Hold Off Scaling Factor (4/8)
  );
}

void MAX30001G::setDefaultNoRtoR() {
  /*
    Sets default values for R-to-R detection.
    - enable: false
    - ptsf: 0b0011   (4) (4/16)
    - ptsf: 0b0110   (6) is also a common value for the threshold.
    - pavg: 0b10     (8) default but (2) might also be used.
    - gain: 0b1111   (Auto-Scale)
    - wndw: 0b011    12 (96ms)
    - hoff: 0b100000 (32)
    - ravg: 0b10     (8)
    - rhsf: 0b100    (4/8)
  */
  setRtoR(
      false,      // enable: Enable R-to-R detection
      0b0011,    // ptsf: Peak Threshold Scaling Factor (4/16)
      0b10,      // pavg: Peak Averaging Weight Factor (8)
      0b1111,    // gain: Auto-Scale
      0b011,     // wndw: Window width (96ms)
      0b100000,  // hoff: Minimum Hold Off
      0b10,      // ravg: Interval Averaging Weight Factor (8)
      0b100      // rhsf: Interval Hold Off Scaling Factor (4/8)
  );
}

void MAX30001G::setRtoR(bool enable, uint8_t ptsf, uint8_t pavg, uint8_t gain, uint8_t wndw, uint8_t hoff, uint8_t ravg, uint8_t rhsf) {
/*
  
  Configures R-to-R detection for the ECG channel with detailed settings.

  Timing resolution is about 8ms
  Delay is about 105ms
  RRINT interrupt is triggered when QRS waveforms is detected
  RTOR register holds time between R waves in RTOR resolution units
  
  Parameters:
  - ptsf: Peak Threshold Scaling Factor    (0 to 15).
  - pavg: Peak Averaging Weight Factor     (0 to  3).
  - gain: Gain setting for RtoR            (0 to 15).
  - wndw: Width of the averaging window    (0 to 15).
  - hoff: Minimum Hold Off                 (0 to 63).
  - ravg: Interval Averaging Weight Factor (0 to  3).
  - rhsf: Interval Hold Off Scaling Factor (0 to  7).
  
  For example:
  - enable: true
  - ptsf: A value of       4 (1/16) might be typical for the threshold.
  - pavg: A value of    0b10 (8) for some averaging.
  - gain: A value of  0b1111 for autoscale.
  - wndw: A value of      12 (96ms) for the window width.
  - hoff: A value of      32 for the hold off.
  - ravg: A value of    0b10 (8) for scaling factor.
  - rhsf: A value of   0b100 (4/8) for hold off scaling factor.
  
  cnfg_rtor1.bit.wndw
    This is the width of the averaging window, which adjusts the algorithm sensitivity 
    to the width of the QRS complex.
    R-to-R Window Averaging (Window Width = WNDW[3:0]*8ms) 
      0000 =  6 x RTOR_RES
      0001 =  8 x RTOR_RES
      0010 = 10 x RTOR_RES
      0011 = 12 x RTOR_RES  (default = 96ms)
      0100 = 14 x RTOR_RES
      0101 = 16 x RTOR_RES
      0110 = 18 x RTOR_RES
      0111 = 20 x RTOR_RES
      1000 = 22 x RTOR_RES
      1001 = 24 x RTOR_RES
      1010 = 26 x RTOR_RES
      1011 = 28 x RTOR_RES
      1100 = Reserved. Do not use.
      1101 = Reserved. Do not use.
      1110 = Reserved. Do not use.
      1111 = Reserved. Do not use.
    The value of RTOR_RES is approximately 8ms, updateGlobalRTOR_RES() computes it.
  
  cnfg_rtor1.bit.gain
    R-to-R Gain (where Gain = 2^RGAIN[3:0], plus an auto-scale option). 
    This is used to maximize the dynamic range of the algorithm.
    0000 =     1
    0001 =     2
    0010 =     4
    0011 =     8
    0100 =    16
    0101 =    32
    0110 =    64 (initial for auto)
    0111 =   128
    1000 =   256
    1001 =   512
    1010 =  1024
    1011 =  2048
    1100 =  4096
    1101 =  8192
    1110 = 16384
    1111 = Auto-Scale (default)
    In Auto-Scale mode, the initial gain is set to 64.
  
  cnfg_rtor1.bit.pavg
    R-to-R Peak Averaging Weight Factor
      This is the weighting factor for the current R-to-R peak observation vs. past peak
      observations when determining peak thresholds. Lower numbers weight current peaks more heavily.
      00 = 2
      01 = 4
      10 = 8 (default)
      11 = 16
      Peak_Average(n) = [Peak(n) + (PAVG-1) x Peak_Average(n-1)] / PAVG
  
  cnfg_rtor1.bit.ptsf
    R-to-R Peak Threshold Scaling Factor
      This is the fraction of the Peak Average value used in the Threshold computation. 
      Values of 1/16 to 16/16 are selected by (PTSF[3:0]+1)/16. 
      default is 4/16
  
  cnfg_rtor2.bit.hoff
    R-to-R Minimum Hold Off
      This sets the absolute minimum interval used for the static portion of the Hold Off criteria. 
      Values of 0 to 63 are supported.
      default is 32 or 0b100000
      
      tHOLD_OFF_MIN = HOFF[5:0] * tRTOR, where tRTOR is approximately 8ms, 
      as determined by FMSTR[1:0] in the CNFG_GEN register. (representing approximately ¼ second).
      The R-to-R Hold Off qualification interval is
        tHold_Off = MAX(tHold_Off_Min, tHold_Off_Dyn).
  
  cnfg_rtor2.bit.ravg
    R-to-R Interval Averaging Weight Factor
      This is the weighting factor for the current R-to-R interval observation vs. the past interval
      observations when determining dynamic holdoff criteria. Lower numbers weight current intervals
      more heavily.
      00 = 2
      01 = 4
      10 = 8 (default)
      11 = 16
      Interval_Average(n) = [Interval(n) + (RAVG-1) x Interval_Average(n-1)] / RAVG.
  
  cnfg_rtor2.bit.rhsf
    R-to-R Interval Hold Off Scaling Factor
      This is the fraction of the R-to-R average interval used for the dynamic portion of the holdoff criteria (tHOLD_OFFDYN).
      Values of 0/8 to 7/8 are selected by RTOR_RHSF[3:0]/8, default is 4/8.
      If 000 (0/8) is selected, then no dynamic factor is used and the holdoff criteria is determined by
      HOFF[5:0] only.
 */

  // Ensure the values are within the allowed range
  if (ptsf > 15)  ptsf = 15; // Peak Threshold Scaling Factor    (0 to 15).
  if (pavg > 3)   pavg =  3; // Peak Averaging Weight Factor     (0 to  3).
  if (gain > 15)  gain = 15; // Gain setting for RtoR            (0 to 15).
  if (wndw > 15)  wndw = 15; // Width of the averaging window    (0 to 15).
  if (hoff > 63)  hoff = 63; // Minimum Hold Off                 (0 to 63).
  if (ravg > 3)   ravg =  3; // Interval Averaging Weight Factor (0 to  3).
  if (rhsf > 7)   rhsf =  7; // Interval Hold Off Scaling Factor (0 to  7).

  // Read the current rtor1 settings
  cnfg_rtor1.all = readRegister24(MAX30001_CNFG_RTOR1);

  // Apply RtoR detection
  cnfg_rtor1.bit.en_rtor = enable; // Enable or disable RtoR detection

  if (enable) {
   
    // Read the current rtor2 settings
    cnfg_rtor2.all = readRegister24(MAX30001_CNFG_RTOR2);

    // Configure RtoR detection parameters in CNFG_RTOR1
    cnfg_rtor1.bit.ptsf = ptsf;      // Peak Threshold Scaling Factor
    cnfg_rtor1.bit.pavg = pavg;      // Peak Averaging Weight Factor
    cnfg_rtor1.bit.gain = gain;      // R-to-R Gain setting
    cnfg_rtor1.bit.wndw = wndw;      // Width of the averaging window
    
    // Configure RtoR detection parameters in CNFG_RTOR2
    cnfg_rtor2.bit.hoff = hoff;      // Minimum Hold Off
    cnfg_rtor2.bit.ravg = ravg;      // Interval Averaging Weight Factor
    cnfg_rtor2.bit.rhsf = rhsf;      // Interval Hold Off Scaling Factor

    // Write the updated rtor2 settings back to the register
    writeRegister(MAX30001_CNFG_RTOR2, cnfg_rtor2.all);

  }
  
  // Write the updated rtor1 settings back to the register
  writeRegister(MAX30001_CNFG_RTOR1, cnfg_rtor1.all);
}
  