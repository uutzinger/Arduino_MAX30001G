/******************************************************************************************************/
// Calibration
/******************************************************************************************************/
#ifndef MAX30001G_CALIBRATION_H
#define MAX30001G_CALIBRATION_H

#include <stdint.h>

class MAX30001G {
    public:
    void setDefaultNoCalibration(void);   // off
    void setDefaultECGCalibration(void);  // 1 Hz
    void setDefaultBIOZCalibration(void); // 1Hz
    /*
      default signal settings are unipolar, 0.5mV, 50% duty cycle
    */
    
    private:
    void setCalibration(bool enableECGCal, bool enableBIOZCal, bool unipolar, bool cal_vmag, uint8_t freq, uint8_t dutycycle);
    /*
      - enableECGCal: Enable or disable ECG calibration
      - enableBIOZCal: Enable or disable BIOZ calibration
      Signal Settings:
      - unipolar: Set to true for unipolar, false for bipolar test signals
      - cal_vmag: Set the calibration voltage magnitude: true for ±0.5 mV, false for ±0.25 mV (CAL_VMAG).
      - freq: Calibration frequency (value = 0..7) will result in FMSTR / (2^(7+value*2)) Hz or approx 250..0.01 Hz
      - dutycycle: Duty cycle percentage (default 50%)
    */
};

#endif