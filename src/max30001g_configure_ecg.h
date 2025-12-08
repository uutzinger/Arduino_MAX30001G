/******************************************************************************************************/
// Configure ECG
/******************************************************************************************************/
#ifndef MAX30001G_CONFIG_ECG_H
#define MAX30001G_CONFIG_ECG_H

#include <stdint.h>

class MAX30001G {
    private:
        void setECGSamplingRate(uint8_t  ECG);
        // 0:125, 1:256, 2:512sps
        void setECGfilter(uint8_t lpf, uint8_t hpf);
        // 0:bypass, 1:40, 2:200 3:150Hz LPF
        // 0:bypass, 1:0.5Hz HPF
        void setECGgain(uint8_t gain);
        // 0:  20V/V
        // 1:  40V/V
        // 2:  80V/V
        // 3: 160V/V 
        void setECGLeadPolarity(bool inverted, bool open);
        // 0: normal, 1: inverted
        // 0: closed to measure samples, 1: open for calibration 
        void setECGAutoRecovery(int threshold_voltage);
        // should be set to <= 98% of the V_ref / ECG_gain
        // 0.98 * 1V / 80V/V * 1000 mV/V = 12.25 mV
        void setECGNormalRecovery();
        // recovery disabled
        void startECGManualRecovery();
        // call after saturation detected with your own program
        void stopECGManualRecovery();
        // manual start needs manual stop
};
#endif
