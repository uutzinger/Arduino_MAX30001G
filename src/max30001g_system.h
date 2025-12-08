#ifndef MAX30001G_SYSTEM_H
#define MAX30001G_SYSTEM_H

#include <stdint.h>

class MAX30001G {
    private:
        void synch(void);           // Synchronize the device
        void swReset(void);         // Reset Device

        void setFMSTR(uint8_t fmstr); // Set Base Clock Frequency
        /*
        Set Base Clock Frequency. 
        This will affect the timing of the ECG and BIOZ measurements.

        FMSTR[1:0] Mapping:
        0 0b00 = 32768 Hz = FLCK
        1 0b01 = 32000 Hz = FLCK*625/640
        2 0b10 = 32000 Hz = FLCK*625/640
        3 0b11 = 31968.78 Hz = FLCK*640/656

        Will execute all the routines below to update global system variables

        Updates
        fmstr              // Hz
        tres               // µs
        ECG_progression    // per second
        RtoR_resolution    // ms
        RtoR_delay         // ms
        CAL_resolution     // µs      

        as well as:

        ECG_samplingRate   // per second
        BIOZ_samplingRate  // per second
        CAL_fcal           // Hz
        ECG_latency        // ms
        BIOZ_frequency     // Hz
        BIOZ_lpf           // Hz
        ECG_lpf            // Hz
        RCAL_freq          // Hz
        */

        // Read registers and update global variables based on register values
        void updateGlobalECG_samplingRate(void);    // ECG Sampling Rate in Hz
        void updateGlobalBIOZ_samplingRate(void);   // BIOZ Sampling Rate in Hz
        void updateGlobalCAL_fcal(void);            // CAL_fcal frequency calibration
        void updateGlobalECG_latency(void);         // ECG latency
        void updateGlobalBIOZ_frequency(void);      // BIOZ frequency
        void updateGlobalBIOZ_test_impedance(void); // Internal test impedance
        void updateGlobalECG_lpf(void);             // ECG low pass filter
        void updateGlobalBIOZ_dlpf(void);            // BIOZ low pass filter
        void updateGlobalRCAL_freq(void);           // RCAL frequency
        void dumpRegisters(void);

};
#endif