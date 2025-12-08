/******************************************************************************************************/
// Include file for the MAX30001G ECG and Bioimpedance module
//
// Urs Utzinger, July 2024
/******************************************************************************************************/

#ifndef MAX30001G_H
#define MAX30001G_H

// Standard Libraries
#include <stdint.h>
#include <SPI.h> // for spi.begin

// MAX30001G Driver Modules
#include "max30001g_defs.h"           // Register definitions
#include "max30001g_regs_typedefs.h"  // Register typedefs
#include "max30001g_globals.h"        // Global variables
#include "max30001g_comm.h"           // SPI communication
#include "max30001g_interrupt.h"      // Interrupt handling
#include "max30001g_readdata.h"       // FIFO and data handling
#include "max30001g_system.h"         // Device reset, sync
#include "max30001g_configure_ecg.h"  // ECG configuration
#include "max30001g_configure_bioz.h" // BIOZ configuration
#include "max30001g_configure_lead.h" // Lead detection settings
#include "max30001g_configure_rtor.h" // R-to-R interval settings
#include "max30001g_calibration.h"    // Calibration functions

/******************************************************************************************************/
/* Structures */
/******************************************************************************************************/

struct ImpedanceModel {
    float magnitude;
    float phase;
};

/******************************************************************************************************/
/* Device Driver */
/******************************************************************************************************/

class MAX30001G {

    public:
        MAX30001G(uint8_t csPin);
    

        // Fucntions ------------------------------------------------------------------
        // Setup Functions
        void setupECG();            // initialize AFE for ECG & RtoR mode
        void setupECGcalibration(); // initialize AFE for ECG calibration mode

        void setupBIOZ();           // initialize AFE for BIOZ
        void setupBIOZcalibrationInternal();
        void setupBIOZcalibrationExternal();

        void scanBIOZ();            // impedance scan from 1..100kHz

        void setupECGandBIOZ();     // initialize AFE for ECG and BIOZ

        void enableAFE(bool ECG, bool BIOZ, bool RtoR);
        
    private:


        // BIOZ functions -------------------------------------------------
        
        // given a desired current, find the closest valid current
        int32_t closestCurrent(int32_t current); 
        
        // calculate impedance 
        // given: Impedance magnitude and Impedance offset Theta 
        float impedancemodel(float theta, float magnitude, float phase); 
        
        ImpedanceModel fitImpedance(const float* phases, const float* impedances, int num_points);
        // compute impedance magnitude and phase from multiple measurements at given modulation frequency but varying phase offsets
        
        // Global Variables
        int _csPin;                  // chip select for SPI

};

#endif // MAX30001G_H
