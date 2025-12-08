/******************************************************************************************************/
// Configure Leads
/******************************************************************************************************/
#ifndef MAX30001G_CONFIG_LEADS_H
#define MAX30001G_CONFIG_LEADS_H

#include <stdint.h>

class MAX30001G {
    private:        

        void setLeadsOnDetection(bool enable);
        // Lead on is used in low power mode to assert interrupt when leads are 
        // attached so that the device can automatically power on

        void setLeadsOffDetection(bool enable, bool bioz_4, uint8_t electrode_impedance);
        // Configure leads off detection for the ECG or BioZ channels.
        // The correct leads off detection features are automaticaly enabled (DC, Current and AC leads off)
        //  Before running this routine
        //  - Make sure  that ecg or bioz is active.
        //  - Lead polarity is set
        // enable: 
        //  - enable/disable leads-off detection.
        // bioz_4: 
        //  - true  = 4 wire BIOZ 
        //  - false = 2 wire BIOZ 
        // electrode_impedance: 
        //  - >0 - 2. 2- 4, 4-10, 10-20, >20 MOhm
        //  - used to determine current magnitude, wet electrodes need higher current for lead on detection

        void setLeadsBias(bool enable, uint8_t resistance);
        // enable:
        //  0 = disable, 1 = enable
        // resistance: 
        //  0     use external lead bias, (in BIOZ mode only available for high current mode)
        //  >=150 internal lead bias 200M Ohm 
        //  >  75 internal lead bias 100M Ohm default
        //  else  internal lead bias 50M Ohm
  
    };
#endif