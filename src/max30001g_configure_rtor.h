/******************************************************************************************************/
// Configure RtoR
/******************************************************************************************************/
#ifndef MAX30001G_CONFIG_RTOR_H
#define MAX30001G_CONFIG_RTOR_H

#include <stdint.h>

class MAX30001G {
    public:
    void setDefaultRtoR();
    // ptsf: 0b0011 (4) (4/16) or (6) (6/16)
    // pavg: 0b10     (8)
    // gain: 0b1111   (Auto-Scale)
    // wndw: 0b011    12ms (96ms)
    // hoff: 0b100000 (32)
    // ravg: 0b10     (8)
    // rhsf: 0b100    (4/8)    
    void setDefaultNoRtoR();

    private:
    void setRtoR(bool enable, uint8_t ptsf, uint8_t pavg, uint8_t gain, uint8_t wndw, uint8_t hoff, uint8_t ravg, uint8_t rhsf);
};
#endif