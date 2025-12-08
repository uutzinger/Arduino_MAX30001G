/******************************************************************************************************/
/* MAX30001G                                                                                          */
/* Register Structures                                                                                */
/******************************************************************************************************/

#ifndef MAX30001G_REGS_H
#define MAX30001G_REGS_H

#include <stdint.h>

class MAX30001G {
    public:
        // Read Registers
        void readAllRegisters(void); // All including the ones below
        void readInfo(void);         // Just Info register
        void readStatusRegisters()   // Just Status register

        // Report
        void dumpRegisters(void);
        void printInfo(void);
        void printStatus();

    private:
        // Report Interrupts
        void printEN_INT(max30001_en_int_reg_t en_int);
        void printEN_INT1() { printEN_INT(en_int1); }
        void printEN_INT2() { printEN_INT(en_int2); }
        // Report Management
        void printMNGR_INT(void);
        void printMNGR_DYN(void);
        // Report Configuration
        void printCNFG_GEN(void);
        void printCNFG_CAL(void);
        void printCNFG_EMUX(void);
        void printCNFG_ECG(void);
        void printCNFG_BMUX(void);
        void printCNFG_BIOZ(void);
        void printCNFG_BIOZ_LC(void);
        void printCNFG_RTOR1(void);
        void printCNFG_RTOR2(void);
};

#endif