#ifndef MAX30001G_INTERRUPT_H
#define MAX30001G_INTERRUPT_H

#include <stdint.h>

class MAX30001G {

    public:
        void serviceAllInterrupts();
        /* General Interrupt Handling Routine
        * This will updated global variables
        *  The main program will need to check the variables
        */
        
        bool setInterrupt1(bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire) { 
            return setInterrupt(MAX30001_EN_INT1, ecg, bioz, rtor, leadon, leadoff, bioz_fourwire); } // Configures INT1
        bool setInterrupt2(bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire) { 
            return setInterrupt(MAX30001_EN_INT2, ecg, bioz, rtor, leadon, leadoff, bioz_fourwire);}  // Configures INT2
        
        /*
        Enable/Disable Interrupts.

        There are two interrupt lines and each can be triggered by setting its registers
        Here we set the first interrupt line.

        ecg:      ECG   related interrupts enabled
        bioz:     BIOZ  related interrupts enabled
        leadon:   Leads on detection enabled
        leadoff:  Leads off detection enabled
        fourwire: BIOZ  current monitor if 4 wire BIOZ is used
        */
       
        void setDefaultInterruptClearing();
        /*
        * Set defaults for interrupt clearing
        * We will clear automatically or when data register is read
        
        Clear RR Interrupt
        0 - on status read
        1 - on R to R read **
        2 - self clear after 2-8ms
        3 - do not use
        
        Fast Mode Clear Behaviour
        0 - clear on read **
        1 - clear on read and fast recovery is complete
        
        Sample synchronization pulse clear behaviour
        0 - clear on read
        1 - self clear after 1/4 of data cycle **
        
        Sample Synchronization Pulse Frequency
        0 - every sample instant **
        1 - every 2nd sample instant
        2 - every 4th sample instant
        3 - every 16th sample instant
        
        */

        void setFIFOInterruptThreshold(uint8_t ecg,  uint8_t bioz);
        /* 
        Number of samples in FIFO that will trigger an interrupt

        ECG interrupt threshold
        BIOZ interrupt threshold

        ecg  1..32
        bioz 1..8

        */

    private: 
        bool setInterrupt(uint8_t interrupt, bool ecg, bool bioz, bool leadon, bool leadoff, bool bioz_fourwire);
