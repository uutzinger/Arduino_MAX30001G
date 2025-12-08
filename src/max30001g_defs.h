/******************************************************************************************************/
/* MAX30001G */
/******************************************************************************************************/
#ifndef MAX30001_DEFS_H
#define MAX30001_DEFS_H

// Define SPI speed
#define MAX30001_SPI_SPEED 1000000

// MAX30001 Register definitions
// Table 12
////////////////////////////////////////////////////////////////////////////////
#define MAX30001_NO_OP           0x00 // No operation                        R/W
#define MAX30001_STATUS          0x01 // Status register                     R   
#define MAX30001_EN_INT1         0x02 // INT B output                        R/W
#define MAX30001_EN_INT2         0x03 // INT2B output                        R/W
#define MAX30001_MNGR_INT        0x04 // Interrupt management                R/W
#define MAX30001_MNGR_DYN        0x05 // Dynamic modes management            R/W
#define MAX30001_SW_RST          0x08 // Software reset                      W
#define MAX30001_SYNCH           0x09 // Synchronize, begins new operations  W
#define MAX30001_FIFO_RST        0x0A // FIFO reset                          W
#define MAX30001_INFO            0x0F // Information on MAX30001             R
#define MAX30001_CNFG_GEN        0x10 // General settings                    R/W
#define MAX30001_CNFG_CAL        0x12 // Internal calibration settings       R/W
#define MAX30001_CNFG_EMUX       0x14 // Input multiplexer for ECG           R/W 
#define MAX30001_CNFG_ECG        0x15 // ECG channel                         R/W
#define MAX30001_CNFG_BMUX       0x17 // Input multiplexr for BIOZ           R/W
#define MAX30001_CNFG_BIOZ       0x18 // BIOZ channel                        R/W
#define MAX30001_CNFG_BIOZ_LC    0x1A // BIOZ low current ranges             R/W
#define MAX30001_CNFG_RTOR1      0x1D // R to R heart rate detection         R/W
#define MAX30001_CNFG_RTOR2      0x1E // second part of Register             R/W

// FIFO is a circular memory of 32*24 bits
// An interrupt is triggered when threshold for number of samples is reached
// An overflow is triggered if the write pointer reaches the read pointer
// FIFO Data:
//   ECG[ 23: 6] two s complement, 
//   ECG[  5: 3] Tag: Valid, Fast, Valid EOF, Fast EOF, Empty, Overflow
//   BIOZ[23: 4] two s complement
//   BIOZ[ 2: 0] Tag,Valid, Over/Under range, Valid EOF, Over/Under EOF, Empty, Overflow
//   RTOR[23:10] two s complement
#define MAX30001_ECG_FIFO_BURST  0x20 // FIFO address to read multiple values sequentially (burst mode)
#define MAX30001_ECG_FIFO        0x21 // FIFO address to read each element individually
#define MAX30001_BIOZ_FIFO_BURST 0x22 // FIFO
#define MAX30001_BIOZ_FIFO       0x23 // FIFO
#define MAX30001_RTOR            0x25 // Result of internal R to R detection

// MAX30001 Commands
#define MAX30001_WRITEREG        0x00
#define MAX30001_READREG         0x01

//
#define MAX30001_RTOR_INTR_MASK  0x04

// MAX30001 Registers
///////////////////////////////////////////////////
// MAX30001 EN_INT Register Bit Masks
#define MAX30001_EN_INT_EINT             (1 << 23)
#define MAX30001_EN_INT_EOVF             (1 << 22)
#define MAX30001_EN_INT_FSTINT           (1 << 21)
#define MAX30001_EN_INT_DCLOFFINT        (1 << 20)
#define MAX30001_EN_INT_BINT             (1 << 19)
#define MAX30001_EN_INT_BOVF             (1 << 18)
#define MAX30001_EN_INT_BOVER            (1 << 17)
#define MAX30001_EN_INT_BUNDER           (1 << 16)
#define MAX30001_EN_INT_BCGMON           (1 << 15)

#define MAX30001_EN_INT_LONINT           (1 << 11)
#define MAX30001_EN_INT_RRINT            (1 << 10)
#define MAX30001_EN_INT_SAMP             (1 << 9)
#define MAX30001_EN_INT_PLLINT           (1 << 8)

#define MAX30001_EN_INT_BCGMP            (1 << 5)
#define MAX30001_EN_INT_BCGMN            (1 << 4)
#define MAX30001_EN_INT_LDOFF_PH         (1 << 3)
#define MAX30001_EN_INT_LDOFF_PL         (1 << 2)
#define MAX30001_EN_INT_LDOFF_NH         (1 << 1)
#define MAX30001_EN_INT_LDOFF_NL         (1 << 0)

#endif