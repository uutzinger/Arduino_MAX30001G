#ifndef MAX30001G_COMM_H
#define MAX30001G_COMM_H

#include <SPI.h>      // Include SPI for communication
#include <stdint.h>
#include "max30001g_defs.h"

// SPI Settings
SPISettings SPI_SETTINGS(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0); 

class MAX30001G {
public:
    MAX30001G(uint8_t csPin);  // Constructor with chip select pin
    void     writeRegister(uint8_t address, uint32_t data);
    uint32_t readRegister24(uint8_t address);
    uint8_t  readRegisterByte(uint8_t address);
    bool     verifyWrite(uint8_t address, uint32_t expectedValue);
    bool     spiCheck();

private:
    uint8_t _csPin;  // Chip select pin
};

#endif  // MAX30001G_COMM_H
