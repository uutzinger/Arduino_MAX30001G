#include <SPI.h>         // SPI driver
#include <Arduino.h>     // for digital write
#include "logger.h"      // Logging 
#include "max30001g.h"

SPISettings SPI_SETTINGS(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0);

void MAX30001G::writeRegister(uint8_t address, uint32_t data) {
  /*
    * Service routine to write to register with SPI
    */
    address &= 0x7F;         // register address is 7 bits
    data &= 0x00FFFFFFUL;    // register payload is 24 bits
    LOGD("SPI write 0x%02X, 0x%06lX", address, (unsigned long)data);  // Address: hex, Data: 24-bit hex
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);

    SPI.transfer((address << 1) | MAX30001_WRITEREG); // write command
    SPI.transfer((data >> 16) & 0xFF);   // top 8 bits
    SPI.transfer((data >> 8)  & 0xFF);   // middle 8 bits
    SPI.transfer(data         & 0xFF);   // low 8 bits
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}
    
uint32_t MAX30001G::readRegister24(uint8_t address) {
  /*
    * Service routine to read 3 bytes of data through SPI
    */
    address &= 0x7F; // register address is 7 bits
    uint32_t data = 0;
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);
    
    SPI.transfer((address << 1) | MAX30001_READREG);
    data  = ((uint32_t)SPI.transfer(0xff) << 16); // top 8 bits
    data |= ((uint32_t)SPI.transfer(0xff) << 8);  // middle 8 bits
    data |=  (uint32_t)SPI.transfer(0xff);        // low 8 bits
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    LOGD("SPI read 0x%02X, 0x%06lX", address, (unsigned long)data);
    return data;
}
    
uint8_t MAX30001G::readRegisterByte(uint8_t address) {
  /*
    * Service routine to read one byte of data through SPI
    */
    address &= 0x7F; // register address is 7 bits
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);
    
    SPI.transfer((address << 1) | MAX30001_READREG);
    uint8_t data =  SPI.transfer(0xff);  // 8 bits
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    LOGD("SPI read 0x%02X, 0x%02X", address, data);
    return data;
}

bool MAX30001G::verifyWrite(uint8_t address, uint32_t expectedValue) {
  uint32_t actualValue = readRegister24(address);
  if ((actualValue & 0x00FFFFFFUL) != (expectedValue & 0x00FFFFFFUL)) {
    LOGE("Write verification failed at 0x%02X: wrote 0x%06lX, read back 0x%06lX",
        address, (unsigned long)(expectedValue & 0x00FFFFFFUL), (unsigned long)(actualValue & 0x00FFFFFFUL));
    return false;
  }
  return true;
}

bool MAX30001G::spiCheck(){
  /*
    Perform a self check to verify connections and chip clock integrity
    Returns true if the self-check passes
  */

  uint32_t originalInfo = readRegister24(MAX30001_INFO);


  // Check if the chip is enabled
  if (originalInfo == 0) {
    LOGE("SPI check failed: INFO register read back as 0x000000");
    return false;
  }

  // Check that that chip produced consistant data
  for (int i = 0; i < 5; i++) {
    uint32_t newInfo = readRegister24(MAX30001_INFO);
    if (newInfo != originalInfo) {
      LOGE("SPI check failed: INFO register unstable (0x%06lX vs 0x%06lX)",
           (unsigned long)(originalInfo & 0x00FFFFFFUL), (unsigned long)(newInfo & 0x00FFFFFFUL));
      return false;
    }
  }
  return true;
}
