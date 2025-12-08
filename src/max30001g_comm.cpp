#include <SPI.h>         // SPI driver
#include <Arduino.h>     // for digital write
#include "logger.h"      // Logging 
#include "max30001g_comm.h"

void MAX30001G::writeRegister(uint8_t address, uint32_t data) {
  /*
    * Service routine to write to register with SPI
    */
    LOGD("SPI write 0x%02X, 0x%06X", address, data);  // Address: hex, Data: 24-bit hex
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);

    SPI.transfer((address << 1) | 0x00); // write command 
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
    uint32_t data;
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);
    
    SPI.transfer((address << 1) | 0x01);
    data  = ((uint32_t)SPI.transfer(0xff) << 16); // top 8 bits
    data |= ((uint32_t)SPI.transfer(0xff) << 8);  // middle 8 bits
    data |=  (uint32_t)SPI.transfer(0xff);        // low 8 bits
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    LOGD("SPI read 0x%02X, 0x%06X", address, data);
    return data;
}
    
uint8_t MAX30001G::readRegisterByte(uint8_t address) {
  /*
    * Service routine to read one byte of data through SPI
    */
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(_csPin, LOW);
    
    SPI.transfer(address << 1 | 0x01);
    uint8_t data =  SPI.transfer(0xff);  // 8 bits
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    LOGD("SPI read 0x%02X, 0x%02X", address, data);
    return data;
}

bool MAX30001G::verifyWrite(uint8_t address, uint32_t expectedValue) {
  uint32_t actualValue = readRegister24(address);
  if ((actualValue & 0xFFFFFF) != (expectedValue & 0xFFFFFF)) {
    LOGE("Write verification failed at 0x%02X: wrote 0x%06X, read back 0x%06X",
        address, expectedValue, actualValue);
    return false;
  }
  return true;
}

bool MAX30001G::spiCheck(){
  /*
    Perform a self check to verify connections and chip clock integrity
    Returns true if the self-check passes
  */

  max30001_info_t originalInfo, newInfo;
  originalInfo.all = readRegister24(MAX30001_INFO);


  // Check if the chip is enabled
  if ( originalInfo.all == 0 ) { return false; } 

  // Check that that chip produced consistant data
  for (int i = 0; i < 5; i++) {
    newInfo.all = readRegister24(MAX30001_INFO);
    if ( newInfo.all != originalInfo.all)  { 
      return false;
    }
  }
  return true;
}