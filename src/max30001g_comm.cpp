#include <SPI.h>         // SPI driver
#include <Arduino.h>     // for digital write
#include "logger.h"      // Logging 
#include "max30001g.h"

SPISettings SPI_SETTINGS(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0);

namespace {

bool hasRepeatedBytePattern24(uint32_t value) {
  const uint8_t b2 = static_cast<uint8_t>((value >> 16) & 0xFFU);
  const uint8_t b1 = static_cast<uint8_t>((value >> 8) & 0xFFU);
  const uint8_t b0 = static_cast<uint8_t>(value & 0xFFU);
  return (b2 == b1) && (b1 == b0);
}

bool infoRegisterLooksPlausible(uint32_t rawInfo) {
  max30001_info_reg_t info;
  info.all = rawInfo & 0x00FFFFFFUL;

  // INFO contains fixed silicon identification fields. If these do not match,
  // the bus is returning garbage even if the read is stable.
  if (info.bit.c1 != 0x1U) {
    return false;
  }
  if (info.bit.c2 != 0x5U) {
    return false;
  }
  if ((rawInfo & 0xFF000000UL) != 0U) {
    return false;
  }
  return true;
}

} // namespace

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

  const uint32_t originalInfo = readRegister24(MAX30001_INFO) & 0x00FFFFFFUL;

  // Reject the common "nothing connected / floating bus" patterns up front.
  if (originalInfo == 0x000000UL) {
    LOGE("SPI check failed: INFO register read back as 0x000000");
    return false;
  }
  if (originalInfo == 0xFFFFFFUL) {
    LOGE("SPI check failed: INFO register read back as 0xFFFFFF");
    return false;
  }
  if (hasRepeatedBytePattern24(originalInfo)) {
    LOGE("SPI check failed: INFO register has suspicious repeated-byte pattern 0x%06lX",
         (unsigned long)originalInfo);
    return false;
  }

  if (!infoRegisterLooksPlausible(originalInfo)) {
    max30001_info_reg_t info;
    info.all = originalInfo;
    LOGE("SPI check failed: INFO register decode invalid (0x%06lX, c1=%u, c2=%u, rev=%u)",
         (unsigned long)originalInfo,
         (unsigned int)info.bit.c1,
         (unsigned int)info.bit.c2,
         (unsigned int)info.bit.revision);
    return false;
  }

  // Check that the chip produces consistent identification data.
  for (int i = 0; i < 5; i++) {
    const uint32_t newInfo = readRegister24(MAX30001_INFO) & 0x00FFFFFFUL;
    if (newInfo != originalInfo) {
      LOGE("SPI check failed: INFO register unstable (0x%06lX vs 0x%06lX)",
           (unsigned long)originalInfo, (unsigned long)newInfo);
      return false;
    }
    if (!infoRegisterLooksPlausible(newInfo)) {
      LOGE("SPI check failed: INFO register changed to invalid decode (0x%06lX)",
           (unsigned long)newInfo);
      return false;
    }
  }

  // A second register should not mirror INFO exactly. When it does, the bus is
  // usually returning a fixed pattern independent of address.
  const uint32_t status = readRegister24(MAX30001_STATUS) & 0x00FFFFFFUL;
  if (status == originalInfo) {
    LOGE("SPI check failed: STATUS matches INFO exactly (0x%06lX), address decode likely broken",
         (unsigned long)status);
    return false;
  }

  return true;
}
