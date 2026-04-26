#include <Arduino.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN   = 12;
const int AFE_INT2_PIN   = 13;

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

void printHealthCheck(const HealthCheckResult& result) {
  const bool info_ok = (result.info_reg != 0U);
  const uint32_t non_pll_fault_mask = MAX30001_STATUS_EOVF |
                                      MAX30001_STATUS_BOVF |
                                      MAX30001_STATUS_BOVER |
                                      MAX30001_STATUS_BUNDR |
                                      MAX30001_STATUS_DCLOFFINT;
  const bool non_pll_fault_present = ((result.status_reg & non_pll_fault_mask) != 0U);
  const bool status_ok = !non_pll_fault_present;

  Serial.println("MAX30001G Hardware Health Check");
  Serial.println("-------------------------------");
  Serial.print("SPI OK:          ");
  Serial.println(result.spi_ok ? "YES" : "NO");
  Serial.print("INFO OK:         ");
  Serial.println(info_ok ? "YES" : "NO");
  Serial.print("STATUS OK:       ");
  Serial.println(status_ok ? "YES" : "NO");
  Serial.print("INFO REG:        0x");
  Serial.println(result.info_reg, HEX);
  Serial.print("STATUS REG:      0x");
  Serial.println(result.status_reg, HEX);
  Serial.print("Idle PLL Status: ");
  Serial.println(result.pll_unlocked ? "UNLOCK FLAG SET" : "NO UNLOCK FLAG");
  Serial.print("Non-PLL Fault:   ");
  Serial.println(non_pll_fault_present ? "YES" : "NO");
  Serial.print("Overall:         ");
  Serial.println(result.spi_ok && info_ok && status_ok ? "PASS" : "FAIL");
}

void setup() {
  currentLogLevel = LOG_LEVEL_INFO;

  Serial.begin(115200);
  delay(1000);

  afe.begin();

  const HealthCheckResult result = afe.healthCheck();
  Serial.println();
  Serial.println("===================================================");
  printHealthCheck(result);

  Serial.println();
  Serial.println("===================================================");
  afe.readInfo();
  afe.printInfo();

  Serial.println();
  Serial.println("===================================================");
  afe.readStatusRegisters();
  afe.printStatus();

  Serial.println();
  Serial.println("===================================================");
  afe.readAllRegisters();
  afe.printAllRegisters();

  Serial.println();
  Serial.println("===================================================");
  Serial.println("END OF HEALTH CHECK");
}

void loop() {
  delay(1000);
}
