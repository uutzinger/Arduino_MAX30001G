/******************************************************************************************************/
/* Structures */
/******************************************************************************************************/

#ifndef MAX30001G_TYPEDEFS_H
#define MAX30001G_TYPEDEFS_H

#include <stdint.h>
#include "max30001g_defs.h"

/**
 * @brief ImpedanceModel structure for storing impedance measurement results.
 * @details Contains the magnitude and phase information for each measured impedance point.
 * @details Results of impedance fitting with fitImpedance() function.
 */
struct ImpedanceModel {
  float magnitude;
  float phase;
};

/**
 * @brief ImpedanceSpectrum structure for storing one full BIOZ sweep.
 * @details Each index corresponds to one modulation frequency bin with fitted magnitude and phase.
 */
struct ImpedanceSpectrum {
  float frequency[MAX30001_BIOZ_NUM_FREQUENCIES];
  float magnitude[MAX30001_BIOZ_NUM_FREQUENCIES];
  float phase[MAX30001_BIOZ_NUM_FREQUENCIES];
};

/**
 * @brief HealthCheckResult structure for storing health check results.
 * @details Contains status information about the device's operational state.
 * @details spi_ok indicates whether SPI communication appears functional.
 * @details info_reg and status_reg store the raw contents of the INFO and STATUS registers for debugging.
 * @details pll_unlocked indicates whether the PLL_UNLOCK status bit is set, which can affect timing and data validity.
 * @details fault_present indicates whether any fault conditions are currently active, which may require attention or indicate issues with the device or measurement setup.
 */
struct HealthCheckResult {
  bool spi_ok;
  uint32_t info_reg;
  uint32_t status_reg;
  bool pll_unlocked;
  bool fault_present;
};

/**
 * @brief BIOZScanConfig configuration structure fields:
 * @details config.avg = 2                       number of samples to average at each frequency/phase point (1-8)
 * @details config.fast = false                  if true, use 60sps BIOZ sampling rate; otherwise use 30sps
 * @details config.fourleads = false             if true, use 4-wire BIOZ configuration; otherwise use 2-wire
 * @details config.max_retries = 4               number of retries per frequency/phase point if measurement is unsuccessful (0-3)
 * @details config.low_target_fraction = 0.10f   signal is low if below 10% of ADC range for current adjustment
 * @details config.high_target_fraction = 0.90f  signal is high if above 90% of ADC range for current adjustment
 * @details config.target_fraction = 0.60f       signal is ok if above 60% of ADC range for current adjustment
 * @details config.outlier_min_samples = 5       minimum number of samples required to perform outlier rejection when averaging at each point
 * @details config.outlier_sigma = 2.5f          number of standard deviations for outlier rejection when averaging multiple readings at each point
 * @details config.timeout_margin_ms = 50        margin in milliseconds for FIFO read timeout when waiting for samples at each point
 * @details config.freq_start_index = 0          index of first modulation frequency to scan (0-10, corresponding to 128kHz..125Hz)
 * @details config.freq_end_index = 7            index of last modulation frequency to scan (0-10, corresponding to 128kHz..125Hz)
 * @details config.initial_current_nA = 8000     initial current magnitude in nanoAmps (55..96,000)
  */
struct BIOZScanConfig {
  uint8_t avg;
  bool fast;
  bool fourleads;
  uint8_t max_retries;
  float low_target_fraction;
  float high_target_fraction;
  float target_fraction;
  uint8_t outlier_min_samples;
  float outlier_sigma;
  uint16_t timeout_margin_ms;
  uint8_t freq_start_index;
  uint8_t freq_end_index;
  int32_t initial_current_nA;

  BIOZScanConfig()
      : avg(2),
        fast(false),
        fourleads(false),
        max_retries(4),
        low_target_fraction(0.10f),
        high_target_fraction(0.90f),
        target_fraction(0.60f),
        outlier_min_samples(5),
        outlier_sigma(2.5f),
        timeout_margin_ms(50),
        freq_start_index(0),
        freq_end_index(7),
        initial_current_nA(8000) {}
};

/**
 * @brief ConfigSnapshot structure for storing/restoring MAX30001G register configurations.
 * @details Contains key configuration registers that define the device's operating state.
 * @details Used for saving/restoring configurations during impedance scans or other operations that modify settings.
 */
struct ConfigSnapshot {
  bool valid;
  uint32_t en_int1;
  uint32_t en_int2;
  uint32_t mngr_int;
  uint32_t mngr_dyn;
  uint32_t cnfg_gen;
  uint32_t cnfg_cal;
  uint32_t cnfg_emux;
  uint32_t cnfg_ecg;
  uint32_t cnfg_bmux;
  uint32_t cnfg_bioz;
  uint32_t cnfg_bioz_lc;
  uint32_t cnfg_rtor1;
  uint32_t cnfg_rtor2;
};

#endif // MAX30001G_TYPEDEFS_H
