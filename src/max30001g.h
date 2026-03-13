/******************************************************************************************************/
// Include file for the MAX30001G ECG and Bioimpedance module
//
// Urs Utzinger, maintainer for the code, July 2024 - present
// GPT-5.3 assisted development, review and testing, Spring 2026
/******************************************************************************************************/

#ifndef MAX30001G_H
#define MAX30001G_H

// Standard Libraries
#include <stdint.h>
#include <SPI.h>

// MAX30001G base data types
#include "max30001g_defs.h"                // Register addresses, bit masks, and other constants
#include "max30001g_regs_typedefs.h"       // Register value structures and unions
#include "max30001g_typedefs.h"            // Data structures for BIOZ and ECG configuration and scan results
#include "max30001g_globals.h"             // Global variables for device state and configuration

// SPI Settings (defined in max30001g_comm.cpp)
extern SPISettings SPI_SETTINGS;

/**
 * @defgroup MAX30001G_Public_API MAX30001G Public API
 * @brief Primary application-facing interface for the MAX30001G driver.
 * @{
 */

/**
 * @brief MAX30001G device driver class used by Arduino sketches.
 * @ingroup MAX30001G_Public_API
 */
class MAX30001G {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
public:

  /**
   * @brief Construct a MAX30001G device instance.
   * @param csPin SPI chip-select pin.
   * @param intPin1 Optional INTB pin number. Use -1 when not connected.
   * @param intPin2 Optional INT2B pin number. Use -1 when not connected.
   */
  MAX30001G(uint8_t csPin, int intPin1 = -1, int intPin2 = -1);

  enum InterruptEvent : uint8_t {
    IRQ_ECG_FIFO = 0,
    IRQ_ECG_OVF,
    IRQ_ECG_FAST,
    IRQ_ECG_LEADOFF,
    IRQ_BIOZ_FIFO,
    IRQ_BIOZ_OVF,
    IRQ_BIOZ_OVER,
    IRQ_BIOZ_UNDER,
    IRQ_BIOZ_CGMON,
    IRQ_LEADS_ON,
    IRQ_RTOR,
    IRQ_PLL_UNLOCK,
    IRQ_EVENT_COUNT
  };

  typedef void (*InterruptCallback)(MAX30001G* device, uint32_t status, void* context);

  /* @brief Initialize the device state and read startup registers. */
  void begin(void);

  /* @brief Stop device operation, disable interrupts, and detach ISR pins. */
  void end(void);

  /**
   * @brief Start measurement engines according to the most recent setup profile.
   */
  void start(void);

  /**
   * @brief Stop active measurement engines.
   */
  void stop(void);

  /**
   * @brief Service pending IRQ work and drain enabled FIFOs.
   * @param reportRaw True to keep raw ADC codes, false to convert to engineering units.
   * @return True if update handled any interrupt/data/fault work.
   */
  bool update(bool reportRaw = false);

  /**
   * @brief Alias for raw-code update cycle.
   * @return True if update handled any interrupt/data/fault work.
   */
  bool updateRaw(void) { return update(true); }

  // ---------------------------------------------------------------------------------------
  // Setup Functions
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Enable or disable ECG, BIOZ, and R-to-R engines.
   * @param ECG True to enable ECG path.
   * @param BIOZ True to enable BIOZ path.
   * @param RtoR True to enable R-to-R detection.
   */
  void enableAFE(bool ECG, bool BIOZ, bool RtoR);
  
  /**
   * @brief Configure a ready-to-use ECG profile.
   * @details  speed 
   * @details    0 ~125 sps
   * @details    1 ~256 sps * default
   * @details    2 ~512 sps
   * @details  gain 
   * @details    0  20 V/V
   * @details    1  40 V/V
   * @details    2  80 V/V * default
   * @details    3 160 V/V
   * @details  three leads
   * @details    true  3 lead ECG (with ground on RL or LL) * default
   * @details    false 2 lead ECG (with RA and LA only), should use internal leads bias
   * @param speed ECG sample-rate selector (0 low, 1 medium, 2 high).
   * @param gain ECG gain selector (0..3 => 20, 40, 80, 160 V/V).
   * @param threeleads True for 3-lead operation, false for 2-lead operation.
   */
  void setupECG(
    uint8_t speed = 1, 
    uint8_t gain = 2, 
    bool threeleads = true);
  
    /**
   * @brief Configure a ready-to-use BIOZ profile.
   * @details speed: 0=low-rate (~25..32sps), 1=high-rate (~50..64sps).
   * @details gain: 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V.
   * @details ahpf: 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass.
   * @details dlpf: 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz.
   * @details dhpf: 0=bypass, 1=0.05Hz, 2/3=0.5Hz class.
   * @param speed BIOZ sample-rate selector (0 low, 1 high).
   * @param gain BIOZ gain selector (0..3 => 10, 20, 40, 80 V/V).
   * @param ahpf BIOZ analog HPF selector (0..7).
   * @param dlpf BIOZ digital LPF selector (0..3).
   * @param dhpf BIOZ digital HPF selector (0..3).
   * @param frequency Target modulation frequency in Hz (mapped to nearest valid FCGEN).
   * @param current Target drive current in nA (mapped/clamped to valid settings).
   * @param phase Demodulation phase offset in degrees.
   * @param leadbias True to enable lead-bias configuration in setup.
   * @param leadsoffdetect True to configure lead-off detection in setup.
   * @param fourleads True for 4-wire BIOZ assumptions, false for 2-wire.
   */
  void setupBIOZ(
    uint8_t speed = 0, 
    uint8_t gain = 1,
    uint8_t ahpf = 1, 
    uint8_t dlpf = 1, 
    uint8_t dhpf = 0,
    uint16_t frequency = 40000, 
    uint16_t current = 8000, 
    float phase = 0.0f,
    bool leadbias = true, 
    bool leadsoffdetect = false, 
    bool fourleads = false
  );

  /**
   * @brief Configure simultaneous ECG and BIOZ operation.
   * @details ecg_speed: 0=~125sps, 1=~256sps, 2=~512sps.
   * @details ecg_gain: 0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V.
   * @details bioz_speed: 0=~25..32sps, 1=~50..64sps.
   * @details bioz_gain: 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V.
   * @details bioz_dlpf: 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz.
   * @details bioz_dhpf: 0=bypass, 1=0.05Hz, 2/3=0.5Hz class.
   * @param ecg_speed ECG sample-rate selector (0..2).
   * @param ecg_gain ECG gain selector (0..3).
   * @param ecg_threeleads True for 3-lead ECG mode.
   * @param bioz_speed BIOZ sample-rate selector (0..1).
   * @param bioz_gain BIOZ gain selector (0..3).
   * @param bioz_dlpf BIOZ digital LPF selector (0..3).
   * @param bioz_dhpf BIOZ digital HPF selector (0..3).
   * @param bioz_frequency BIOZ modulation frequency target in Hz.
   * @param bioz_current BIOZ drive current target in nA.
   * @param bioz_phase BIOZ phase offset target in degrees.
   * @param leadbias True to configure lead bias.
   * @param leadsoffdetect True to configure lead-off detection.
   * @param bioz_fourleads True for 4-wire BIOZ assumptions.
   */
  void setupECGandBIOZ(
    uint8_t ecg_speed = 1, 
    uint8_t ecg_gain = 2, 
    bool ecg_threeleads = true,
    uint8_t bioz_speed = 0, 
    uint8_t bioz_gain = 1,
    uint8_t bioz_dlpf = 1, 
    uint8_t bioz_dhpf = 0,
    uint16_t bioz_frequency = 8000, 
    uint16_t bioz_current = 8000, 
    float bioz_phase = 0.0f,
    bool leadbias = true, 
    bool leadsoffdetect = false, 
    bool bioz_fourleads = false
  ); // initialize AFE for ECG and BIOZ

  /**
   * @brief Configure ECG voltage calibration signal path.
   * @details speed: 0=~125sps, 1=~256sps, 2=~512sps.
   * @details gain: 0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V.
   * @param speed ECG sample-rate selector (0..2).
   * @param gain ECG gain selector (0..3).
   */
  void setupECGSignalCalibration(uint8_t speed = 1, uint8_t gain = 2);
  
  /**
   * @brief Configure BIOZ voltage calibration signal path.
   * @details speed: 0=low-rate (~25..32sps), 1=high-rate (~50..64sps).
   * @details gain: 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V.
   * @param speed BIOZ sample-rate selector (0..1).
   * @param gain BIOZ gain selector (0..3).
   */
  void setupBIOZSignalCalibration(uint8_t speed = 0, uint8_t gain = 1);

  /**
   * @brief Configure internal BIOZ impedance calibration/test path.
   * @details speed: 0=low-rate (~25..32sps), 1=high-rate (~50..64sps).
   * @details gain: 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V.
   * @details ahpf: 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass.
   * @details dlpf: 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz.
   * @details dhpf: 0=bypass, 1=0.05Hz, 2/3=0.5Hz class.
   * @details modulation: 0=unchopped+LPF, 1=chopped no-LPF, 2=chopped+LPF, 3=chopped+resistive-CM.
   * @details modulation_frequency: 0=4Hz, 1=1Hz, 2=0.25Hz, 3=0.0625Hz.
   * @param speed BIOZ sample-rate selector (0..1).
   * @param gain BIOZ gain selector (0..3).
   * @param ahpf BIOZ analog HPF selector (0..7).
   * @param dlpf BIOZ digital LPF selector (0..3).
   * @param dhpf BIOZ digital HPF selector (0..3).
   * @param frequency BIOZ modulation frequency target in Hz.
   * @param current BIOZ drive current target in nA.
   * @param phase Demodulation phase offset in degrees.
   * @param resistance Nominal internal test impedance value.
   * @param modulation Calibration modulation mode selector.
   * @param modulation_frequency Calibration modulation frequency selector.
   */
  void setupBIOZImpedanceCalibration(
    uint8_t speed = 0, 
    uint8_t gain = 1,
    uint8_t ahpf = 1, 
    uint8_t dlpf = 1, 
    uint8_t dhpf = 0,
    uint16_t frequency = 40000, 
    uint16_t current = 8000, 
    float phase = 0.0f,
    uint32_t resistance = 5000, 
    uint8_t modulation = 0, 
    uint8_t modulation_frequency = 3
  );
  
  /**
   * @brief Configure external impedance calibration path.
   * @param frequency BIOZ modulation frequency target in Hz.
   * @param phase Demodulation phase offset in degrees.
   */
  void setupBIOZExternalImpedanceCalibration(
    uint16_t frequency = 40000, float phase = 0.0f
  );

  /**
   * @brief Configure lifecycle-driven BIOZ scan flow.
   * @details This does not start scanning immediately. Call start(), then drive with update().
   * @param avg BIOZ FIFO average depth used per point.
   * @param fast True for high-rate BIOZ sampling.
   * @param fourleads True for 4-wire BIOZ assumptions.
   * @param fullRange True to include lowest available frequency bins.
   */
  void setupBIOZScan(uint8_t avg, bool fast, bool fourleads, bool fullRange);

  /**
   * @brief Configure lifecycle-driven BIOZ scan flow.
   * @details This does not start scanning immediately. Call start(), then drive with update().
   * @param config Scan configuration parameters.
   * @param reuseCurrents True to reuse previously optimized current profile.
   */
  void setupBIOZScan(const BIOZScanConfig& config = BIOZScanConfig(), bool reuseCurrents = false);
  
  /**
   * @brief Run impedance scan using basic options.
   * @param avg BIOZ FIFO average depth used per point.
   * @param fast True for high-rate BIOZ sampling.
   * @param fourleads True for 4-wire BIOZ assumptions.
   * @param fullRange True to include lowest available frequency bins.
   */
  void scanBIOZ(uint8_t avg = 2, bool fast = false, bool fourleads = false, bool fullRange = false);
  
  /**
   * @brief Run impedance scan using advanced tunable scan config.
   * @details BIOZScanConfig config;
   * @details config.avg = 2                       number of samples to average at each frequency/phase point (1-8)
   * @details config.fast = false                  if true, use 60sps BIOZ sampling rate; otherwise use 30sps
   * @details config.fourleads = false             if true, use 4-wire BIOZ configuration; otherwise use 2-wire
   * @details config.use_internal_resistor = false if true, scan the internal calibration resistor instead of external electrodes
   * @details config.internal_resistor_ohm = 1000  nominal internal resistor selection, nearest supported value is used
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
   * @param config Full scan configuration structure.
   * @param reuseCurrents True to reuse previously learned current profile.
   */
  void scanBIOZ(const BIOZScanConfig& config, bool reuseCurrents = false);
  
  /**
   * @brief Start state-machine style BIOZ scan flow.
   * @details Arms non-blocking scan execution; each update() call advances scan steps.
   */
  void startBIOZScan(void);
  
  /**
   * @brief Stop state-machine style BIOZ scan flow.
   * @details Stops scan flow and powers down AFE engines.
   */
  void stopBIOZScan(void);

  /**
   * @brief Advance state-machine style BIOZ scan flow by one step.
   * @details Executes one non-blocking scan state transition per call.
   */
  void stepBIOZScan(void);

  // ---------------------------------------------------------------------------------------
  // Registers: max30001g_regs.cpp
  // ---------------------------------------------------------------------------------------

  /* @brief Read all known MAX30001G registers into global register mirrors. */
  void readAllRegisters(void);
  
  /* @brief Read INFO register into global mirror. */
  void readInfo(void);
  
  /* @brief Read status register(s) and refresh status flags. */
  void readStatusRegisters(void);
 
  /* @brief Save current register configuration into internal snapshot. */
  void saveConfig(void);
  
  /* @brief Restore register configuration from internal snapshot. */
  void restoreConfig(void);
  
  /* @brief Read and print saved/current configuration related registers. */
  void printConfig(void);
  
  /* @brief Print all known register mirrors for debugging. */
  void printAllRegisters(void);
  
  /* @brief Print INFO register decode. */
  void printInfo(void);
  
  /* @brief Print STATUS register decode. */
  void printStatus(void);

  // ---------------------------------------------------------------------------------------
  // System: max30001g_system.h
  // ---------------------------------------------------------------------------------------

  /* @brief Issue synchronization command to align conversion timing. */
  void synch(void);
  
  /* @brief Issue software reset to device. */
  void swReset(void);
  
  /**
   * @brief Set master clock selector and recompute timing globals.
   * @details  0 0b00 = 32768 Hz    = FCLK
   * @details  1 0b01 = 32000 Hz    = FCLK*625/640
   * @details  2 0b10 = 32000 Hz    = FCLK*625/640
   * @details  3 0b11 = 31968.78 Hz = FCLK*640/656
   * @param fmstr FMSTR selector (0..3)(32768, 32000, 32000, 31968.78 Hz)
   */
  void setFMSTR(uint8_t fmstr);

  /**
   * @brief Run basic health checks and return a summary result.
   * @return HealthCheckResult with SPI, INFO, STATUS, and fault summary fields.
   */
  HealthCheckResult healthCheck(void);

  // ---------------------------------------------------------------------------------------
  // Read Data: max30001g_readdata.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Reset ECG and BIOZ FIFOs. Does not affect data already transferred to RingBuffer.
   */
  void FIFOReset(void);

  /**
   * @brief Burst-read ECG FIFO samples.
   * @param reportRaw True to keep ADC codes; false to convert to engineering units.
   */
  void readECG_FIFO(bool reportRaw);

  /**
   * @brief Burst-read BIOZ FIFO samples.
   * @param reportRaw True to keep ADC codes; false to convert to engineering units.
   */
  void readBIOZ_FIFO(bool reportRaw);
  
  /**
   * @brief Read one ECG FIFO sample.
   * @param reportRaw True to keep ADC code; false to convert to mV.
   * @return Sample value or sentinel for invalid/empty cases.
   */
  float readECG(bool reportRaw);
  
  /**
   * @brief Read RTOR result and update RR-interval globals/buffer.
   */
  void readRTOR(void);

  /**
   * @brief Read one BIOZ FIFO sample.
   * @param reportRaw True to keep ADC code; false to convert to ohms.
   * @return Sample value or sentinel for invalid/empty cases.
   */
  float readBIOZ(bool reportRaw);

  // ---------------------------------------------------------------------------------------
  // Interrupts: max30001g_interrupt.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Read STATUS and update interrupt-driven global flags.
   */
  void serviceAllInterrupts(void);

  /**
   * @brief Service interrupts only when ISR software flags indicate pending work.
   * @details Uses Arduino-core critical-section calls (`noInterrupts()` / `interrupts()`)
   *          to atomically snapshot and clear software pending flags.
   * @return True if an interrupt service pass was performed.
   */
  bool servicePendingInterrupts(void);

  /**
   * @brief Read and clear fault-related status conditions.
   */
  void readAndClearFaults(void);

  /**
   * @brief Clear software-latched interrupt/fault flags.
   * @details This does not read/write device registers; it only resets library globals
   *          that are latched by STATUS/FIFO parsing.
   */
  void clearLatchedStatusFlags(void);

  /**
   * @brief Register callback for one specific interrupt event.
   * @details device  - MAX30001G instance
   * @details status  - STATUS register snapshot captured by serviceAllInterrupts()
   * @details context - user pointer passed at registration time
   * @details event   - one value from InterruptEvent (for example IRQ_ECG_FIFO, IRQ_BIOZ_FIFO, IRQ_RTOR).
   * @param event Interrupt event selector.
   * @param cb Callback function pointer.
   * @param context User context pointer passed to callback.
   * @return True when registration succeeds.
   */
  bool setInterruptEventCallback(InterruptEvent event, InterruptCallback cb, void* context = nullptr);

  /**
   * @brief Register aggregate callback invoked for status bits matching a mask.
   * @param cb Callback function pointer.
   * @param context User context pointer passed to callback.
   * @param statusMask STATUS bit mask used for callback gating.
   * @return True when registration succeeds.
   */
  bool setInterruptAggregateCallback(InterruptCallback cb, void* context = nullptr, uint32_t statusMask = 0x00FFFFFFUL);

  /**
   * @brief Clear callback for one interrupt event.
   * @details event must be one value from InterruptEvent.
   * @param event Interrupt event selector.
   */
  void clearInterruptEventCallback(InterruptEvent event);

  /**
   * @brief Clear aggregate interrupt callback.
   */
  void clearInterruptAggregateCallback(void);

  /**
   * @brief Clear all registered interrupt callbacks for this instance.
   */
  void clearAllInterruptCallbacks(void);

  /**
   * @brief Convenience callback handler: read ECG FIFO.
   * @param reportRaw True to keep ADC codes; false to convert.
   */
  void handleECGFifoInterrupt(bool reportRaw = false);

  /**
   * @brief Convenience callback handler: read BIOZ FIFO.
   * @param reportRaw True to keep ADC codes; false to convert.
   */
  void handleBIOZFifoInterrupt(bool reportRaw = false);

  /**
   * @brief Convenience callback handler: read RTOR data.
   */
  void handleRtoRInterrupt(void);

  /**
   * @brief Configure interrupt line INT1 event enables.
   * @param ecg Enable ECG-related interrupt bits.
   * @param bioz Enable BIOZ-related interrupt bits.
   * @param rtor Enable RTOR interrupt bit.
   * @param leadon Enable lead-on interrupt bit.
   * @param leadoff Enable lead-off interrupt bit.
   * @param bioz_fourwire Enable 4-wire BIOZ current monitor interrupt behavior.
   * @return True when register write/verify succeeds.
   */
  bool setInterrupt1(bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire = false) {
    return setInterrupt(MAX30001_EN_INT1, ecg, bioz, rtor, leadon, leadoff, bioz_fourwire);
  }
  
  /**
   * @brief Configure interrupt line INT2 event enables.
   * @param ecg Enable ECG-related interrupt bits.
   * @param bioz Enable BIOZ-related interrupt bits.
   * @param rtor Enable RTOR interrupt bit.
   * @param leadon Enable lead-on interrupt bit.
   * @param leadoff Enable lead-off interrupt bit.
   * @param bioz_fourwire Enable 4-wire BIOZ current monitor interrupt behavior.
   * @return True when register write/verify succeeds.
   */
  bool setInterrupt2(bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire = false) {
    return setInterrupt(MAX30001_EN_INT2, ecg, bioz, rtor, leadon, leadoff, bioz_fourwire);
  }

  /**
   * @brief Configure default interrupt-clear, clears on RR read, fast recovery read, and sample read.
   */
  void setDefaultInterruptClearing(void);

  /**
   * @brief Set FIFO threshold that triggers ECG/BIOZ FIFO interrupts.
   * @param ecg ECG threshold in samples (device supports 1..32).
   * @param bioz BIOZ threshold in samples (device supports 1..8).
   */
  void setFIFOInterruptThreshold(uint8_t ecg, uint8_t bioz);

  // ---------------------------------------------------------------------------------------
  // ECG Configuration: max30001g_configure_ecg.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Set ECG sample-rate selector.
   * @details 0 low    ~125sps
   * @details 1 medium ~256sps
   * @details 2 high   ~512sps * default
   * @param ECG Rate selector (0 low, 1 medium, 2 high; constrained by FMSTR).
   */
  void setECGSamplingRate(uint8_t ECG);

  /**
   * @brief Configure ECG digital filters. 
   * @details digital lpf for the AFE  
   * @details  0 bypass ~  0 Hz
   * @details  1 low    ~ 40 Hz
   * @details  2 medium ~100 Hz
   * @details  3 high   ~150 Hz
   * @details digital hpf for the AFE  
   * @details  0         bypass * default for all speeds
   * @details  1         0.5 Hz 
   * @details fixed external analog HPF on C_HPF: 
   * @details   0.1 uF   5.00 Hz (best motion artifact suppression)
   * @details   1.0 uF   0.50 Hz
   * @details   10.0 uF   0.05 Hz (MediBrick, highest signal quality preference)
   * @param lpf LPF selector (0 bypass, 1 low, 2 medium, 3 high).
   * @param hpf HPF selector (0 bypass, 1 enable 0.5Hz class).
   */
  void setECGfilter(uint8_t lpf, uint8_t hpf);
  
  /**
   * @brief Set ECG PGA gain.
   * @details  0  20 V/V
   * @details  1  40 V/V
   * @details  2  80 V/V * default
   * @details  3 160 V/V
   * @param gain Gain selector (0..3 => 20, 40, 80, 160 V/V).
   */
  void setECGgain(uint8_t gain);
  
  /**
   * @brief Configure ECG polarity and input switch state.
   * @param inverted True to swap ECG polarity.
   * @param open True to open inputs for calibration routing.
   */
  void setECGLeadPolarity(bool inverted, bool open);
  
  /**
   * @brief Enable automatic ECG fast-recovery mode.
   * @param threshold_voltage Threshold in mV used to compute FAST_TH.
   */
  void setECGAutoRecovery(int threshold_voltage);
  
  /* @brief Disable ECG fast-recovery mode and return to normal operation. */
  void setECGNormalRecovery(void);
  
  /* @brief Enter manual ECG fast-recovery mode. */
  void startECGManualRecovery(void);
  
  /* @brief Exit manual ECG fast-recovery mode. */
  void stopECGManualRecovery(void);

  // ---------------------------------------------------------------------------------------
  // BIOZ Configuration: max30001g_configure_bioz.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Set BIOZ sample-rate selector.
   * @details 0 approx 30 sps (25-32) depending on FMSTR
   * @details 1 approx 60 sps (50-64) depending on FMSTR *
   * @param speed_select 0 low-rate mode (25-32sps), 1 high-rate mode (50..64sps)
   */
  void setBIOZSamplingRate(uint8_t speed_select);
  /**
   * @brief Set BIOZ gain and front-end power/noise mode.
   * @details  0  10 V/V
   * @details  1  20 V/V **
   * @details  2  40 V/V
   * @details  3  80 V/V
   * @param gain Gain selector (0..3 => 10*, 20, 40, 80 V/V).
   * @param lowNoise True for low-noise mode, false for low-power mode.
   */
  void setBIOZgain(uint8_t gain, bool lowNoise);

  /**
   * @brief Set BIOZ modulation frequency by target Hz.
   * @details frequency (historically data was recorded at 50KHz, any frequency from 125Hz to 128kHz is available)
   * @details   128,000
   * @details    80,000
   * @details    40,000 **
   * @details    18,000
   * @details     8,000
   * @details     4,000
   * @details     2,000
   * @details     1,000
   * @details       500 might have 60/50Hz interference
   * @details       250 might have 60/50Hz interference
   * @details       125 might have 60/50Hz interference
   * @param frequency_hz Requested frequency in Hz (mapped to nearest valid FCGEN).
   */
  void setBIOZModulationFrequencyByFrequency(uint16_t frequency_hz);

  /**
   * @brief Set BIOZ modulation frequency by FCGEN index.
   * @details selector to frequency mapping:
   * @details 0=128000Hz, 1=80000Hz, 2=40000Hz, 3=18000Hz, 4=8000Hz,
   * @details 5=4000Hz, 6=2000Hz, 7=1000Hz, 8=500Hz, 9=250Hz, 10=125Hz.
   * @param frequency_selector FCGEN selector (0..10 valid).
   */
  void setBIOZModulationFrequencybyIndex(uint8_t frequency_selector);

  /**
   * @brief Set BIOZ current magnitude.
   * @details  current in nanoAmps (8uA works with all settings, higher current is better for measuring low impedance)
   * @details  55..96,000 nano Amps
   * @details  8,000 **
   * @param current_nA Requested current in nA (mapped/clamped to valid table values). High-current (8uA..96uA) Low-current range (55nA..1100nA)
   */
  void setBIOZmag(uint32_t current_nA);

  /**
   * @brief Set BIOZ demodulation phase by selector.
   * @details   FCGEN=0000:  selector 0..3   (45.00deg steps) (128kHz)
   * @details   FCGEN=0001:  selector 0..7   (22.50deg steps) (80kHz)
   * @details   FCGEN>=0010: selector 0..15  (11.25deg steps) (40kHz and lower)
   * @param selector Phase index (range depends on FCGEN).
   */
   void setBIOZPhaseOffsetbyIndex(uint8_t selector);

   /**
   * @brief Set BIOZ demodulation phase by degree request.
   * @details phase delay between current and voltage measurement
   * @details for accurate phase and impedance reading at least two phase settings
   * @details   need to be measured, default is no phase shift, just measure real component
   * @details phase in degrees
   * @details    <80kHz  80kHz 128kHz
   * @details 0    0.00    0.0    0.0 **
   * @details 1   11.25   22.5   45.0
   * @details 2   22.50   45.0   90.0
   * @details 3   33.75   67.5  135.0
   * @details 4   45.00   90.0 
   * @details 5   56.25  112.5
   * @details 6   67.50  125.0
   * @details 7   78.75  147.5
   * @details 8   90.00  170.0
   * @details 9  101.25
   * @details 10 112.50
   * @details 11 123.75
   * @details 12 135.00
   * @details 13 146.25
   * @details 14 157.50
   * @details 15 168.75
   * @param phase_deg Requested phase in degrees.
   */
  void setBIOZPhaseOffsetbyPhase(float phase_deg);

  /**
   * @brief Set BIOZ demodulation phase by explicit frequency and phase.
   * @details Phase-index resolution depends on frequency band:
   * @details 128kHz uses 4 phases (45deg steps), 80kHz uses 8 phases (22.5deg steps),
   * @details 40kHz and below use 16 phases (11.25deg steps).
   * @param frequency_hz Frequency context in Hz.
   * @param phase_deg Requested phase in degrees.
   */
  void setBIOZPhaseOffsetbyPhase(uint16_t frequency_hz, float phase_deg);

  /**
   * @brief Configure BIOZ analog and digital filters.
   * @details ahpf (should be lower than selected modulation frequency)
   * @details  0    60 Hz
   * @details  1   150 Hz
   * @details  2   500 Hz
   * @details  3  1000 Hz
   * @details  4  2000 Hz
   * @details  5  4000 Hz
   * @details >=6  bypass
   * @details dlpf (set higher for fast changing signals, but more noise)
   * @details  0   0 Hz
   * @details  1   4 Hz **
   * @details  2   8 Hz
   * @details  3  16 Hz
   * @details dhpf (bypass or filter out static content to visualize cardiac induced changes)
   * @details  0   bypass **
   * @details  1   0.05Hz
   * @details  2   0.5Hz
   * @param ahpf Analog HPF selector (0..7). 0:60Hz 1:150Hz 2:500Hz* 3:1000Hz 4:2000Hz 5:4000Hz 6/7:bypass
   * @param lpf Digital LPF selector (0..3). 0:bypass 1:~4Hz* 2:~8Hz 3:~16Hz 
   * @param hpf Digital HPF selector (0..3). 0:bypass* 1:0.05Hz 2/3:0.5Hz
   */
  void setBIOZfilter(uint8_t ahpf, uint8_t lpf, uint8_t hpf);

  /**
   * @brief Set BIOZ current-generator modulation mode.
   * @details  0 = Unchopped with LPF * (higher noise, excellent 50/60 rejection, recommended for ECG, BIOZ applications)
   * @details  1 = Chopped without LPF (low noise, poor 50/60Hz rejection, recommended for BIOZ applications, possibly battery powered ECG))
   * @details  2 = Chopped with LPF (low noise, excellent 50/60Hz rejection)
   * @details  3 = Chopped with resistive CM (Not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance)
   * @details  Datasheet note: use mode 0 when BIOZ_HI_LOB=0 (low-current range).
   * @param mode BMUX_CG_MODE selector (0..3).
   */
  void setBIOZmodulation(uint8_t mode);

  /**
   * @brief Enable/disable BIOZ current monitor.
   * @param enable True to enable current monitor path.
   */
  void setBIOZCurrentMonitor(bool enable);

  // ---------------------------------------------------------------------------------------
  // Leads Configuration: max30001g_configure_leads.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Enable/disable ultra-low-power lead-on detection.
   * @details  Lead on is used in low power mode to assert interrupt when leads are 
   * @details  attached so that the device can automatically power on
   * @param enable True to enable ULP lead-on detection.
   */
  void setLeadsOnDetection(bool enable);

  /**
   * @brief Configure lead-off detection behavior.
   * @details Configure leads off detection for the ECG or BioZ channels.
   * @details The correct leads off detection features are automaticaly enabled (DC, Current and AC leads off)
   * @details  Before running this routine
   * @details  - Make sure  that ecg or bioz is active.
   * @details  - Lead polarity is set
   * @details  electrode_impedance: 
   * @details  - 0-2, 2-4, 4-10, 10-20, >20 MOhm
   * @details  - used to determine current magnitude, wet electrodes need higher current for lead-off detection
   * @param enable True to enable lead-off detection.
   * @param bioz_4 True for 4-wire BIOZ assumptions, false for 2-wire.
   * @param electrode_impedance Electrode impedance estimate in MOhm.
   */
  void setLeadsOffDetection(bool enable, bool bioz_4, uint8_t electrode_impedance);

  /**
   * @brief Configure lead-bias mode and resistance.
   * @details resistance: 
   * @details  0     use external lead bias (in BIOZ mode, only available for high current mode)
   * @details  >=150 internal lead bias 200M Ohm 
   * @details  >  75 internal lead bias 100M Ohm default
   * @details  else  internal lead bias 50M Ohm
   * @param enable True to enable lead-bias path.
   * @param resistance Resistance selector value (mapped to internal/external options).
   */
  void setLeadsBias(bool enable, uint8_t resistance);

  // ---------------------------------------------------------------------------------------
  // RtoR Configuration: max30001g_configure_rtor.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Apply default RTOR settings and enable RTOR detection.
   */
  void setDefaultRtoR(void);

  /**
   * @brief Apply default RTOR settings with RTOR disabled.
   * @details  Default RtoR parameters are:
   * @details   - enable: true
   * @details   - ptsf: 0b0011   (4/16) 
   * @details       This is the fraction of the Peak Average value used in the Threshold computation.
   * @details   - ptsf: 0b0101   (6/16) is also a common value for the threshold.
   * @details   - pavg: 0b10     (8) default but 2 might also be used, how many past peak values are used to update peak thershold
   * @details       This is the weighting factor for the current R-to-R peak observation vs. past peak observations when determining peak thresholds.
   * @details   - gain: 0b1111   (Auto-Scale)
   * @details   - wndw: 0b0011   12xRTOR_RES (~96ms)
   * @details   - hoff: 0b100000 (32)
   * @details   - ravg: 0b10     (8)
   * @details   - rhsf: 0b100    (4/8)
   */
   void setDefaultNoRtoR(void);

   /**
   * @brief Configure RTOR algorithm parameters.
   * @details ptsf: 0..15 maps to threshold scale 1/16..16/16.
   * @details pavg: 0..3  maps to peak averaging depth {2,4,8,16}.
   * @details gain: 0..15 maps to RTOR gain table (15 is auto-scale).
   * @details wndw: 0..11 valid QRS windows, 12..15 reserved.
   * @details hoff: 0..63 hold-off counts.
   * @details ravg: 0..3  maps to interval averaging depth {2,4,8,16}.
   * @details rhsf: 0..7  maps to dynamic hold-off scale 0/8..7/8.
   * @param enable True to enable RTOR detection.
   * @param ptsf Peak threshold scaling factor selector (0..15).
   * @param pavg Peak averaging selector (0..3).
   * @param gain RTOR gain selector (0..15).
   * @param wndw QRS averaging window selector (0..11 valid, 12..15 reserved).
   * @param hoff Minimum hold-off selector (0..63).
   * @param ravg Interval averaging selector (0..3).
   * @param rhsf Dynamic hold-off scaling selector (0..7).
   */
  void setRtoR(bool enable, uint8_t ptsf, uint8_t pavg, uint8_t gain, uint8_t wndw, uint8_t hoff, uint8_t ravg, uint8_t rhsf);

  // ---------------------------------------------------------------------------------------
  // Test Signal and Calibration: max30001g_calibration.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Disable ECG/BIOZ voltage calibration source routing.
   */
  void setDefaultNoTestSignal(void);
  
  /**
   * @brief Apply default ECG voltage calibration signal settings.
   */
  void setDefaultECGTestSignal(void);
  
  /**
   * @brief Apply default BIOZ voltage calibration signal settings.
   */
  void setDefaultBIOZTestSignal(void);
  
  /**
   * @brief Apply default voltage calibration settings to ECG and BIOZ.
   */
  void setDefaultECGandBIOZTestSignal(void);

  /**
   * @brief Configure calibration voltage source and waveform settings.
   * @details freq (FCAL): 0..7 maps to FMSTR/{128,512,2048,8192,32768,131072,524288,2097152}.
   * @details Example at FMSTR=32768Hz: {256,64,16,4,1,0.25,0.0625,0.015625}Hz.
   * @details dutycycle: 1..99% valid, 50% uses the dedicated CALP_SEL path.
   * @param enableECGCalSignal True to route VCAL to ECG path.
   * @param enableBIOZCalSignal True to route VCAL to BIOZ path.
   * @param unipolar True for unipolar waveform, false for bipolar waveform.
   * @param cal_vmag False for 0.25mV class, true for 0.5mV class.
   * @param freq FCAL selector (0..7).
   * @param dutycycle Duty cycle in percent (1..99, 50 uses dedicated mode).
   */
  void setTestSignal(
    bool enableECGCalSignal,
    bool enableBIOZCalSignal,
    bool unipolar,
    bool cal_vmag,
    uint8_t freq,
    uint8_t dutycycle
  );

  /**
   * @brief Apply default internal BIOZ test impedance settings.
   */
  void setDefaultBIOZTestImpedance(void);

  /**
   * @brief Disable internal BIOZ test impedance path.
   */
  void setDefaultNoBIOZTestImpedance(void);

  /**
   * @brief Configure BIOZ test impedance and optional modulation.
   * @param enable True to enable test impedance.
   * @param useHighResistance True for high-resistance RNOM table.
   * @param enableModulation True to enable RMOD modulation.
   * @param rnomValue RNOM selector.
   * @param rmodValue RMOD selector.
   * @param modFreq Modulation frequency selector (0..3).
   * @details useHighResistance=true, rnomValue 0..3 -> 27k,108k,487k,1029k
   * @details   0  00 =   27 kΩ
   * @details   1  01 =  108 kΩ
   * @details   2  10 =  487 kΩ
   * @details   3  11 = 1029 kΩ
   * @details useHighResistance=false, rnomValue 0..7 -> 5k..625ohm
   * @details   0 000 =    5 kΩ
   * @details   1 001 =    2.5 kΩ
   * @details   2 010 =    1.667 kΩ
   * @details   3 011 =    1.25 kΩ
   * @details   4 100 =    1 kΩ
   * @details   5 101 =    0.833 kΩ
   * @details   6 110 =    0.714 kΩ
   * @details   7 111 =    0.625 kΩ
   * @details enableModulation=true enables RMOD with rmodValue and modFreq.
   * @details    For low resistance values between 625Ω and 5kΩ, the resistance can be modulated. 
   * @details    Resistance will switch between RNOM (resistance) and (RNOM - RMOD) at the selected modulation rate. 
   * @details - rmodValue: Modulated resistance setting (0 to 7, RMOD). Only applicable when using low resistance.    
   * @details - modFreq: resistance modulation frequency (0 to 3). Only applicable when using low resistance.
   * @details     0=4Hz, 1=1Hz, 2=0.25Hz, 3=0.0625Hz
   * @details BMUX_RNOM[2:0]  BMUX_RMOD[2:0]  NOMINAL RESISTANCE (Ω)  MODULATED RESISTANCE (mΩ)
   * @details    000             000             5000                    2960
   * @details                    001                                     980.5
   * @details                    010                                     247.5
   * @details                    1xx                                     none
   * @details    001             000             2500                    740.4
   * @details                    001                                     245.2
   * @details                    010                                     61.9
   * @details                    1xx                                     none
   * @details    010             000             1667                    329.1
   * @details                    001                                     109.0
   * @details                    010                                     27.5
   * @details                    1xx                                     none
   * @details    011             000             1250                    185.1
   * @details                    001                                     61.3
   * @details                    010                                     none
   * @details                    1xx                                     none
   * @details    100             000             1000                    118.5
   * @details                    001                                     39.2
   * @details                    010                                     none
   * @details                    1xx                                     none
   * @details    101             000              833                    82.3
   * @details                    001                                     27.2
   * @details                    010                                     none
   * @details                    1xx                                     none
   * @details    110             000              714                    60.5
   * @details                    001                                     20.0
   * @details                    010                                     none
   * @details                    1xx                                     none
   * @details    111             000              625                    46.3
   * @details                    001                                     15.3
   * @details                    010                                     none
   * @details                    1xx                                     none
   */
  void setBIOZTestImpedance(
    bool enable,
    bool useHighResistance,
    bool enableModulation,
    uint8_t rnomValue,
    uint8_t rmodValue,
    uint8_t modFreq
  );

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
private:

  // ---------------------------------------------------------------------------------------
  // Register related: max30001g_regs.cpp
  // ---------------------------------------------------------------------------------------
  
  /**
   * @brief Print one EN_INT register decode.
   * @param en_int EN_INT register mirror to print.
   */
  void printEN_INT(max30001_en_int_reg_t en_int);
  
  /* @brief Print EN_INT1 register decode. */
  void printEN_INT1(void) { printEN_INT(en_int1); }
  
  /* @brief Print EN_INT2 register decode. */
  void printEN_INT2(void) { printEN_INT(en_int2); }
  
  /* @brief Print MNGR_INT register decode. */
  void printMNGR_INT(void);
  
  /* @brief Print MNGR_DYN register decode. */
  void printMNGR_DYN(void);

  /* @brief Print CNFG_GEN register decode. */
  void printCNFG_GEN(void);
  
  /* @brief Print CNFG_CAL register decode. */
  void printCNFG_CAL(void);
  
  /* @brief Print CNFG_EMUX register decode. */
  void printCNFG_EMUX(void);
  
  /* @brief Print CNFG_ECG register decode. */
  void printCNFG_ECG(void);
  
  /* @brief Print CNFG_BMUX register decode. */
  void printCNFG_BMUX(void);
  
  /* @brief Print CNFG_BIOZ register decode. */
  void printCNFG_BIOZ(void);
  
  /* @brief Print CNFG_BIOZ_LC register decode. */
  void printCNFG_BIOZ_LC(void);
  
  /* @brief Print CNFG_RTOR1 register decode. */
  void printCNFG_RTOR1(void);
  
  /* @brief Print CNFG_RTOR2 register decode. */
  void printCNFG_RTOR2(void);

  // ---------------------------------------------------------------------------------------
  // System: max30001g_system.h
  // ---------------------------------------------------------------------------------------

  // Read registers and update global variables based on register values
  
  /** @brief Update computed ECG sampling-rate global from current registers. */
  void updateGlobalECG_samplingRate(void);
  
  /** @brief Update computed BIOZ sampling-rate global from current registers. */
  void updateGlobalBIOZ_samplingRate(void);
  
  /* @brief Update calibration frequency global from current registers. */
  void updateGlobalCAL_fcal(void);
  
  /* @brief Update ECG latency global from current timing/filter state. */
  void updateGlobalECG_latency(void);
  
  /* @brief Update BIOZ modulation-frequency global from current registers. */
  void updateGlobalBIOZ_frequency(void);
  
  /* @brief Update BIOZ test-impedance globals from current registers. */
  void updateGlobalBIOZ_test_impedance(void);
  
  /* @brief Update ECG LPF frequency global from current registers. */
  void updateGlobalECG_lpf(void);
  
  /* @brief Update BIOZ DLPF frequency global from current registers. */
  void updateGlobalBIOZ_dlpf(void);

  /* @brief Update RCAL modulation-frequency global from current registers. */
  void updateGlobalRCAL_freq(void);

  /* @brief Recompute timing-dependent globals from current register state. */
  void refreshTimingGlobals(void);

  // ---------------------------------------------------------------------------------------
  // Read Data: max30001g_readdata.cpp (internal single-sample helpers)
  // ---------------------------------------------------------------------------------------

  // ---------------------------------------------------------------------------------------
  // Interrupts: max30001g_interrupt.cpp
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Internal interrupt-line configuration helper.
   * @details interrupt: use MAX30001_EN_INT1 or MAX30001_EN_INT2.
   * @param interrupt Interrupt register selector (`MAX30001_EN_INT1` or `MAX30001_EN_INT2`).
   * @param ecg Enable ECG-related interrupt bits.
   * @param bioz Enable BIOZ-related interrupt bits.
   * @param rtor Enable RTOR interrupt bit.
   * @param leadon Enable lead-on interrupt bit.
   * @param leadoff Enable lead-off interrupt bit.
   * @param bioz_fourwire Enable 4-wire BIOZ current monitor interrupt behavior.
   * @return True when register write/verify succeeds.
   */
  bool setInterrupt(uint8_t interrupt, bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire);

  /**
   * @brief Latch software flags from a STATUS register snapshot.
   * @param statusWord Raw STATUS[23:0] bits.
   */
  void latchStatusFlags(uint32_t statusWord);

  /**
   * @brief Read STATUS and latch software flags.
   * @return STATUS[23:0] snapshot that was read.
   */
  uint32_t readStatusAndLatchFlags(void);

  // ---------------------------------------------------------------------------------------
  // SPI Communication: max30001g_comm.cpp
  // ---------------------------------------------------------------------------------------
  /**
   * @brief Write one 24-bit register value.
   * @param address Register address.
   * @param data 24-bit payload value.
   */
  void writeRegister(uint8_t address, uint32_t data);

  /**
   * @brief Read one 24-bit register value.
   * @param address Register address.
   * @return Register value  24 bits (in 32bit container).
   */
  uint32_t readRegister24(uint8_t address);

  /**
   * @brief Read one 24 bit register value. Alias for readRegister24.
   * @param address Register address.
   * @return Register value in lower 24 bits.
   */
  uint32_t readRegister(uint8_t address) { return readRegister24(address); }

  /**
   * @brief Read one 8 bit register.
   * @param address Register address.
   * @return Register value 8 bit.
   */
  uint8_t readRegisterByte(uint8_t address);

  /**
   * @brief Read one 8 bit register value. Alias for readRegisterByte.
   * @param address Register address.
   * @return Register value 8 bits.
   */
  uint8_t readRegister8(uint8_t address) { return readRegisterByte(address); };

  /**
   * @brief Verify register write by readback comparison.
   * @param address Register address.
   * @param expectedValue Expected 24-bit value.
   * @return True when readback matches expected value.
   */
  bool verifyWrite(uint8_t address, uint32_t expectedValue);

  /**
   * @brief Verify register write by readback comparison. Alias for verifyWrite().
   * @param address Register address.
   * @param expectedValue Expected 24-bit value.
   * @return True when readback matches expected value.
   */
  bool verifRegister(uint8_t address, uint32_t expectedValue) { return verifyWrite(address, expectedValue); }

  /**
   * @brief Perform SPI communication sanity check.
   * @return True when SPI path appears functional.
   */
  bool spiCheck();

  // ---------------------------------------------------------------------------------------
  // BIOZ: Impedance spectroscopy helper functions
  // ---------------------------------------------------------------------------------------

  /**
   * @brief Quantize requested drive current to nearest supported setting.
   * @param current Requested current value.
   * @return Closest supported current value.
   */
  int32_t closestCurrent(int32_t current);

  /**
   * @brief Parametric impedance model. Input is impedance and phase of sample and demodulation offset theta.
   * @param theta Demodulation phase-offset input in degrees.
   * @param magnitude Impedance magnitude of sample
   * @param phase Phase parameter of sample in degrees.
   * @return Modeled impedance projection at demodulation phase offset.
   */
  float impedancemodel(float theta, float magnitude, float phase);

  /**
   * @brief Fit impedance magnitude/phase from multi-phase measurements.
   * @param phases Array of demodulation phase offsets (degrees).
   * @param impedances Array of measured impedance values at the corresponding phase offsets.
   * @param num_points Number of valid measurements (array size).
   * @return Fitted impedance model parameters.
   */
  ImpedanceModel fitImpedance(const float* phases, const float* impedances, int num_points);

  uint8_t biozScanPhaseCountForFreq(uint8_t freq_idx) const;
  uint8_t biozScanSelectAHpf(float lowest_frequency_hz) const;
  float biozScanRobustMeanFromBuffer(uint8_t outlier_min_samples, float outlier_sigma, bool& hasSamples);

  // ---------------------------------------------------------------------------------------
  // Device
  // ---------------------------------------------------------------------------------------
  enum SetupProfile : uint8_t {
    PROFILE_NONE = 0,
    PROFILE_ECG,
    PROFILE_ECG_CAL,
    PROFILE_BIOZ,
    PROFILE_BIOZ_CAL,
    PROFILE_BIOZ_IMP_CAL,
    PROFILE_ECG_AND_BIOZ,
    PROFILE_BIOZ_SCAN
  };

  enum BiozScanState : uint8_t {
    BIOZ_SCAN_IDLE = 0,
    BIOZ_SCAN_INIT,
    BIOZ_SCAN_CONFIG_FREQ,
    BIOZ_SCAN_PREPARE_ATTEMPT,
    BIOZ_SCAN_CONFIG_PHASE,
    BIOZ_SCAN_WAIT_DATA,
    BIOZ_SCAN_PROCESS_PHASE,
    BIOZ_SCAN_EVALUATE_ATTEMPT,
    BIOZ_SCAN_FINALIZE_FREQ,
    BIOZ_SCAN_FINISH
  };

  int _csPin;   // chip select for SPI
  int _intPin1; // interrupt pin INTB (EN_INT1)
  int _intPin2; // interrupt pin INT2B (EN_INT2), optional

  /**
   * @brief ISR shim for INTB pin edge.
   */
  static void onAFE_IRQ1(void);
  /**
   * @brief ISR shim for INT2B pin edge.
   */
  static void onAFE_IRQ2(void);

  SetupProfile _profile;
  bool _configured;
  bool _running;

  bool _useECG;
  bool _useBIOZ;
  bool _useRTOR;

  bool _drainECGOnUpdate;
  bool _drainBIOZOnUpdate;
  bool _readRTOROnUpdate;

  bool _applyLeadSettingsOnStart;
  bool _leadBiasEnable;
  uint8_t _leadBiasResistance;
  bool _leadOffEnable;
  bool _leadOffBioz4;
  uint8_t _leadOffElectrodeImpedance;
  bool _leadOnEnable;

  ConfigSnapshot _savedConfig;

  // BIOZ scan current profile cache (for reuse across scans)
  bool _scanCurrentProfileValid;
  int32_t _scanCurrentProfile[MAX30001_BIOZ_NUM_FREQUENCIES];
  uint8_t _scanProfileFreqStart;
  uint8_t _scanProfileFreqEnd;
  bool _scanProfileFast;

  BIOZScanConfig _scanRuntimeConfig;
  bool _scanReuseCurrents;
  bool _scanInProgress;
  bool _scanCompleted;

  BiozScanState _BIOZScanState;
  uint8_t _scanFreqIndex;
  uint8_t _scanPhaseIndex;
  uint8_t _scanAttempt;
  uint8_t _scanNumPhaseMeasurements;
  uint32_t _scanWaitDeadlineMs;

  float _scanThresholdMin;
  float _scanThresholdMax;
  float _scanAdcTarget;

  bool _scanSawValid;
  bool _scanSawInvalidOrRange;
  bool _scanAnyAboveTarget;
  bool _scanAnyInTarget;
  float _scanMaxAbsRaw;

  int32_t _scanCurrent[MAX30001_BIOZ_NUM_FREQUENCIES];
  float _scanFrequency[MAX30001_BIOZ_NUM_FREQUENCIES];
  float _scanImpedance[MAX30001_BIOZ_NUM_FREQUENCIES][MAX30001_BIOZ_NUM_PHASES];
  float _scanPhaseDeg[MAX30001_BIOZ_NUM_FREQUENCIES][MAX30001_BIOZ_NUM_PHASES];
};

/** @} */ // end of MAX30001G_Public_API

#endif // MAX30001G_H
