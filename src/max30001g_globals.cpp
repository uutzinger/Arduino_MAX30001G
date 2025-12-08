/******************************************************************************************************/
// Global variables for the MAX30001G library
/******************************************************************************************************/

#include "max30001g_globals.h"

// Variables
// ------------------------------------------------------------------------
volatile bool ecg_available;             // ECG data available interrupt
volatile bool bioz_available;            // BIOZ data available interrupt
volatile bool rtor_available;            // R to R data available interrupt

RingBuffer<float 128> ECG_data;
RingBuffer<float 128> BIOZ_data;
int        ecg_counter;                  // number of ECG samples in buffer
int        bioz_counter;                 // number of BIOZ samples in buffer
float      heart_rate;                   // in beats per minute
float      rr_interval;                  // in milliseconds

float impedance_magnitude[8];
float impedance_phase[8]; 
float impedance_frequency[8];

volatile bool ecg_lead_off;               // ECG lead off detection interrupt
volatile bool ecg_overflow_occurred;      // ECG FIFO overflow
volatile bool bioz_cgm_occurred;          // BIOZ current generator monitor
volatile bool bioz_undervoltage_occurred; // BIOZ under voltage
volatile bool bioz_overvoltage_occurred;  // BIOZ over voltage
volatile bool bioz_overflow_occurred;     // BIOZ FIFO overlow
volatile bool leads_on_detected;          // Ultra low power leads on
volatile bool pll_unlocked_occurred;      // PLL is not locked

bool over_voltage_detected;
bool under_voltage_detected;
bool valid_data_detected;
bool EOF_detected;

// Status of the MAX30001 timing
// and measurement settings
// -----------------------------
float    ECG_samplingRate;               // [sps]
float    BIOZ_samplingRate;              // [sps] 
float    RtoR_resolution;                // [ms] R to R resolution time
float    CAL_resolution;                 // [ms] calibration resolition time
float    CAL_fcal;                       // 
float    fmstr;                          // [Hz] main clock frequency
float    tres;                           // [micro s] R to R peak detection threshold
float    ECG_progression;                // [Hz] ECG progression rate
float    ECG_hpf;                        // [Hz] ECG high pass filter
float    ECG_lpf;                        // [Hz] ECG low pass filter
int      ECG_gain;                       // [V/V] ECG gain
int      BIOZ_gain;                      // [V/V] BIOZ gain
int      BIOZ_cgmag;                     // [nano A] BIOZ current generator magnitude 
int      BIOZ_cmres;                     // [kOhm] BIOZ common mode feedback resistance for low current mode
int      RBIASV_res;                     // [MOhm] common mode feedback resistance for ECG and BIOZ high current mode
float    BIOZ_frequency;                 // [HZ]
float    BIOZ_phase;                     // [degrees]
float    BIOZ_ahpf;                      // [Hz] BIOZ high pass filter
float    BIOZ_dhpf;                      // [Hz] BIOZ low pass filter
float    BIOZ_dlpf;                      // [Hz] BIOZ low pass filter
float    BIOZ_test_rnom;                 // [kOhm] BIOZ test resistor nominal value
float    BIOZ_test_rmod;                 // [kOhm] BIOZ test resistor modulation value
float    BIOZ_test_frequency;            // [Hz] BIOZ test frequency for modulation
uint32_t BIOZ cmres;                     // [kOhm] common mode feedback resistance
int32_t  V_ref = 1000;                   // [mV]
int32_t  V_AVDD = 1800;                  // [mV]
float    RCAL_freq = 0;                  // Hz resistance modulation for test impedance

// All Configuration and Status Registers

max30001_status_reg_t       status;       // 0x01
max30001_en_int_reg_t       en_int1;      // 0x02
max30001_en_int_reg_t       en_int2;      // 0x03
max30001_mngr_int_reg_t     mngr_int;     // 0x04
max30001_mngr_dyn_reg_t     mngr_dyn;     // 0x05
max30001_info_reg_t         info;         // 0x0f
max30001_cnfg_gen_reg_t     cnfg_gen;     // 0x10
max30001_cnfg_cal_reg_t     cnfg_cal;     // 0x12
max30001_cnfg_emux_reg_t    cnfg_emux;    // 0x14
max30001_cnfg_ecg_reg_t     cnfg_ecg;     // 0x15
max30001_cnfg_bmux_reg_t    cnfg_bmux;    // 0x17
max30001_cnfg_bioz_reg_t    cnfg_bioz;    // 0x18
max30001_cnfg_bioz_lc_reg_t cnfg_bioz_lc; // 0x1A
max30001_cnfg_rtor1_reg_t   cnfg_rtor1;   // 0x1D
max30001_cnfg_rtor2_reg_t   cnfg_rtor2;   // 0x1D