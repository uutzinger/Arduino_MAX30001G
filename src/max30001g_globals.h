#ifndef MAX30001G_GLOBALS_H
#define MAX30001G_GLOBALS_H

#include <stdint.h>
#include "RingBuffer.h"
#include "max30001g_defs.h"
#include "max30001g_typedefs.h"
#include "max30001g_regs_typedefs.h"

// Interrupt Flags
extern volatile bool ecg_available;
extern volatile bool bioz_available;
extern volatile bool rtor_available;


// Data Buffers
extern RingBuffer<float, 128> ECG_data;
extern RingBuffer<float, 128> BIOZ_data;
extern RingBuffer<float, 16> RTOR_data;
extern RingBuffer<ImpedanceSpectrum, 4> BIOZ_spectrum;

extern int ecg_counter;
extern int bioz_counter;
extern int rtor_counter;
extern float rr_interval;

// Impedance Measurement Data
extern float impedance_magnitude[MAX30001_BIOZ_NUM_FREQUENCIES];
extern float impedance_phase[MAX30001_BIOZ_NUM_FREQUENCIES];
extern float impedance_frequency[MAX30001_BIOZ_NUM_FREQUENCIES];

// Error & Status Flags
extern volatile bool ecg_lead_off;
extern volatile bool ecg_fast_recovery_occurred;
extern volatile bool ecg_overflow_occurred;
extern volatile bool bioz_cgm_occurred;
extern volatile bool bioz_undervoltage_occurred;
extern volatile bool bioz_overvoltage_occurred;
extern volatile bool bioz_overflow_occurred;
extern volatile bool leads_on_detected;
extern volatile bool pll_unlocked_occurred;

extern volatile bool afe_irq_pending;
extern volatile bool afe_irq1_pending;
extern volatile bool afe_irq2_pending;

extern bool over_voltage_detected;
extern bool under_voltage_detected;
extern bool valid_data_detected;
extern bool EOF_detected;

// MAX30001 Timing and Measurement Settings
extern float    ECG_samplingRate;
extern float    BIOZ_samplingRate;
extern float    RtoR_resolution;
extern float    RtoR_delay;
extern float    CAL_resolution;
extern float    CAL_fcal;
extern float    fmstr;
extern float    tres;
extern float    ECG_progression;
extern float    ECG_hpf;
extern float    ECG_lpf;
extern float    ECG_latency;
extern int      ECG_gain;
extern int      BIOZ_gain;
extern int      BIOZ_cgmag;
extern uint32_t BIOZ_cmres;
extern uint16_t RBIASV_res;
extern float    BIOZ_frequency;
extern float    BIOZ_phase;
extern float    BIOZ_ahpf;                       // [Hz]   BIOZ analog high pass filter
extern float    BIOZ_dlpf;                       // [Hz]   BIOZ digital low pass filter
extern float    BIOZ_dhpf;                       // [Hz]   BIOZ digital high pass filter
extern float    BIOZ_test_rnom;                  // [kOhm] BIOZ test resistor nominal value
extern float    BIOZ_test_rmod;                  // [kOhm] BIOZ test resistor modulation value
extern float    BIOZ_test_frequency;             // [Hz]   BIOZ test frequency for modulation
extern int32_t  V_ref;
extern int32_t  V_AVDD;
extern float    RCAL_freq;

// MAX30001 Configuration and Status Registers
extern max30001_status_reg_t       status;
extern max30001_en_int_reg_t       en_int1;
extern max30001_en_int_reg_t       en_int2;
extern max30001_mngr_int_reg_t     mngr_int;
extern max30001_mngr_dyn_reg_t     mngr_dyn;
extern max30001_info_reg_t         info;
extern max30001_cnfg_gen_reg_t     cnfg_gen;
extern max30001_cnfg_cal_reg_t     cnfg_cal;
extern max30001_cnfg_emux_reg_t    cnfg_emux;
extern max30001_cnfg_ecg_reg_t     cnfg_ecg;
extern max30001_cnfg_bmux_reg_t    cnfg_bmux;
extern max30001_cnfg_bioz_reg_t    cnfg_bioz;
extern max30001_cnfg_bioz_lc_reg_t cnfg_bioz_lc;
extern max30001_cnfg_rtor1_reg_t   cnfg_rtor1;
extern max30001_cnfg_rtor2_reg_t   cnfg_rtor2;

#endif // MAX30001G_GLOBALS_H
