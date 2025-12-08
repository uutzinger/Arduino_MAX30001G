/******************************************************************************************************/
// Reading Data
/******************************************************************************************************/
#include "logger.h"         // Logging 
#include "max30001g_globals.h"
#include "max30001g_defs.h" // MAX30001G
#include "max30001g_comm.h" // SPI communication
#include "max30001g_readdata.h"

/******************************************************************************************************/
// FIFO Data Handling
/******************************************************************************************************/
void MAX30001G::FIFOReset(void) {
  /*
   * Resets the ECG and BIOZ FIFOs by writing to the FIFO_RST register.
   * Does not affect measurement process not affected
   */

  status.all = readRegister(MAX30001_STATUS);
  if (this-status.bit.eovf == 1 || status.bit.bovf == 1) {
    LOGE("MAX30001G: FIFO overflow reset failed, overflow flags are still set.");
  } else {
    LOGD("MAX30001G: FIFO overflow reset successful, overflow flags are cleared.");
  }

  // FIFO_RST register (0x0A) is used to reset both ECG and BIOZ FIFOs.
  LOGD("MAX30001G: Resetting ECG and BIOZ FIFOs.");

  // Write to the FIFO_RST register (0x0A) to reset the FIFO pointers
  writeRegister(MAX30001_FIFO_RST, 0x000000);

}

/******************************************************************************************************/
// R to R reading
/******************************************************************************************************/

void MAX30001G::readHRandRR(void) {
/*
 * Read the RTOR register to calculate the heart rate and RR interval
 *   Call this routine when RTOR interrupt occured
 */
  rtor.all = readRegister24(MAX30001_RTOR);
  rr_interval = float(rtor.data) * RtoR_resolution; // in [ms]
  heart_rate = 60000./rr_interval; // in beats per minute 
}

/******************************************************************************************************/
// ECG and BIOZ reading
/******************************************************************************************************/

float MAX30001G::readECG(bool reportRaw) {
 /*
  * Read the ECG data register
  */
  
  max30001_ecg_burst_t ecg_data; // FIFO burst and regular read have same register structure
  float fdata;

  ecg_data = readRegister24(MAX30001_ECG_FIFO);

  readStatusRegisters();

  LOGD("ECG Tag: %3u", ecg_data.bit.etag);

  if (ecg_data.bit.etag == 0 || ecg_data.bit.etag == 2) {     // 0,2 are valid samples
    int32_t sdata = (int32_t)(ecg_data.bit.data << 14) >> 14; // Sign extend the data 20 bit
    if (reportRaw) {
      fdata = float(sdata);
    } else {
      //   V_ECG (mV) = ADC x VREF / (2^17 x ECG_GAIN): ECG_GAIN is 20V/V, 40V/V, 80V/V or 160V/V. 
      fdata = float(sdata * V_ref) / float(131072 * ECG_gain);
    }
  } else {
    fdata = INFINITY; //
    sdata = INT32_MAX;   // 
  }
  LOGD("ECG Sample: %.2f [mV] %d", fdata, sdata)

  // over_voltage_detected;
  // under_voltage_detected;
  // valid_data_detected;
  // EOF_detected;

  switch (ecg_data.bit.btag) {
  
    case 0b000: // valid
      valid_data_detected = true;
      break;
    case 0b001: // transient
      valid_data_detected = false;
      break;
    case 0b010: // valid and last sample in FIFO
      valid_data_detected = true;
      EOF_data_detected = true;
      break;
    case 0b011: // last and transient sample in FIFO
      valid_data_detected = false;
      EOF_data_detected = true;
      break;
    case 0b110: // attempt to read empty FIFO
      valid_data_detected = false;
      EOF_data_detected = true;
      break;
    case 0b111: // FIFO overflow occurred
      valid_data_detected = false;
      FIFOReset();
    default:
      valid_data_detected = false;
      EOF_data_detected = false;
      LOGE("Invalid btag value: %d", bioz_data.bit.btag);
      break
  }
    
  return fdata;
}

float MAX30001G::readBIOZ(bool reportRaw) {
/*
 * Read the BIOZ data register
 */
  
  max30001_bioz_burst_t bioz_data; // FIFO burst and regular read have same register structure
  float fdata;

  bioz_data = readRegister24(MAX30001_BIOZ_FIFO);
  readStatusRegisters();

  LOGD("BIOZ Tag: %3u", bioz_data.bit.btag);

  if (bioz_data.bit.btag == 0 || bioz_data.bit.btag == 2) {     // 0,2 are valid samples
    int32_t sdata = (int32_t)(bioz_data.bit.data << 12) >> 12; // Sign extend the data 20 bit
    if (reportRaw) {
      fdata = float(sdata);
    } else {
      // Fill the buffer with calibrated values
      // BioZ (Ω) = ADC x VREF / (2^19 x BIOZ_CGMAG x BIOZ_GAIN)
      //   BIOZ_CGMAG is 55 to 96000 nano A, set by CNFG_BIOZ and CNFG_BIOZ_LC 
      //   BIOZ_GAIN = 10V/V, 20V/V, 40V/V, or 80V/V. BIOZ_GAIN are set in CNFG_BIOZ (0x18).
      fdata = float(sdata * V_ref) / (float(524288* BIOZ_cgmag * BIOZ_gain) * 1e-9); // in Ω
    }
  } else {
    fdata = INFINITY; //
    sdata = INT32_MAX;   // 
  }
  LOGD("BIOZ Sample: %.2f [Ohm] %d", fdata, sdata)

  // over_voltage_detected;
  // under_voltage_detected;
  // valid_data_detected;
  // EOF_detected;

  switch (bioz_data.bit.btag) {
  
    case 0b000: // valid
      valid_data_detected = true;
      break;
    case 0b001: // transient
      valid_data_detected = false;
      break;
    case 0b010: // valid and last sample in FIFO
      valid_data_detected = true;
      EOF_data_detected = true;
      break;
    case 0b011: // last and transient sample in FIFO
      valid_data_detected = false;
      EOF_data_detected = true;
      break;
    case 0b110: // attempt to read empty FIFO
      valid_data_detected = false;
      EOF_data_detected = true;
      break;
    case 0b111: // FIFO overflow occurred
      valid_data_detected = false;
      FIFOReset();
    default:
      valid_data_detected = false;
      EOF_data_detected = false;
      LOGE("Invalid btag value: %d", bioz_data.bit.btag);
      break
  }
    
  return fdata;
}

/* BURST READ */ 
/* -------------------------------------------------*/

void MAX30001G::readECG_FIFO(bool reportRaw) {
/*
 * Burst read of the ECG FIFO:
 * Call this routine when ECG FIFO interrupt occurred
 */
  
  // Obtain number of samples in the FIFO buffer
  mngr_int.all = readRegister24(MAX30001_MNGR_INT);
  uint8_t  num_samples = (mngr_int.e_fit + 1);
    
  max30001_ecg_burst_reg_t ecg_data;
  float fdata;
  
  // Burst read FIFO register
  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(_csPin, LOW);
  SPI.transfer((ECG_FIFO_BURST << 1) | 0x01);
  ecg_counter = 0;
  bool overflow = false;
  
  for (int i = 0; i < num_samples; i++) {
    ecg_data.all = ((uint32_t)SPI.transfer(0x00) << 16) | // top 8 bits
                   ((uint32_t)SPI.transfer(0x00) << 8)  | // middle 8 bits
                   (uint32_t)SPI.transfer(0x00);         // low 8 bits
    LOGD("ECG Tag: %3u", ecg_data.bit.etag);
    // check for valid sample, etag = 0 or 2 marks valid data
    if (ecg_data.bit.etag == 0 || ecg_data.bit.etag == 2) {
      int32_t sdata = (int32_t)(ecg_data.bit.data << 14) >> 14; // Sign extend the data 18 bit
      if (reportRaw) {
        fdata = float(sdata);
      } else {
        // Fill the ring buffer with calibrated values
        //   V_ECG (mV) = ADC x VREF / (2^17 x ECG_GAIN): ECG_GAIN is 20V/V, 40V/V, 80V/V or 160V/V. 
        fdata = float(sdata * V_ref) / float(131072 * ECG_gain);
      }
      if ( ECG_data.push(fdata, overflow=false) == 1) { // push one value to ring buffer
        ecg_counter++;
      } else {
        LOGE("ECG buffer write  overflow");
      }
      LOGD("ECG Sample: %.2f [mV] %d", fdata, sdata);
    } else if (ecg_data.bit.etag == 0x07) { 
      // Check for device FIFO Overflow
      overflow = true;
      break;
    }
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  if (overflow) { 
    LOGW("ECG FIFO overflow detected, resetting FIFO");
    FIFOReset(); 
  }
  LOGD("Read %3u samples from ECG FIFO ", ecg_counter);
}

void MAX30001G::readBIOZ_FIFO(bool reportRaw) {
/*
 * Burst read of the BIOZ FIFO
 *  Call this routine when BIOZ FIFO interrupt occurred
 */
  
  // Obtain number of samples in the FIFO buffer
  mngr_int.all = readRegister24(MAX30001_MNGR_INT);
  uint8_t  num_samples = (mngr_int.b_fit +1);
  
  max30001_bioz_burst_t bioz_data;
  float fdata;

  // Burst read FIFO register
  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(_csPin, LOW);
  SPI.transfer((BIOZ_FIFO_BURST << 1) | 0x01);
  bioz_counter = 0;
  bool overflow = false;

  for (int i = 0; i < num_samples; i++) {
    bioz_data.all = ((uint32_t)SPI.transfer(0x00) << 16) | // to 8 bits
                    ((uint32_t)SPI.transfer(0x00) << 8)  | // middle 8 bits
                      (uint32_t)SPI.transfer(0x00);         // low 8 bits
    LOGD("BIOZ Tag: %3u", bioz_data.bit.btag);
    if (bioz_data.bit.btag == 0 || bioz_data.bit.btag == 2) {     // 0,2 are valid samples
      int32_t sdata = (int32_t)(bioz_data.bit.data << 12) >> 12; // Sign extend the data 20 bit
      if (reportRaw) {
        fdata = float(sdata);
      } else {
        // Fill the buffer with calibrated values
        // BioZ (Ω) = ADC x VREF / (2^19 x BIOZ_CGMAG x BIOZ_GAIN)
        //   BIOZ_CGMAG is 55 to 96000 nano A, set by CNFG_BIOZ and CNFG_BIOZ_LC 
        //   BIOZ_GAIN is 10V/V, 20V/V, 40V/V, or 80V/V, set in CNFG_BIOZ (0x18)
        fdata = float(sdata * V_ref * 1e9) / float(524288* BIOZ_cgmag * BIOZ_gain); // in Ω
      }
      if (BIOZ_data.push(fdata) == 1) { 
        bioz_counter++;
      } else {
        LOGE("BIOZ buffer write  overflow");
      }        
      LOGD("BIOZ Sample: %.2f [Ohm] %d", fdata, sdata)
    } else if (bioz_data.bit.btag == 0x07) { 
      // Check for device FIFO Overflow
      overflow = true;
      break;
    }
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  if (overflow) { FIFOReset(); }
  LOGD("Read %3u samples from BIOZ FIFO ", bioz_counter);
}
