/******************************************************************************************************/
// Reading Data
/******************************************************************************************************/
#include <Arduino.h>
#include <math.h>
#include <limits>
#include "logger.h"
#include "max30001g.h"

namespace {

constexpr uint8_t ECG_TAG_VALID      = 0b000;
constexpr uint8_t ECG_TAG_FAST       = 0b001;
constexpr uint8_t ECG_TAG_VALID_EOF  = 0b010;
constexpr uint8_t ECG_TAG_FAST_EOF   = 0b011;
constexpr uint8_t ECG_TAG_EMPTY      = 0b110;
constexpr uint8_t ECG_TAG_OVERFLOW   = 0b111;

constexpr uint8_t BIOZ_TAG_VALID      = 0b000;
constexpr uint8_t BIOZ_TAG_RANGE      = 0b001;
constexpr uint8_t BIOZ_TAG_VALID_EOF  = 0b010;
constexpr uint8_t BIOZ_TAG_RANGE_EOF  = 0b011;
constexpr uint8_t BIOZ_TAG_EMPTY      = 0b110;
constexpr uint8_t BIOZ_TAG_OVERFLOW   = 0b111;

inline int32_t signExtend(uint32_t value, uint8_t bitWidth) {
  const uint32_t signBit = 1UL << (bitWidth - 1U);
  const uint32_t mask    = (1UL << bitWidth) - 1UL;
  value &= mask;
  return static_cast<int32_t>((value ^ signBit) - signBit);
}

inline float ecgCodeToMilliVolt(int32_t code) {
  //   V_ECG (mV) = ADC x VREF / (2^17 x ECG_GAIN): ECG_GAIN is 20V/V, 40V/V, 80V/V or 160V/V. 
  const float denom = static_cast<float>(131072.0f * static_cast<float>(ECG_gain));
  if (denom == 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  return (static_cast<float>(code) * static_cast<float>(V_ref)) / denom;
}

inline float biozCodeToOhm(int32_t code) {  
  // BioZ (Ω) = ADC x VREF / (2^19 x BIOZ_CGMAG x BIOZ_GAIN)
  //   BIOZ_CGMAG is 55 to 96000 nano A, set by CNFG_BIOZ and CNFG_BIOZ_LC 
  //   BIOZ_GAIN = 10V/V, 20V/V, 40V/V, or 80V/V. BIOZ_GAIN are set in CNFG_BIOZ (0x18).
  const float denom = 524288.0f * static_cast<float>(BIOZ_cgmag) * static_cast<float>(BIOZ_gain) * 1e-9f;
  if (denom == 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  return (static_cast<float>(code) * static_cast<float>(V_ref)) / denom;
}

inline bool isEcgTagWithSample(uint8_t tag) {
  return (tag == ECG_TAG_VALID || tag == ECG_TAG_FAST || tag == ECG_TAG_VALID_EOF || tag == ECG_TAG_FAST_EOF);
}

inline bool isBiozTagWithSample(uint8_t tag) {
  return (tag == BIOZ_TAG_VALID || tag == BIOZ_TAG_RANGE || tag == BIOZ_TAG_VALID_EOF || tag == BIOZ_TAG_RANGE_EOF);
}

} // namespace

/******************************************************************************************************/
// FIFO Data Handling
/******************************************************************************************************/
void MAX30001G::FIFOReset(void) {
  /*
   * Resets the ECG and BIOZ FIFOs by writing to the FIFO_RST register.
   * Does not affect the ECG/BIOZ conversion engines.
   */
  writeRegister(MAX30001_FIFO_RST, 0x000000);

  // Read/latch STATUS after reset so software flags are not lost by clear-on-read behavior.
  readStatusAndLatchFlags();
  if (status.bit.eovf || status.bit.bovf) {
    LOGW("MAX30001G: FIFO reset issued, but overflow bits remain set (STATUS=0x%06lX).",
         (unsigned long)(status.all & 0x00FFFFFFUL));
  } else {
    LOGD("MAX30001G: FIFO reset successful.");
  }
}

/******************************************************************************************************/
// R to R reading
/******************************************************************************************************/
void MAX30001G::readRTOR(void) {
  /*
   * Read the RTOR register to calculate RR interval.
   * Call this when RRINT was asserted.
   */
  rtor_counter = 0;
  max30001_rtor_reg_t rtor;
  rtor.all = readRegister24(MAX30001_RTOR);

  const uint16_t intervalCounts = static_cast<uint16_t>(rtor.bit.data);
  rr_interval = static_cast<float>(intervalCounts) * RtoR_resolution; // [ms]

  if (intervalCounts == 0U || intervalCounts == 0x3FFFU || rr_interval <= 0.0f) {
    // 0x3FFF indicates overflow condition in RTOR timing counter.
    rr_interval = 0.0f;
  }

  if (RTOR_data.push(rr_interval) == 1U) {
    rtor_counter = 1;
  } else {
    LOGE("RTOR ring buffer full; sample dropped.");
  }

  // Consume software-latched RTOR event state once data was read.
  rtor_available = false;
}

/******************************************************************************************************/
// Single-sample ECG and BIOZ reading
/******************************************************************************************************/
float MAX30001G::readECG(bool reportRaw) {
  max30001_ecg_burst_reg_t ecg_data;
  ecg_data.all = readRegister24(MAX30001_ECG_FIFO);

  const uint8_t tag = static_cast<uint8_t>(ecg_data.bit.etag);
  EOF_detected = (tag == ECG_TAG_VALID_EOF || tag == ECG_TAG_FAST_EOF || tag == ECG_TAG_EMPTY);

  if (tag == ECG_TAG_OVERFLOW) {
    ecg_overflow_occurred = true;
    valid_data_detected = false;
    FIFOReset();
    return std::numeric_limits<float>::infinity();
  }

  if (!isEcgTagWithSample(tag)) {
    valid_data_detected = false;
    return std::numeric_limits<float>::infinity();
  }

  const int32_t sdata = signExtend(ecg_data.bit.data, 18);
  const float fdata = reportRaw ? static_cast<float>(sdata) : ecgCodeToMilliVolt(sdata);
  valid_data_detected = (tag == ECG_TAG_VALID || tag == ECG_TAG_VALID_EOF);

  LOGD("ECG Tag: %u, Sample: %.3f", tag, fdata);
  return fdata;
}

float MAX30001G::readBIOZ(bool reportRaw) {
  max30001_bioz_burst_reg_t bioz_data;
  bioz_data.all = readRegister24(MAX30001_BIOZ_FIFO);

  const uint8_t tag = static_cast<uint8_t>(bioz_data.bit.btag);
  EOF_detected = (tag == BIOZ_TAG_VALID_EOF || tag == BIOZ_TAG_RANGE_EOF || tag == BIOZ_TAG_EMPTY);

  if (tag == BIOZ_TAG_OVERFLOW) {
    bioz_overflow_occurred = true;
    valid_data_detected = false;
    FIFOReset();
    return std::numeric_limits<float>::infinity();
  }

  if (!isBiozTagWithSample(tag)) {
    valid_data_detected = false;
    return std::numeric_limits<float>::infinity();
  }

  const int32_t sdata = signExtend(bioz_data.bit.data, 20);
  const float fdata = reportRaw ? static_cast<float>(sdata) : biozCodeToOhm(sdata);
  valid_data_detected = (tag == BIOZ_TAG_VALID || tag == BIOZ_TAG_VALID_EOF);

  // Update range flags from STATUS when range-indicating tags are present.
  if (tag == BIOZ_TAG_RANGE || tag == BIOZ_TAG_RANGE_EOF) {
    readStatusAndLatchFlags();
  }

  LOGD("BIOZ Tag: %u, Sample: %.3f", tag, fdata);
  return fdata;
}

/******************************************************************************************************/
// FIFO burst reads
/******************************************************************************************************/

void MAX30001G::readECG_FIFO(bool reportRaw) {
  /*
   * Burst read ECG FIFO. Stops at EOF/EMPTY/OVERFLOW or after 32 words.
   */
  mngr_int.all = readRegister24(MAX30001_MNGR_INT);
  const uint8_t min_samples = static_cast<uint8_t>(mngr_int.bit.e_fit + 1U); // 1..32

  ecg_counter = 0;
  EOF_detected = false;
  valid_data_detected = false;

  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(_csPin, LOW);
  SPI.transfer((MAX30001_ECG_FIFO_BURST << 1) | MAX30001_READREG);

  for (uint8_t i = 0; i < 32; i++) {
    max30001_ecg_burst_reg_t ecg_data;
    ecg_data.all = (static_cast<uint32_t>(SPI.transfer(0x00)) << 16) | // top 8 bits
                   (static_cast<uint32_t>(SPI.transfer(0x00)) << 8)  | // middle 8 bits
                   (static_cast<uint32_t>(SPI.transfer(0x00)));        // low 8 bits

    const uint8_t tag = static_cast<uint8_t>(ecg_data.bit.etag);

    if (tag == ECG_TAG_OVERFLOW) {
      ecg_overflow_occurred = true;
      digitalWrite(_csPin, HIGH);
      SPI.endTransaction();
      FIFOReset();
      LOGW("ECG FIFO overflow detected during burst read.");
      return;
    }

    if (tag == ECG_TAG_EMPTY) {
      EOF_detected = true;
      break;
    }

    if (isEcgTagWithSample(tag)) {
      const int32_t sdata = signExtend(ecg_data.bit.data, 18);
      const float sample = reportRaw ? static_cast<float>(sdata) : ecgCodeToMilliVolt(sdata);
      if (ECG_data.push(sample) == 1U) {
        ecg_counter++;
      } else {
        LOGE("ECG ring buffer full; sample dropped.");
      }
      valid_data_detected = (tag == ECG_TAG_VALID || tag == ECG_TAG_VALID_EOF);
      if (tag == ECG_TAG_VALID_EOF || tag == ECG_TAG_FAST_EOF) {
        EOF_detected = true;
        if (i + 1U >= min_samples) {
          break;
        }
      }
    } else {
      LOGW("Unexpected ECG tag value: %u", tag);
    }

    // Once minimum threshold has been satisfied, stop at first EOF.
    if ((i + 1U) >= min_samples && EOF_detected) {
      break;
    }
  }

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  LOGD("Read %d samples from ECG FIFO.", ecg_counter);
}

void MAX30001G::readBIOZ_FIFO(bool reportRaw) {
  /*
   * Burst read BIOZ FIFO. Stops at EOF/EMPTY/OVERFLOW or after 32 words.
   */
  mngr_int.all = readRegister24(MAX30001_MNGR_INT);
  const uint8_t min_samples = static_cast<uint8_t>(mngr_int.bit.b_fit + 1U); // 1..8

  bioz_counter = 0;
  EOF_detected = false;
  valid_data_detected = false;
  bool range_tag_seen = false;

  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(_csPin, LOW);
  SPI.transfer((MAX30001_BIOZ_FIFO_BURST << 1) | MAX30001_READREG);

  for (uint8_t i = 0; i < 32; i++) {
    max30001_bioz_burst_reg_t bioz_data;
    bioz_data.all = (static_cast<uint32_t>(SPI.transfer(0x00)) << 16) | // top 8 bits
                    (static_cast<uint32_t>(SPI.transfer(0x00)) << 8)  | // middle 8 bits
                    (static_cast<uint32_t>(SPI.transfer(0x00)));        // low 8 bits

    const uint8_t tag = static_cast<uint8_t>(bioz_data.bit.btag);

    if (tag == BIOZ_TAG_OVERFLOW) {
      bioz_overflow_occurred = true;
      digitalWrite(_csPin, HIGH);
      SPI.endTransaction();
      FIFOReset();
      LOGW("BIOZ FIFO overflow detected during burst read.");
      return;
    }

    if (tag == BIOZ_TAG_EMPTY) {
      EOF_detected = true;
      break;
    }

    if (isBiozTagWithSample(tag)) {
      const int32_t sdata = signExtend(bioz_data.bit.data, 20);
      const float sample = reportRaw ? static_cast<float>(sdata) : biozCodeToOhm(sdata);
      if (BIOZ_data.push(sample) == 1U) {
        bioz_counter++;
      } else {
        LOGE("BIOZ ring buffer full; sample dropped.");
      }

      if (tag == BIOZ_TAG_RANGE || tag == BIOZ_TAG_RANGE_EOF) {
        // Indicates sample is outside programmed range (over or under).
        range_tag_seen = true;
        valid_data_detected = false;
      } else {
        valid_data_detected = true;
      }

      if (tag == BIOZ_TAG_VALID_EOF || tag == BIOZ_TAG_RANGE_EOF) {
        EOF_detected = true;
        if (i + 1U >= min_samples) {
          break;
        }
      }
    } else {
      LOGW("Unexpected BIOZ tag value: %u", tag);
    }

    // Once minimum threshold has been satisfied, stop at first EOF.
    if ((i + 1U) >= min_samples && EOF_detected) {
      break;
    }
  }

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  if (range_tag_seen) {
    readStatusAndLatchFlags();
  }

  LOGD("Read %d samples from BIOZ FIFO.", bioz_counter);
}
