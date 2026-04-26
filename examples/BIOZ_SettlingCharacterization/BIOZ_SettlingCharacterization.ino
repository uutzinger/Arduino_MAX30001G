#include <Arduino.h>
#include <math.h>
#include "max30001g.h"
#include "logger.h"

const uint8_t AFE_CS_PIN = 6;
const int AFE_INT1_PIN = 12;
const int AFE_INT2_PIN = -1;

// Enable or disable settling experiments here.
const bool RUN_LOW_RATE_SUITE = true;
const bool RUN_FAST_RATE_SUITE = true;
const bool RUN_PHASE_TESTS = true;
const bool RUN_FREQUENCY_TESTS = true;
const bool RUN_CURRENT_TESTS = true;
const bool RUN_SAMPLING_RATE_TRANSITION_TESTS = true;
const bool RUN_DLPF_TESTS = true;
const bool RUN_AHPF_TESTS = true;
const bool RUN_BIOZ_START_TEST = true;

const uint32_t PLL_STARTUP_SETTLE_MS = 500U;
const uint32_t PLL_STATUS_CLEAR_MS = 10U;
const uint32_t SAMPLE_TIMEOUT_MS = 2500U;
const uint8_t SCAN_AVG_SAMPLES = 8U;
const uint8_t FIFO_THRESHOLD_SAMPLES = SCAN_AVG_SAMPLES;
const uint8_t PRECONDITION_SAMPLES = 24U;
const uint8_t SETTLING_SAMPLES = 80U;
const uint8_t GROUP_SUMMARY_SAMPLES = 8U;
const float SETTLING_THRESHOLD_PERCENT = 0.1f;
const uint8_t SETTLING_WINDOW_SAMPLES = GROUP_SUMMARY_SAMPLES;
const uint8_t MAX_SETTLING_SUMMARIES = 40U;
const bool PRINT_SAMPLE_ROWS = true;
const bool PRINT_GROUP_SUMMARY_ROWS = true;

const uint8_t BIOZ_GAIN_20_VV = 1U;
const uint8_t BIOZ_AHPF_60HZ = 0U;
const uint8_t BIOZ_AHPF_150HZ = 1U;
const uint8_t BIOZ_AHPF_500HZ = 2U;
const uint8_t BIOZ_DHPF_BYPASS = 0U;
const uint32_t BIOZ_INTERNAL_RESISTOR_OHM = 1000UL;
const uint8_t BIOZ_RESISTOR_MODULATION = 0U;
const uint8_t BIOZ_RESISTOR_MOD_FREQ = 0U;

const uint8_t BIOZ_SPEED_LOW_RATE = 0U;   // ~32 sps
const uint8_t BIOZ_SPEED_FAST_RATE = 1U;  // ~64 sps

const uint8_t BIOZ_DLPF_BYPASS = 0U;
const uint8_t BIOZ_DLPF_4HZ = 1U;
const uint8_t BIOZ_DLPF_8HZ = 2U;
const uint8_t BIOZ_DLPF_16HZ = 3U;

struct BiozPoint {
  uint16_t frequency_hz;
  float phase_deg;
  uint32_t current_nA;
  uint8_t speed;
  uint8_t ahpf;
  uint8_t dlpf;
};

struct SettlingResult {
  bool settled;
  uint8_t sample_index;
  uint32_t elapsed_ms;
  uint8_t fifo_blocks_of_8;
};

struct SettlingSummary {
  const char* suite_speed;
  const char* test_type;
  const char* from_value;
  const char* to_value;
  float frequency_hz;
  float phase_deg;
  uint32_t current_nA;
  uint8_t speed;
  uint8_t ahpf;
  uint8_t dlpf;
  float final_reference_ohm;
  SettlingResult result;
};

MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);
SettlingSummary settlingSummaries[MAX_SETTLING_SUMMARIES];
uint8_t settlingSummaryCount = 0U;

BiozPoint basePoint(uint8_t speed) {
  BiozPoint point;
  point.frequency_hz = 8192U;
  point.phase_deg = 0.0f;
  point.current_nA = 8000UL;
  point.speed = speed;
  point.ahpf = BIOZ_AHPF_150HZ;
  point.dlpf = BIOZ_DLPF_4HZ;
  return point;
}

void clearBiozBuffer() {
  float value = 0.0f;
  while (BIOZ_data.available() > 0) {
    BIOZ_data.pop(value);
  }
}

void clearBiozAcquisitionState() {
  clearBiozBuffer();
  bioz_available = false;
  afe_irq_pending = false;
  afe_irq1_pending = false;
  afe_irq2_pending = false;
  valid_data_detected = false;
  over_voltage_detected = false;
  under_voltage_detected = false;
  EOF_detected = false;
}

float rawBiozToOhm(float raw_code) {
  const float vref_volts = static_cast<float>(V_ref) * 1e-3f;
  const float current_amps = static_cast<float>(BIOZ_cgmag) * 1e-9f;
  const float denom = 524288.0f * current_amps * static_cast<float>(BIOZ_gain);
  return (denom > 0.0f) ? (raw_code * vref_volts) / denom : 0.0f;
}

uint32_t scanFifoPollIntervalMs() {
  const float sps = (BIOZ_samplingRate > 1.0f) ? BIOZ_samplingRate : 1.0f;
  uint32_t interval = static_cast<uint32_t>(
    (1000.0f * static_cast<float>(SCAN_AVG_SAMPLES)) / sps
  );
  if (interval < 5U) {
    interval = 5U;
  }
  return interval;
}

uint32_t statusWord() {
  afe.readStatusRegisters();
  return status.all & 0x00FFFFFFUL;
}

bool pllOk(uint32_t status_word) {
  return (status_word & MAX30001_STATUS_PLLINT) == 0U;
}

void printPllStatus(const char* context) {
  const uint32_t word = statusWord();
  Serial.print(context);
  Serial.print(" PLL: ");
  Serial.print(pllOk(word) ? "OK" : "UNLOCKED");
  Serial.print(" (STATUS=0x");
  Serial.print(word, HEX);
  Serial.println(")");
}

uint8_t rnomValueForInternalResistor(uint32_t resistance_ohm) {
  if      (resistance_ohm > 2500UL) { return 0U; }
  else if (resistance_ohm > 1667UL) { return 1U; }
  else if (resistance_ohm > 1250UL) { return 2U; }
  else if (resistance_ohm > 1000UL) { return 3U; }
  else if (resistance_ohm >  833UL) { return 4U; }
  else if (resistance_ohm >  714UL) { return 5U; }
  else if (resistance_ohm >  625UL) { return 6U; }
  else                              { return 7U; }
}

void configureScanLikeBioz(const BiozPoint& point, bool enable_bioz) {
  afe.stop();
  afe.swReset();
  afe.setFMSTR(0U);

  afe.setBIOZSamplingRate(point.speed);
  afe.setBIOZgain(BIOZ_GAIN_20_VV, true);
  afe.setBIOZModulationFrequencyByFrequency(point.frequency_hz);
  afe.setBIOZmag(point.current_nA);
  afe.setBIOZmodulation((BIOZ_cgmag < 8000UL) ? 0U : 1U);
  afe.setBIOZfilter(point.ahpf, point.dlpf, BIOZ_DHPF_BYPASS);
  afe.setBIOZPhaseOffsetbyPhase(point.frequency_hz, point.phase_deg);

  afe.setDefaultNoTestSignal();
  afe.setBIOZTestImpedance(
    true,
    false,
    false,
    rnomValueForInternalResistor(BIOZ_INTERNAL_RESISTOR_OHM),
    0U,
    0U
  );
  afe.setDefaultNoRtoR();
  afe.setDefaultInterruptClearing();

  afe.setFIFOInterruptThreshold(32U, FIFO_THRESHOLD_SAMPLES);
  afe.setInterrupt1(false, true, false, false, false);
  afe.setInterrupt2(false, false, false, false, false);

  if (enable_bioz) {
    afe.enableAFE(false, true, false);
    afe.setLeadsBias(false, 0U);
    afe.setLeadsOffDetection(false, false, 0U);
    afe.setLeadsOnDetection(false);
    afe.FIFOReset();
    afe.synch();
  }
  clearBiozAcquisitionState();
}

void configureActiveBioz(const BiozPoint& point) {
  configureScanLikeBioz(point, true);
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset();
  afe.synch();
  clearBiozAcquisitionState();
}

void applyTransition(const BiozPoint& from, const BiozPoint& to, bool bioz_start_transition) {
  if (bioz_start_transition) {
    afe.enableAFE(false, true, false);
    afe.setLeadsBias(false, 0U);
    afe.setLeadsOffDetection(false, false, 0U);
    afe.setLeadsOnDetection(false);
    afe.FIFOReset();
    afe.synch();
    clearBiozAcquisitionState();
    return;
  }

  if (to.speed != from.speed) {
    afe.setBIOZSamplingRate(to.speed);
  }

  if (to.frequency_hz != from.frequency_hz) {
    afe.setBIOZModulationFrequencyByFrequency(to.frequency_hz);
  }

  if ((to.current_nA != from.current_nA) || (to.frequency_hz != from.frequency_hz)) {
    afe.setBIOZmag(to.current_nA);
    afe.setBIOZmodulation((BIOZ_cgmag < 8000UL) ? 0U : 1U);
  }

  if ((to.ahpf != from.ahpf) || (to.dlpf != from.dlpf)) {
    afe.setBIOZfilter(to.ahpf, to.dlpf, BIOZ_DHPF_BYPASS);
  }

  if ((to.phase_deg != from.phase_deg) || (to.frequency_hz != from.frequency_hz)) {
    afe.setBIOZPhaseOffsetbyPhase(to.frequency_hz, to.phase_deg);
  }

  afe.FIFOReset();
  afe.synch();
  clearBiozAcquisitionState();
}

bool collectSample(float& value_ohm) {
  if (BIOZ_data.available() > 0) {
    float raw_code = 0.0f;
    BIOZ_data.pop(raw_code);
    value_ohm = rawBiozToOhm(raw_code);
    return true;
  }

  const uint32_t start_ms = millis();
  const uint32_t fifo_poll_interval_ms = scanFifoPollIntervalMs();
  uint32_t last_direct_read_ms = millis();

  while ((millis() - start_ms) < SAMPLE_TIMEOUT_MS) {
    const uint32_t now = millis();
    if ((now - last_direct_read_ms) >= fifo_poll_interval_ms) {
      afe.readBIOZ_FIFO(true);
      last_direct_read_ms = now;
    }

    if (BIOZ_data.available() > 0) {
      float raw_code = 0.0f;
      BIOZ_data.pop(raw_code);
      value_ohm = rawBiozToOhm(raw_code);
      return true;
    }

    delay(2);
  }

  value_ohm = 0.0f;
  return false;
}

void discardSamples(uint8_t count) {
  for (uint8_t i = 0U; i < count; ++i) {
    float value_ohm = 0.0f;
    collectSample(value_ohm);
  }
}

const char* speedLabel(uint8_t speed) {
  return speed == BIOZ_SPEED_FAST_RATE ? "fast" : "low";
}

const char* dlpfLabel(uint8_t dlpf) {
  switch (dlpf) {
    case BIOZ_DLPF_BYPASS: return "bypass";
    case BIOZ_DLPF_4HZ: return "4Hz";
    case BIOZ_DLPF_8HZ: return "8Hz";
    case BIOZ_DLPF_16HZ: return "16Hz";
    default: return "unknown";
  }
}

const char* ahpfLabel(uint8_t ahpf) {
  switch (ahpf) {
    case BIOZ_AHPF_60HZ: return "60Hz";
    case BIOZ_AHPF_150HZ: return "150Hz";
    case BIOZ_AHPF_500HZ: return "500Hz";
    case 3U: return "1kHz";
    case 4U: return "2kHz";
    case 5U: return "4kHz";
    case 6U:
    case 7U: return "bypass";
    default: return "unknown";
  }
}

void printExperimentHeader() {
  Serial.println("# BIOZ settling characterization");
  Serial.println("# Experiment: use the MAX30001G internal 1kOhm BIOZ BIST load, change one BIOZ setting at a time, reset/synchronize the FIFO, then record the post-transition settling response.");
  Serial.println("# Setup and collection now mirror BIOZScan: scan-like low-level BIOZ setup, FIFO threshold avg=8, raw FIFO reads, scan-style raw-to-ohm conversion, FIFOReset(), synch(), then sample-count settling.");
  Serial.println("# The same test suite can run at low and fast BIOZ sampling rates. Each case below has its own settings line and data header.");
  Serial.println("# Individual sample timestamps are grouped by FIFO burst; use 8-sample group summaries for scan-equivalent timing.");
  Serial.print("# Settling result: first sample where ");
  Serial.print(SETTLING_WINDOW_SAMPLES);
  Serial.print(" consecutive valid samples stay within ");
  Serial.print(SETTLING_THRESHOLD_PERCENT, 3);
  Serial.println("% of the final reference.");
}

void printCaseSettings(
  const char* suite_speed,
  const char* test_type,
  const char* from_value,
  const char* to_value,
  const BiozPoint& target,
  float final_reference_ohm
) {
  Serial.println();
  Serial.print("# suite: ");
  Serial.println(suite_speed);
  Serial.print("# case: ");
  Serial.print(test_type);
  Serial.print(" ");
  Serial.print(from_value);
  Serial.print(" -> ");
  Serial.println(to_value);

  Serial.print("# settings: load_ohm=");
  Serial.print(BIOZ_INTERNAL_RESISTOR_OHM);
  Serial.print(", frequency_hz=");
  Serial.print(BIOZ_frequency, 1);
  Serial.print(", phase_deg=");
  Serial.print(target.phase_deg, 2);
  Serial.print(", current_nA=");
  Serial.print(BIOZ_cgmag);
  Serial.print(", speed=");
  Serial.print(speedLabel(target.speed));
  Serial.print("(");
  Serial.print(target.speed);
  Serial.print("), ahpf=");
  Serial.print(ahpfLabel(target.ahpf));
  Serial.print("(");
  Serial.print(target.ahpf);
  Serial.print("), dlpf=");
  Serial.print(dlpfLabel(target.dlpf));
  Serial.print("(");
  Serial.print(target.dlpf);
  Serial.print("), fifo_threshold_samples=");
  Serial.print(FIFO_THRESHOLD_SAMPLES);
  Serial.print(", raw_fifo_capture=1");
  Serial.print(", final_reference_ohm=");
  Serial.println(final_reference_ohm, 3);
}

float deltaFromFinalPercent(float value_ohm, float final_reference_ohm) {
  const float denominator = fabs(final_reference_ohm) > 0.001f ? fabs(final_reference_ohm) : 1.0f;
  return 100.0f * fabs(value_ohm - final_reference_ohm) / denominator;
}

uint8_t fifoBlocksOf8(uint8_t sample_index) {
  return static_cast<uint8_t>((sample_index + GROUP_SUMMARY_SAMPLES - 1U) / GROUP_SUMMARY_SAMPLES);
}

SettlingResult findSettlingResult(
  const float value_ohm[],
  const uint32_t elapsed_ms[],
  const bool valid_sample[],
  float final_reference_ohm
) {
  SettlingResult result;
  result.settled = false;
  result.sample_index = 0U;
  result.elapsed_ms = 0U;
  result.fifo_blocks_of_8 = 0U;

  for (uint8_t start = 0U; (start + SETTLING_WINDOW_SAMPLES) <= SETTLING_SAMPLES; ++start) {
    bool settled_window = true;
    for (uint8_t offset = 0U; offset < SETTLING_WINDOW_SAMPLES; ++offset) {
      const uint8_t index = start + offset;
      if (!valid_sample[index] ||
          deltaFromFinalPercent(value_ohm[index], final_reference_ohm) > SETTLING_THRESHOLD_PERCENT) {
        settled_window = false;
        break;
      }
    }

    if (settled_window) {
      result.settled = true;
      result.sample_index = start + 1U;
      result.elapsed_ms = elapsed_ms[start];
      result.fifo_blocks_of_8 = fifoBlocksOf8(result.sample_index);
      break;
    }
  }

  return result;
}

void printCaseStatus(uint32_t word) {
  Serial.print("# status_after_capture: status_hex=0x");
  Serial.print(word, HEX);
  Serial.print(", pll_state=");
  Serial.println(pllOk(word) ? "OK" : "UNLOCKED");
}

void printSampleRow(
  uint8_t index,
  uint32_t elapsed_ms,
  float value_ohm,
  float final_reference_ohm,
  bool valid_sample
) {
  Serial.print(index);
  Serial.print(",");
  Serial.print(elapsed_ms);
  Serial.print(",");
  Serial.print(value_ohm, 3);
  Serial.print(",");
  Serial.print(deltaFromFinalPercent(value_ohm, final_reference_ohm), 3);
  Serial.print(",");
  Serial.println(valid_sample ? 1U : 0U);
}

void printGroupRow(
  uint8_t group,
  uint8_t last_sample_index,
  uint32_t elapsed_ms,
  float mean_ohm,
  float min_ohm,
  float max_ohm,
  float final_reference_ohm,
  uint8_t valid_sample_count
) {
  Serial.print(group);
  Serial.print(",");
  Serial.print(last_sample_index);
  Serial.print(",");
  Serial.print(elapsed_ms);
  Serial.print(",");
  Serial.print(mean_ohm, 3);
  Serial.print(",");
  Serial.print(min_ohm, 3);
  Serial.print(",");
  Serial.print(max_ohm, 3);
  Serial.print(",");
  Serial.print(deltaFromFinalPercent(mean_ohm, final_reference_ohm), 3);
  Serial.print(",");
  Serial.println(valid_sample_count);
}

void printSettlingResult(const SettlingResult& result) {
  Serial.println("# table: settling summary for this case");
  Serial.println("threshold_percent,window_samples,settled,sample_index,elapsed_ms,fifo_blocks_of_8");
  Serial.print(SETTLING_THRESHOLD_PERCENT, 3);
  Serial.print(",");
  Serial.print(SETTLING_WINDOW_SAMPLES);
  Serial.print(",");
  Serial.print(result.settled ? 1U : 0U);
  Serial.print(",");
  Serial.print(result.sample_index);
  Serial.print(",");
  Serial.print(result.elapsed_ms);
  Serial.print(",");
  Serial.println(result.fifo_blocks_of_8);
}

void rememberSettlingSummary(
  const char* suite_speed,
  const char* test_type,
  const char* from_value,
  const char* to_value,
  const BiozPoint& target,
  float final_reference_ohm,
  const SettlingResult& result
) {
  if (settlingSummaryCount >= MAX_SETTLING_SUMMARIES) {
    return;
  }

  SettlingSummary& summary = settlingSummaries[settlingSummaryCount++];
  summary.suite_speed = suite_speed;
  summary.test_type = test_type;
  summary.from_value = from_value;
  summary.to_value = to_value;
  summary.frequency_hz = BIOZ_frequency;
  summary.phase_deg = target.phase_deg;
  summary.current_nA = BIOZ_cgmag;
  summary.speed = target.speed;
  summary.ahpf = target.ahpf;
  summary.dlpf = target.dlpf;
  summary.final_reference_ohm = final_reference_ohm;
  summary.result = result;
}

void printSettlingSummaryTable() {
  Serial.println();
  Serial.println("# final settling summary");
  Serial.println("suite_speed,test_type,from_setting,to_setting,frequency_hz,phase_deg,current_nA,speed,ahpf,dlpf,final_reference_ohm,threshold_percent,window_samples,settled,sample_index,elapsed_ms,fifo_blocks_of_8");
  for (uint8_t i = 0U; i < settlingSummaryCount; ++i) {
    const SettlingSummary& summary = settlingSummaries[i];
    Serial.print(summary.suite_speed);
    Serial.print(",");
    Serial.print(summary.test_type);
    Serial.print(",");
    Serial.print(summary.from_value);
    Serial.print(",");
    Serial.print(summary.to_value);
    Serial.print(",");
    Serial.print(summary.frequency_hz, 1);
    Serial.print(",");
    Serial.print(summary.phase_deg, 2);
    Serial.print(",");
    Serial.print(summary.current_nA);
    Serial.print(",");
    Serial.print(speedLabel(summary.speed));
    Serial.print(",");
    Serial.print(ahpfLabel(summary.ahpf));
    Serial.print(",");
    Serial.print(dlpfLabel(summary.dlpf));
    Serial.print(",");
    Serial.print(summary.final_reference_ohm, 3);
    Serial.print(",");
    Serial.print(SETTLING_THRESHOLD_PERCENT, 3);
    Serial.print(",");
    Serial.print(SETTLING_WINDOW_SAMPLES);
    Serial.print(",");
    Serial.print(summary.result.settled ? 1U : 0U);
    Serial.print(",");
    Serial.print(summary.result.sample_index);
    Serial.print(",");
    Serial.print(summary.result.elapsed_ms);
    Serial.print(",");
    Serial.println(summary.result.fifo_blocks_of_8);
  }
}

void runSettlingCase(
  const char* suite_speed,
  const char* test_type,
  const char* from_value,
  const char* to_value,
  const BiozPoint& from,
  const BiozPoint& to,
  bool bioz_start_transition = false
) {
  if (bioz_start_transition) {
    configureScanLikeBioz(to, false);
    delay(100U);
  } else {
    configureActiveBioz(from);
    discardSamples(PRECONDITION_SAMPLES);
  }

  const uint32_t transition_ms = millis();
  applyTransition(from, to, bioz_start_transition);

  float value_ohm[SETTLING_SAMPLES];
  uint32_t elapsed_ms[SETTLING_SAMPLES];
  bool valid_sample[SETTLING_SAMPLES];

  for (uint8_t i = 0U; i < SETTLING_SAMPLES; ++i) {
    valid_sample[i] = collectSample(value_ohm[i]);
    elapsed_ms[i] = millis() - transition_ms;
  }

  const uint32_t capture_status_word = statusWord();

  float final_reference = 0.0f;
  uint8_t final_count = 0U;
  for (uint8_t i = SETTLING_SAMPLES; i > 0U && final_count < GROUP_SUMMARY_SAMPLES; --i) {
    const uint8_t index = i - 1U;
    if (valid_sample[index]) {
      final_reference += value_ohm[index];
      final_count++;
    }
  }
  final_reference = final_count > 0U ? final_reference / static_cast<float>(final_count) : 0.0f;

  const SettlingResult settling_result = findSettlingResult(
    value_ohm,
    elapsed_ms,
    valid_sample,
    final_reference
  );
  rememberSettlingSummary(suite_speed, test_type, from_value, to_value, to, final_reference, settling_result);

  printCaseSettings(suite_speed, test_type, from_value, to_value, to, final_reference);
  printCaseStatus(capture_status_word);

  if (PRINT_SAMPLE_ROWS) {
    Serial.println("# table: individual FIFO samples");
    Serial.println("sample_index,elapsed_ms_since_transition,value_ohm,delta_from_final_percent,valid_sample");
    for (uint8_t i = 0U; i < SETTLING_SAMPLES; ++i) {
      printSampleRow(
        i + 1U,
        elapsed_ms[i],
        value_ohm[i],
        final_reference,
        valid_sample[i]
      );
    }
    printSettlingResult(settling_result);
  }

  if (PRINT_GROUP_SUMMARY_ROWS) {
    Serial.println("# table: 8-sample group summaries");
    Serial.println("group_index,last_sample_index,elapsed_ms_since_transition,mean_ohm,min_ohm,max_ohm,delta_from_final_percent,valid_sample_count");
    for (uint8_t start = 0U, group = 1U; start < SETTLING_SAMPLES; start += GROUP_SUMMARY_SAMPLES, ++group) {
      float sum = 0.0f;
      float min_value = 0.0f;
      float max_value = 0.0f;
      uint8_t count = 0U;
      uint32_t group_elapsed_ms = 0U;
      uint8_t last_sample_index = start + 1U;

      for (uint8_t j = 0U; j < GROUP_SUMMARY_SAMPLES && (start + j) < SETTLING_SAMPLES; ++j) {
        const uint8_t index = start + j;
        if (!valid_sample[index]) {
          continue;
        }

        if (count == 0U) {
          min_value = value_ohm[index];
          max_value = value_ohm[index];
        } else {
          if (value_ohm[index] < min_value) { min_value = value_ohm[index]; }
          if (value_ohm[index] > max_value) { max_value = value_ohm[index]; }
        }

        sum += value_ohm[index];
        group_elapsed_ms = elapsed_ms[index];
        last_sample_index = index + 1U;
        count++;
      }

      const float mean_value = count > 0U ? sum / static_cast<float>(count) : 0.0f;
      printGroupRow(
        group,
        last_sample_index,
        group_elapsed_ms,
        mean_value,
        min_value,
        max_value,
        final_reference,
        count
      );
    }
  }
}

void runPhaseTests(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = from;

  from.phase_deg = 0.0f;
  to.phase_deg = 45.0f;
  runSettlingCase(suite_speed, "phase", "0deg", "45deg", from, to);

  from.phase_deg = 45.0f;
  to.phase_deg = 90.0f;
  runSettlingCase(suite_speed, "phase", "45deg", "90deg", from, to);

  from.phase_deg = 168.75f;
  to.phase_deg = 0.0f;
  runSettlingCase(suite_speed, "phase", "168.75deg", "0deg", from, to);
}

void runFrequencyTests(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = from;

  from.frequency_hz = 4096U;
  to.frequency_hz = 8192U;
  runSettlingCase(suite_speed, "frequency", "4096Hz", "8192Hz", from, to);

  from.frequency_hz = 8192U;
  to.frequency_hz = 4096U;
  runSettlingCase(suite_speed, "frequency", "8192Hz", "4096Hz", from, to);

  from.frequency_hz = 2048U;
  to.frequency_hz = 8192U;
  runSettlingCase(suite_speed, "frequency", "2048Hz", "8192Hz", from, to);
}

void runCurrentTests(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = from;

  from.current_nA = 8000UL;
  to.current_nA = 16000UL;
  runSettlingCase(suite_speed, "current", "8000nA", "16000nA", from, to);

  from.current_nA = 16000UL;
  to.current_nA = 8000UL;
  runSettlingCase(suite_speed, "current", "16000nA", "8000nA", from, to);

  from.current_nA = 8000UL;
  to.current_nA = 32000UL;
  runSettlingCase(suite_speed, "current", "8000nA", "32000nA", from, to);

  from.current_nA = 32000UL;
  to.current_nA = 8000UL;
  runSettlingCase(suite_speed, "current", "32000nA", "8000nA", from, to);
}

void runSamplingRateTests() {
  BiozPoint from = basePoint(BIOZ_SPEED_LOW_RATE);
  BiozPoint to = from;

  from.speed = BIOZ_SPEED_LOW_RATE;
  to.speed = BIOZ_SPEED_FAST_RATE;
  runSettlingCase("transition", "sampling_rate", "low", "fast", from, to);

  from.speed = BIOZ_SPEED_FAST_RATE;
  to.speed = BIOZ_SPEED_LOW_RATE;
  runSettlingCase("transition", "sampling_rate", "fast", "low", from, to);
}

void runDlpfTests(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = from;

  from.dlpf = BIOZ_DLPF_4HZ;
  to.dlpf = BIOZ_DLPF_BYPASS;
  runSettlingCase(suite_speed, "dlpf", "4Hz", "bypass", from, to);

  from.dlpf = BIOZ_DLPF_BYPASS;
  to.dlpf = BIOZ_DLPF_4HZ;
  runSettlingCase(suite_speed, "dlpf", "bypass", "4Hz", from, to);

  from.dlpf = BIOZ_DLPF_4HZ;
  to.dlpf = BIOZ_DLPF_8HZ;
  runSettlingCase(suite_speed, "dlpf", "4Hz", "8Hz", from, to);

  from.dlpf = BIOZ_DLPF_4HZ;
  to.dlpf = BIOZ_DLPF_16HZ;
  runSettlingCase(suite_speed, "dlpf", "4Hz", "16Hz", from, to);
}

void runAhpfTests(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = from;

  from.frequency_hz = 256U;
  to.frequency_hz = 256U;
  from.ahpf = BIOZ_AHPF_150HZ;
  to.ahpf = BIOZ_AHPF_60HZ;
  runSettlingCase(suite_speed, "ahpf", "150Hz", "60Hz", from, to);

  from.ahpf = BIOZ_AHPF_60HZ;
  to.ahpf = BIOZ_AHPF_150HZ;
  runSettlingCase(suite_speed, "ahpf", "60Hz", "150Hz", from, to);

  from.frequency_hz = 1024U;
  to.frequency_hz = 1024U;
  from.ahpf = BIOZ_AHPF_500HZ;
  to.ahpf = BIOZ_AHPF_150HZ;
  runSettlingCase(suite_speed, "ahpf", "500Hz", "150Hz", from, to);

  from.ahpf = BIOZ_AHPF_150HZ;
  to.ahpf = BIOZ_AHPF_500HZ;
  runSettlingCase(suite_speed, "ahpf", "150Hz", "500Hz", from, to);
}

void runBiozStartTest(uint8_t base_speed) {
  const char* suite_speed = speedLabel(base_speed);
  BiozPoint from = basePoint(base_speed);
  BiozPoint to = basePoint(base_speed);
  runSettlingCase(suite_speed, "bioz_start", "stopped", "enabled", from, to, true);
}

void runTestSuite(uint8_t base_speed) {
  Serial.println();
  Serial.print("# suite_start: ");
  Serial.println(speedLabel(base_speed));

  if (RUN_PHASE_TESTS) { runPhaseTests(base_speed); }
  if (RUN_FREQUENCY_TESTS) { runFrequencyTests(base_speed); }
  if (RUN_CURRENT_TESTS) { runCurrentTests(base_speed); }
  if (RUN_DLPF_TESTS) { runDlpfTests(base_speed); }
  if (RUN_AHPF_TESTS) { runAhpfTests(base_speed); }
  if (RUN_BIOZ_START_TEST) { runBiozStartTest(base_speed); }

  Serial.print("# suite_end: ");
  Serial.println(speedLabel(base_speed));
}

void runAllTests() {
  printExperimentHeader();
  Serial.println("# MAX30001G BIOZ settling characterization started.");
  printPllStatus("# Startup idle");

  if (RUN_LOW_RATE_SUITE) { runTestSuite(BIOZ_SPEED_LOW_RATE); }
  if (RUN_FAST_RATE_SUITE) { runTestSuite(BIOZ_SPEED_FAST_RATE); }
  if (RUN_SAMPLING_RATE_TRANSITION_TESTS) { runSamplingRateTests(); }

  printSettlingSummaryTable();
  afe.stop();
  Serial.println("# BIOZ settling characterization complete.");
}

void setup() {
  currentLogLevel = LOG_LEVEL_WARN;

  Serial.begin(115200);
  delay(1000);

  afe.begin();
  runAllTests();
}

void loop() {
  delay(1000);
}
