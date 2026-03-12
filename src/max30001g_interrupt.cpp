/******************************************************************************************************/
// Interrupt Setting and Handling
/******************************************************************************************************/
#include <Arduino.h>
#include "logger.h"
#include "max30001g.h"

namespace {

constexpr uint32_t kDefaultStatusMask = 0x00FFFFFFUL;
constexpr uint8_t kMaxCallbackDevices = 4;
constexpr uint32_t kEventMasks[MAX30001G::IRQ_EVENT_COUNT] = {
  MAX30001_STATUS_EINT,      // IRQ_ECG_FIFO
  MAX30001_STATUS_EOVF,      // IRQ_ECG_OVF
  MAX30001_STATUS_FSTINT,    // IRQ_ECG_FAST
  MAX30001_STATUS_DCLOFFINT, // IRQ_ECG_LEADOFF
  MAX30001_STATUS_BINT,      // IRQ_BIOZ_FIFO
  MAX30001_STATUS_BOVF,      // IRQ_BIOZ_OVF
  MAX30001_STATUS_BOVER,     // IRQ_BIOZ_OVER
  MAX30001_STATUS_BUNDR,     // IRQ_BIOZ_UNDER
  MAX30001_STATUS_BCGMON,    // IRQ_BIOZ_CGMON
  MAX30001_STATUS_LONINT,    // IRQ_LEADS_ON
  MAX30001_STATUS_RRINT,     // IRQ_RTOR
  MAX30001_STATUS_PLLINT     // IRQ_PLL_UNLOCK
};

struct CallbackRegistry {
  MAX30001G* device = nullptr;
  MAX30001G::InterruptCallback eventCallbacks[MAX30001G::IRQ_EVENT_COUNT] = {nullptr};
  void* eventContexts[MAX30001G::IRQ_EVENT_COUNT] = {nullptr};
  MAX30001G::InterruptCallback aggregateCallback = nullptr;
  void* aggregateContext = nullptr;
  uint32_t aggregateMask = kDefaultStatusMask;
};

CallbackRegistry g_registries[kMaxCallbackDevices];

inline bool isValidEvent(MAX30001G::InterruptEvent event) {
  return static_cast<uint8_t>(event) < static_cast<uint8_t>(MAX30001G::IRQ_EVENT_COUNT);
}

bool isRegistryEmpty(const CallbackRegistry& registry) {
  if (registry.aggregateCallback != nullptr) {
    return false;
  }
  for (uint8_t i = 0; i < static_cast<uint8_t>(MAX30001G::IRQ_EVENT_COUNT); i++) {
    if (registry.eventCallbacks[i] != nullptr) {
      return false;
    }
  }
  return true;
}

CallbackRegistry* getRegistry(MAX30001G* device, bool createIfMissing) {
  for (uint8_t i = 0; i < kMaxCallbackDevices; i++) {
    if (g_registries[i].device == device) {
      return &g_registries[i];
    }
  }
  if (!createIfMissing) {
    return nullptr;
  }
  for (uint8_t i = 0; i < kMaxCallbackDevices; i++) {
    if (g_registries[i].device == nullptr) {
      g_registries[i].device = device;
      g_registries[i].aggregateMask = kDefaultStatusMask;
      return &g_registries[i];
    }
  }
  return nullptr;
}

inline void dispatchEventCallback(CallbackRegistry* registry, MAX30001G* device, MAX30001G::InterruptEvent event, uint32_t statusWord) {
  if (registry == nullptr) {
    return;
  }
  const uint8_t idx = static_cast<uint8_t>(event);
  if ((statusWord & kEventMasks[idx]) != 0U) {
    MAX30001G::InterruptCallback cb = registry->eventCallbacks[idx];
    if (cb != nullptr) {
      cb(device, statusWord, registry->eventContexts[idx]);
    }
  }
}

} // namespace

/******************************************************************************************************/

void MAX30001G::clearLatchedStatusFlags(void) {
  ecg_available = false;
  bioz_available = false;
  rtor_available = false;

  ecg_lead_off = false;
  ecg_fast_recovery_occurred = false;
  ecg_overflow_occurred = false;
  bioz_cgm_occurred = false;
  bioz_undervoltage_occurred = false;
  bioz_overvoltage_occurred = false;
  bioz_overflow_occurred = false;
  leads_on_detected = false;
  pll_unlocked_occurred = false;
}

/* statusWord is the content of the captured STATUS register */
void MAX30001G::latchStatusFlags(uint32_t statusWord) {
  status.all = statusWord;

  if (status.bit.eint) {
    ecg_available = true;
  }
  if (status.bit.eovf) {
    ecg_overflow_occurred = true;
  }
  if (status.bit.fstint) {
    ecg_fast_recovery_occurred = true;
  }
  if (status.bit.dcloffint) {
    ecg_lead_off = true;
  }
  if (status.bit.bint) {
    bioz_available = true;
  }
  if (status.bit.bovf) {
    bioz_overflow_occurred = true;
  }
  if (status.bit.bover) {
    bioz_overvoltage_occurred = true;
  }
  if (status.bit.bundr) {
    bioz_undervoltage_occurred = true;
  }
  if (status.bit.bcgmon) {
    bioz_cgm_occurred = true;
  }
  if (status.bit.lonint) {
    leads_on_detected = true;
  }
  if (status.bit.rrint) {
    rtor_available = true;
  }
  if (status.bit.pllint) {
    pll_unlocked_occurred = true;
  }
}

uint32_t MAX30001G::readStatusAndLatchFlags(void) {
  const uint32_t statusWord = readRegister24(MAX30001_STATUS);
  latchStatusFlags(statusWord);
  return statusWord;
}

bool MAX30001G::servicePendingInterrupts() {
/*
 * ISR-safe service helper.
 * Reads and clears software pending flags set by onAFE_IRQ1/onAFE_IRQ2.
 * Uses Arduino core critical-section calls (noInterrupts/interrupts).
 * Returns true when STATUS was serviced.
 */
  bool pending = false;
  noInterrupts();
  if (afe_irq_pending || afe_irq1_pending || afe_irq2_pending) {
    pending = true;
    afe_irq_pending = false;
    afe_irq1_pending = false;
    afe_irq2_pending = false;
  }
  interrupts();

  if (!pending) {
    return false;
  }
  serviceAllInterrupts();
  return true;
}

void MAX30001G::serviceAllInterrupts() {
/* General Interrupt Handling Routine
 * This will update global variables
 * The main program will need to check the variables
 */

  const uint32_t statusWord = readStatusAndLatchFlags();
  LOGD("Interrupt Status: 0x%06lX", (unsigned long)statusWord);

  if (status.bit.eint) {
    LOGD("ECG FIFO interrupt");
  }

  if (status.bit.eovf) {
    LOGD("ECG buffer overflow detection interrupt");
  }

  if (status.bit.fstint) {
    LOGD("Fast recovery interrupt");
  }

  if (status.bit.dcloffint) {
    LOGD("ECG lead off interrupt");
  }

  if (status.bit.bint) {
    LOGD("BIOZ FIFO interrupt");
  }

  if (status.bit.bovf) {
    LOGD("BIOZ FIFO overflow interrupt");
  }

  if (status.bit.bover) {
    LOGD("BIOZ over voltage interrupt");
  }

  if (status.bit.bundr) {
    LOGD("BIOZ under voltage interrupt");
  }

  if (status.bit.bcgmon) {
    LOGD("BIOZ current generator monitor interrupt");
  }

  if (status.bit.lonint) {
    LOGD("Leads on detection");
  }

  if (status.bit.rrint) {
    LOGD("R to R interrupt");
  }

  if (status.bit.pllint) {
    LOGD("PLL interrupt");
  }

  CallbackRegistry* registry = getRegistry(this, false);

  // Callback dispatch happens after default status-to-flag propagation.
  if (registry != nullptr &&
      registry->aggregateCallback != nullptr &&
      ((statusWord & registry->aggregateMask) != 0U)) {
    registry->aggregateCallback(this, statusWord, registry->aggregateContext);
  }

  dispatchEventCallback(registry, this, IRQ_ECG_FIFO,    statusWord);
  dispatchEventCallback(registry, this, IRQ_ECG_OVF,     statusWord);
  dispatchEventCallback(registry, this, IRQ_ECG_FAST,    statusWord);
  dispatchEventCallback(registry, this, IRQ_ECG_LEADOFF, statusWord);
  dispatchEventCallback(registry, this, IRQ_BIOZ_FIFO,   statusWord);
  dispatchEventCallback(registry, this, IRQ_BIOZ_OVF,    statusWord);
  dispatchEventCallback(registry, this, IRQ_BIOZ_OVER,   statusWord);
  dispatchEventCallback(registry, this, IRQ_BIOZ_UNDER,  statusWord);
  dispatchEventCallback(registry, this, IRQ_BIOZ_CGMON,  statusWord);
  dispatchEventCallback(registry, this, IRQ_LEADS_ON,    statusWord);
  dispatchEventCallback(registry, this, IRQ_RTOR,        statusWord);
  dispatchEventCallback(registry, this, IRQ_PLL_UNLOCK,  statusWord);
}

void MAX30001G::readAndClearFaults(void) { // Read and clear any faults in the device
  readStatusAndLatchFlags();

  if (status.bit.eovf || status.bit.bovf) {
    FIFOReset();
  }
}

bool MAX30001G::setInterruptEventCallback(InterruptEvent event, InterruptCallback cb, void* context) {
  if (!isValidEvent(event)) {
    LOGE("Invalid interrupt event callback registration.");
    return false;
  }
  CallbackRegistry* registry = getRegistry(this, true);
  if (registry == nullptr) {
    LOGE("No free callback registry slots (max %u).", (unsigned)kMaxCallbackDevices);
    return false;
  }
  const uint8_t idx = static_cast<uint8_t>(event);
  registry->eventCallbacks[idx] = cb;
  registry->eventContexts[idx] = context;
  return true;
}

bool MAX30001G::setInterruptAggregateCallback(InterruptCallback cb, void* context, uint32_t statusMask) {
  CallbackRegistry* registry = getRegistry(this, true);
  if (registry == nullptr) {
    LOGE("No free callback registry slots (max %u).", (unsigned)kMaxCallbackDevices);
    return false;
  }
  registry->aggregateCallback = cb;
  registry->aggregateContext = context;
  registry->aggregateMask = statusMask & kDefaultStatusMask;
  return true;
}

void MAX30001G::clearInterruptEventCallback(InterruptEvent event) {
  if (!isValidEvent(event)) {
    return;
  }
  CallbackRegistry* registry = getRegistry(this, false);
  if (registry == nullptr) {
    return;
  }
  const uint8_t idx = static_cast<uint8_t>(event);
  registry->eventCallbacks[idx] = nullptr;
  registry->eventContexts[idx] = nullptr;
  if (isRegistryEmpty(*registry)) {
    registry->device = nullptr;
  }
}

void MAX30001G::clearInterruptAggregateCallback(void) {
  CallbackRegistry* registry = getRegistry(this, false);
  if (registry == nullptr) {
    return;
  }
  registry->aggregateCallback = nullptr;
  registry->aggregateContext = nullptr;
  registry->aggregateMask = kDefaultStatusMask;
  if (isRegistryEmpty(*registry)) {
    registry->device = nullptr;
  }
}

void MAX30001G::clearAllInterruptCallbacks(void) {
  CallbackRegistry* registry = getRegistry(this, false);
  if (registry == nullptr) {
    return;
  }
  registry->aggregateCallback = nullptr;
  registry->aggregateContext = nullptr;
  registry->aggregateMask = kDefaultStatusMask;
  for (uint8_t i = 0; i < static_cast<uint8_t>(IRQ_EVENT_COUNT); i++) {
    registry->eventCallbacks[i] = nullptr;
    registry->eventContexts[i] = nullptr;
  }
  registry->device = nullptr;
}

void MAX30001G::handleECGFifoInterrupt(bool reportRaw) {
  readECG_FIFO(reportRaw);
}

void MAX30001G::handleBIOZFifoInterrupt(bool reportRaw) {
  readBIOZ_FIFO(reportRaw);
}

void MAX30001G::handleRtoRInterrupt(void) {
  readRTOR();
}


bool MAX30001G::setInterrupt(uint8_t interrupt, bool ecg, bool bioz, bool rtor, bool leadon, bool leadoff, bool bioz_fourwire) {
/*
  Enable/Disable Interrupts.

  There are two interrupt lines and each can be triggered by setting its registers
  Here we set the first interrupt line.

  interrupt:      MAX30001_EN_INT1 or MAX30001_EN_INT2
  ecg:            ECG related interrupts enabled
  bioz:           BIOZ related interrupts enabled
  rtor:           R to R detection enabled
  leadon:         Leads on detection enabled (requires custom startup)
  leadoff:        Leads off detection enabled
  bioz_fourwire:  Leads off detection with under and over voltage detection (4 wire BIOZ)

Registers involved

General

  en_int1.bit.intb_type
    Hardware logic used for the interrupt pin
    0 disabled
    1 CMOS **
    2 OpenDrain
    3 OpenDrain with internal pull up * default

BIOZ

  en_int1.bit.en_bint       
    BIOZ FIFO
    set if threshold for available samples in FIFO is met
    remains active until the FIFO is read to the extent required by BFIT

  en_int1.bit.en_bovf       
    BIOZ FIFO overflow
    Overlow of FIFO occured. Remains active until FIFO reset.

  en_int1.bit.en_bcgmon     
    BIOZ current monitor
    DRVN/P is in lead off condition
    Remains on as long as condition persist, cleared by reading status
    4 electrode  BIOZ

  en_int1.bit.en_bunder
    BIOZ under range
    Output has exceeded low threshold (BLOFF_LO_IT).
    Remains on as long as condition persists, cleared by reading status
    4  electrode BIOZ with lead off detection enabled

  en_int1.bit.en_bover      
    BIOZ over range
    Output has exceeded high threshold (BLOFF_HI_IT).
    Remains on as long as condition persists, cleared by reading status
    2 or 4 electrode BIOZ with lead off detection enabled

  en_int1.bit.en_pllint     
    PLL unlocked
    PLL has lost its phase or is not yet active
    Can occur when ECG or BIOZ channels are enabled
    Remains on until PLL is locked and status is read

ECG

  en_int1.bit.en_eint       
    ECG FIFO
    set if threshold for evailable samples in FIFO is met
    remains active until the FIFO is read to the extent required by EFIT

  en_int1.bit.en_fstint     
    ECG fast recovery
    Manual or automatic recovery started
    Behavious set by CLR_FAST in MNGR_INT 
  
  en_int1.bit.en_eovf       
    ECG FIFO overflow
    Overflow in the FIFO occured. Remains active until FIFO reset

  en_int1.bit.en_samp       
    Sample synchronization pulse
    To synchronize other external hardware based on ECG sampling instant

RTOR

    en_int1.bit.en_rrint      
    ECG R to R detection
    Cleared based on CLR_RRINT in MNGR_INT register

ECG & BIOZ

  en_int1.bit.en_dcloffint  
    DC lead off detection
    Remains on until lead on and cleared by status read

  en_int1.bit.en_lonint     
    Ultra low power leads on detection
    EN_ULP_LON in CNFG_GEN determine the condition
    Remains on while leads are on and cleared by status read.

*/

  max30001_en_int_reg_t *en_int;

  // Select the correct interrupt register
  if (interrupt == MAX30001_EN_INT1) {
    en_int = &en_int1;
  } else if (interrupt == MAX30001_EN_INT2) {
    en_int = &en_int2;
  } else {
    LOGE("Invalid interrupt selection.");
    return false;
  }

  en_int->all  = readRegister24(interrupt);

  // Interrupt hardware
  // 0 disabled, 
  // 1 CMOS, ** faster, actively drives high and low
  // 2 OpenDrain, 
  // 3 OpenDrain with internal pull up
  en_int->bit.intb_type     = 1;                  // CMOS push-pull
  en_int->bit.en_samp       = 0;                  // no external hardware needs sampling pulse

  // ECG related
  en_int->bit.en_eint       = ecg  ? 1 : 0;       // ECG FIFO interrupt enable
  en_int->bit.en_fstint     = ecg  ? 1 : 0;       // ECG fast recovery interrupt enable
  en_int->bit.en_eovf       = ecg  ? 1 : 0;       // ECG FIFO overflow interrupt enable
  en_int->bit.en_rrint      = rtor ? 1 : 0;       // ECG R to R detection interrupt enable

  // BIOZ related
  en_int->bit.en_bint       = bioz ? 1 : 0;       // BIOZ FIFO interrupt enable
  en_int->bit.en_bovf       = bioz ? 1 : 0;       // BIOZ FIFO overflow interrupt enable
  en_int->bit.en_pllint     = 0;                  // PLL unlocked, PLL has lost its phase or is not yet active

  // Leads On
  en_int->bit.en_lonint     = leadon ? 1 : 0;     // Ultra low power leads on detection interrupt enable

  // Leads Off
  en_int->bit.en_dcloffint  = 0;                  // reset
  en_int->bit.en_bunder     = 0;                  // reset
  en_int->bit.en_bover      = 0;                  // reset
  en_int->bit.en_bcgmon     = 0;                  // reset
  if (leadoff) {
    en_int->bit.en_dcloffint = 1;                 // DC lead-off detection enable
    if (bioz) {
      en_int->bit.en_bover = 1;                   // BIOZ over range interrupt enable
    }
    if (bioz && bioz_fourwire) {
      en_int->bit.en_bunder = 1;                  // BIOZ under range interrupt enable
      en_int->bit.en_bcgmon = 1;                  // BIOZ current monitor (only if leadoff & fourwire)
    }
  }

  writeRegister(interrupt, en_int->all);
  if (!verifRegister(interrupt, en_int->all)) {
    LOGE("Failed to set interrupt register.");
    return false;
  }
  return true;
}


void MAX30001G::setDefaultInterruptClearing(){
/*
 * Set defaults for interrupt clearing
 * We will clear automatically or when data register is read
 
 mngr_int.bit.clr_rrint
  Clear RR Interrupt
  0 - on status read
  1 - on R to R read **
  2 - self clear after 2-8ms
  3 - do not use
  
 mngr_int.bit.clr_fast
  Fast Mode Clear Behaviour
  0 - clear on read **
  1 - clear on read and fast recovery is complete
 
 mngr_int.bit.clr_samp
  Sample synchronization pulse clear behaviour
  0 - clear on read
  1 - self clear after 1/4 of data cycle **
 
 mngr_int.bit.samp_it
  Sample Synchronization Pulse Frequency
  0 - every sample instant **
  1 - every 2nd sample instant
  2 - every 4th sample instant
  3 - every 16th sample instant
 
*/
  
  mngr_int.all  = readRegister24(MAX30001_MNGR_INT);

  // Interrupt Clearing Behaviour

  mngr_int.bit.clr_rrint = 1; // clear on RR register read
  mngr_int.bit.clr_fast  = 0; // clear on status register read
  mngr_int.bit.clr_samp  = 1; // self clear
  mngr_int.bit.samp_it   = 0; // every sample

  writeRegister(MAX30001_MNGR_INT, mngr_int.all);
}

void MAX30001G::setFIFOInterruptThreshold(uint8_t ecg,  uint8_t bioz) {
/* 
 Number of samples in FIFO that will trigger an interrupt

  ecg  1..32
  bioz 1..8

 mngr_int.bit.e_fit
  ECG interrupt treshold
  value+1 = 1-32 unread samples

 mngr_int.bit.b_fit
  BIOZ interrupt treshold
  value+1 = 1-8 unread samples
*/
  mngr_int.all  = readRegister24(MAX30001_MNGR_INT);

  if ((bioz <= 8) && (bioz > 0)) {
    mngr_int.bit.b_fit = bioz-1; // 1-8  BIOZ interrupt treshold
    LOGln("BIOZ interrupt treshold set to %d - 1", bioz);
  } else {
    LOGE("BIOZ interrupt treshold out of range: 1..8");
  }

  if ((ecg <= 32) && (ecg > 0)) {
    mngr_int.bit.e_fit = ecg-1; // 1-32  ECG interrupt treshold
    LOGln("ECG interrupt treshold set to %d - 1", ecg);
  } else {
    LOGE("ECG interrupt treshold out of range: 1..32");
  }

  writeRegister(MAX30001_MNGR_INT, mngr_int.all);

}
