/******************************************************************************************************/
// Interrupt Setting and Handling
/******************************************************************************************************/
#include "logger.h"
#include "max30001g_comm.h"
#include "max30001g_globals.h"
#include "max30001g_defs.h"
#include "max30001g_interrupt.h"


/******************************************************************************************************/

void MAX30001G::serviceAllInterrupts() {
/* General Interrupt Handling Routine
 * This will update global variables
 * The main program will need to check the variables
 */

  status.all = readRegister24(MAX30001_STATUS);

  LOGD("Interrupt Status: %b", status);

  if (status.bit.eint) {
    LOGD("ECG FIFO interrupt");
    ecg_available = true;
  }

  if (status.bit.eovf) {
    LOGD("ECG buffer overflow detection interrupt");
    ecg_overflow_occurred = true;
  }

  if (status.bit.fstint) {
    LOGD("Fast recovery interrupt");
    ecg_fast_recovery_occurred = true;
  }

  if (status.bit.dcloffint) {
    LOGD("ECG lead off interrupt");
    ecg_lead_off = true;
  }

  if (status.bit.bint) {
    LOGD("BIOZ FIFO interrupt");
    bioz_available = true;
  }

  if (status.bit.bovf) {
    LOGD("BIOZ FIFO overflow interrupt");
    bioz_overflow_occurred = true;
  }

  if (status.bit.bover) {
    LOGD("BIOZ over voltage interrupt");
    bioz_overvoltage_occurred = true;
  }

  if (status.bit.bundr) {
    LOGD("BIOZ under voltage interrupt");
    bioz_undervoltage_occurred =  true;
  }

  if (status.bit.bcgmon) {
    LOGD("BIOZ current generator monitor interrupt");
    bioz_cgm_occurred = true;
  }

  if (status.bit.lonint) {
    LOGD("Leads on detection");
    leads_on_detected = true;
  }

  if (status.bit.print) {
    LOGD("R to R interrupt");
    rtor_available = true;
  }

  if (status.bit.pllint) {
    LOGD("PLL interrupt");
    pll_unlocked_occurred = true;
  }
}


bool MAX30001G::setInterrupt(uint8_t interrupt, bool ecg, bool bioz, bool rtor, bool leadson, bool leadsoff, bool bioz_fourwire) {
/*
  Enable/Disable Interrupts.

  There are two interrupt lines and each can be triggered by setting its registers
  Here we set the first interrupt line.

  ecg:            ECG related interrupts enabled
  bioz:           BIOZ related interrupts enabled
  rtor:           R to R detection enabled
  leadson:        Leads on detection enabled (requires custom startup)
  leadsoff:       Leads off detection enabled
  bioz_fourwire:  Leads off detection with under and over voltage detection (4 wire BIOZ)

Registers involved

General

  en_int1.bit.int_type
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
    return;
  }

  en_int.all  = readRegister24(interrupt);

  // Interrupt hardware
  // 0 disabled, 
  // 1 CMOS, ** faster, actively drives high and low
  // 2 OpenDrain, 
  // 3 OpenDrain with internal pull up
  en_int.bit.int_type      = 1;  
  en_int.bit.en_samp       = 0;                   // no external hardware needs sampling pulse

  // ECG related
  en_int.bit.en_eint       = ecg  ? 1 : 0;        // ECG FIFO interrupt enable
  en_int.bit.en_fstint     = ecg  ? 1 : 0;        // ECG fast recovery interrupt enable
  en_int.bit.en_eovf       = ecg  ? 1 : 0;        // ECG FIFO overflow interrupt enable
  en_int.bit.en_rrint      = rtor ? 1 : 0;        // ECG R to R detection interrupt enable

  // BIOZ related
  en_int.bit.en_bint       = bioz ? 1 : 0;        // BIOZ FIFO interrupt enable
  en_int.bit.en_bover      = bioz ? 1 : 0;        // BIOZ over range interrupt enable
  en_int.bit.en_bovf       = bioz ? 1 : 0;        // BIOZ FIFO overflow interrupt enable
  en_int.bit.en_pllint     = 0;                   // PLL unlocked, PLL has lost its phase or is not yet active

  // Leads On
  en_int.bit.en_lonint     = leadon;               // Ultra low power leads on detection interrupt enable

  // Leads Off
  en_int.bit.en_dcloffint  = 0;                   // reset
  en_int.bit.en_bunder     = 0;                   // reset
  if (leadsoff) {
    en_int.bit.en_dcloffint = 1;                  // DC lead-off detection enable
    if (bioz && bioz_fourwire) {
      en_int.bit.en_bunder = 1;                   // BIOZ under range interrupt enable
      en_int.bit.en_bcgmon = 1;                   // BIOZ current monitor (only if leadoff & fourwire)
    }
  }

  writeRegister(interrupt, en_int.all);
  if (!verifRegister(interrupt, en_int.all)) {
    LOGE("Failed to set interrupt register.");
    return false;
  }
  return true;
}


void MAX30001G::setDefaultInterruptClearing(){
/*
 * Set defaults for interrupt clearing
 * We will clear automatically or when data register is read
 
 mngr_int_reg.bit.clr_rrint
  Clear RR Interrupt
  0 - on status read
  1 - on R to R read **
  2 - self clear after 2-8ms
  3 - do not use
  
 mngr_int_reg.bit.clr_fast
  Fast Mode Clear Behaviour
  0 - clear on read **
  1 - clear on read and fast recovery is complete
 
 mngr_int_reg.bit.clr_samp  mngr_int_reg.bit.b_fit     = bioz;   // 0-8  BIOZ interrupt treshold
  Sample synchronization pulse clear behaviour
  0 - clear on read
  1 - self clear after 1/4 of data cycle **
 
 mngr_int_reg.bit.samp_it
  Sample Synchronization Pulse Frequency
  0 - every sample instant **
  1 - every 2nd sample instant
  2 - every 4th sample instant
  3 - every 16th sample instant
 
*/
  
  mngr_int_reg.all  = readRegister24(MAX30001_MNGR_INT);

  // Interrupt Clearing Behaviour

  mngr_int_reg.bit.clr_rrint = 1; // clear on RR register read
  mngr_int_reg.bit.clr_fast  = 0; // clear on status register read
  mngr_int_reg.bit.clr_samp  = 1; // self clear
  mngr_int_reg.bit.samp_it   = 0; // every sample

  writeRegister(MAX30001_MNGR_INT, mngr_int_reg.all);
}

void MAX30001G::setFIFOInterruptThreshold(uint8_t ecg,  uint8_t bioz) {
/* 
 Number of samples in FIFO that will trigger an interrupt

  ecg  1..32
  bioz 1..8

 mngr_int_reg.bit.e_fit
  ECG interrupt treshold
  value+1 = 1-32 unread samples

 mngr_int_reg.bit.b_fit
  BIOZ interrupt treshold
  value+1 = 1-8 unread samples
*/
  mngr_int_reg.all  = readRegister24(MAX30001_MNGR_INT);

  if ((bioz <= 8) && (bioz > 0)) {
    mngr_int_reg.bit.b_fit = bioz-1; // 1-8  BIOZ interrupt treshold
    LOGln("BIOZ interrupt treshold set to %d - 1", bioz);
  } else {
    LOGE("BIOZ interrupt treshold out of range: 1..8");
  }

  if ((ecg < 32) && (ecg > 0)) {
    mngr_int_reg.bit.e_fit = ecg-1; // 1-32  ECG interrupt treshold
    LOGln("ECG interrupt treshold set to %d - 1", ecg);
  } else {
    LOGE("ECG interrupt treshold out of range: 1..32");
  }

  writeRegister(MAX30001_MNGR_INT, mngr_int_reg.all);

}
