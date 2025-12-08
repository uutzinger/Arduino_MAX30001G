#ifndef MAX30001G_REGS_TYPEDEFS_H
#define MAX30001G_REGS_TYPEDEFS_H

#include <stdint.h>

/**
 * @brief STATUS (0x01)
 * page 42
 **/
typedef union
{
  uint32_t all;

  struct
  {
    uint32_t ldoff_nl  : 1; //  0   lead off detection status 
    uint32_t ldoff_nh  : 1; //  1
    uint32_t ldoff_pl  : 1; //  2
    uint32_t ldoff_ph  : 1; //  3

    uint32_t bcgmn     : 1; //  4   BIOZ channel gain monitor, negative output
    uint32_t bcgmp     : 1; //  5   BIOZ channel gain monitor, positive output
    uint32_t reserved1 : 1; //  6  
    uint32_t reserved2 : 1; //  7

    uint32_t pllint    : 1; //  8   PLL unlock interrupt
    uint32_t samp      : 1; //  9   sample synchronization pulse
    uint32_t print     : 1; // 10   ECG R2R detection interrupt
    uint32_t lonint    : 1; // 11   Ultra low power leads on detection interrupt

    uint32_t pedge     : 1; // 12 * not described in the datasheet
    uint32_t povf      : 1; // 13 *
    uint32_t print     : 1; // 14 *
    uint32_t bcgmon    : 1; // 15   BIOZ current monitor

    uint32_t bundr     : 1; // 16   BIOZ under range
    uint32_t bover     : 1; // 17   BIOZ over range
    uint32_t bovf      : 1; // 18   BIOZ FIFO  flow
    uint32_t bint      : 1; // 19   BIOZ FIFO interrupt

    uint32_t dcloffint : 1; // 20   DC lead off detection interrupt
    uint32_t fstint    : 1; // 21   ECG fast recovery mode is engaged
    uint32_t eovf      : 1; // 22   ECG FIFO overflow
    uint32_t eint      : 1; // 23   ECG FIFO interrupt

    uint32_t reserved3 : 8; //24-31

  } bits;

} max30001_status_reg_t;

/**
 * @brief EN_INT (0x02) and (0x03)
 * page 43
 * we can attach two interrupt lines to functions of the MAX30001
 * multiple interrupts can be enabled at the same time as the bits are OR-ed
 **/
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t int_type     : 2; // 0,1   interrupt type, 00 disabled, CMOS, OpenDrain, OpenDrain with internal pull up
    uint32_t reserved1    : 6; // 2-7
    uint32_t en_pllint    : 1; // 8     PLL unlock interrupt enable
    uint32_t en_samp      : 1; // 9     sample synchronization pulse enable
    uint32_t en_rrint     : 1; // 10    ECG R to R detection interrupt enable
    uint32_t en_lonint    : 1; // 11    Ultra low power leads on detection interrupt enable
    uint32_t reserved2    : 3; // 12-14
    uint32_t en_bcgmon    : 1; // 15    BIOZ current monitor interrupt enable
    uint32_t en_bunder    : 1; // 16    BIOZ under range interrupt enable
    uint32_t en_bover     : 1; // 17    BIOZ over range interrupt enable
    uint32_t en_bovf      : 1; // 18    BIOZ FIFO overflow interrupt enable
    uint32_t en_bint      : 1; // 19    BIOZ FIFO interrupt enable
    uint32_t en_dcloffint : 1; // 20  DC lead off detection interrupt enable
    uint32_t en_fstint    : 1; // 21    ECG fast recovery interrupt enable
    uint32_t en_eovf      : 1; // 22    ECG FIFO overflow interrupt enable
    uint32_t en_eint      : 1; // 23    ECG FIFO interrupt enable
    uint32_t reserved3    : 8  // 24-31
  } bit;
} max30001_en_int_reg_t;

/**
 * @brief MNGR_INT (0x04)
 * page 44
 */
typedef union
{
  uint32_t all;

  struct
  {
    uint32_t samp_it   : 2; //  0,1  sample synchronization pulse frequency
    uint32_t clr_samp  : 1; //  2    sample synchronization pulse clear behavior
    uint32_t clr_pedge : 1; //  3 *
    uint32_t clr_rrint : 2; //  4,5  RTOR R peak detection clear behavior
    uint32_t clr_fast  : 1; //  6,   fast mode interrupt clear behavior
    uint32_t reserved1 : 1; //  7
    uint32_t reserved2 : 4; //  8-11
    uint32_t reserved3 : 4; // 12-15
    uint32_t b_fit     : 3; // 16-18 BIOZ interrupt treshold
    uint32_t e_fit     : 5; // 19-23 ECG interrupt treshold
    uint32_t reserved  : 8; // 24-31
  } bit;

} max30001_mngr_int_reg_t;

/**
 * @brief MNGR_DYN (0x05)
 * page 45
 */
typedef union
{
  uint32_t all;

  struct
  {
    uint32_t bloff_lo_it : 8; //  0-7   BIOZ lead off under range threshold
    uint32_t bloff_hi_it : 8; //  8-15  BIOZ lead off over range threshold
    uint32_t fast_th     : 6; // 16-21  ECG fast recovery threshold, 0x3F, if 2048*FAST_TH for more than 125ms go to recovery mode
    uint32_t fast        : 2; // 22,23  ECG fast recovery mode, 00 normal, 01 manual, 11 do not use
    uint32_t reserved    : 8; // 24-31
  } bit;

} max30001_mngr_dyn_reg_t;

/**
 * @brief INFO (0x0F)
 * page 42
 */
typedef union max30001_info_reg
{
  uint32_t all;

  struct
  {
    uint32_t n1        : 4; //  0- 3 might randomly vary
    uint32_t n2        : 4; //  4- 7 might randomly vary
    uint32_t n3        : 4; //  8-11 might randomly vary
    uint32_t c1        : 2; // 12-13 should be 0b01
    uint32_t n4        : 2; // 14-15 might vary randomly
    uint32_t revision  : 4; // 16-19 should be b0101
    uint32_t c2        : 4; // 20-23
    uint32_t reserved  : 8; // 24-31

  } bit;

} max30001_info_reg_t;

/**
 * @brief CNFG_GEN (0x10)
 * page 47
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t rbiasn     : 1; //  0,   enable resistive bias on N
    uint32_t rbiasp     : 1; //  1,   enable resistive bias on P
    uint32_t rbiasv     : 2; //  2,3  bias mode: 50, 100, 200MOhm, do notuse
    uint32_t en_rbias   : 2; //  4,5  disabled, ECG, BIOZ, reserved
    uint32_t vth        : 2; //  6,7  lead off threshold, 300, 400, 450, 500mV
    uint32_t imag       : 3; //  8-10 lead off magnitude selection, 0, 5, 10, 20, 50, 100 nA
    uint32_t ipol       : 1; // 11    lead off curren polarity, ECGN pullup, ECGN pull down
    uint32_t en_dcloff  : 2; // 12,13 lead off enable
    uint32_t en_bloff   : 2; // 14,15 BIOZ lead off enable
    uint32_t reserved1  : 1; // 16
    uint32_t en_pace    : 1; // 17 *  not in the datasheet
    uint32_t en_bioz    : 1; // 18    enable BIOZ channel
    uint32_t en_ecg     : 1; // 19    enable ECG channel
    uint32_t fmstr      : 2; // 20,21 master clock frequency
    uint32_t en_ulp_lon : 2; // 22,23 ultra low power lead on detection
    uint32_t reserved   : 8; // 24-31
  } bit;

} max30001_cnfg_gen_reg_t;

/**
 * @brief CNFG_CAL (0x12)
 * page 49
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t thigh      : 11; //  0-10, calibration time source selection
    uint32_t fifty      :  1; // 11     calibrationd duty cycle mode
    uint32_t fcal       :  3; // 12-14  calibration frequency selection
    uint32_t reserved1  :  5; // 15-19
    uint32_t vmag       :  1; // 20     calibration voltage magnitude
    uint32_t vmode      :  1; // 21     calibration voltage mode
    uint32_t vcal       :  1; // 22     calibration voltage enable
    uint32_t reserved2  :  1; // 23
    uint32_t reserved3  :  8; // 24-31
  } bit;

} max30001_cnfg_cal_reg_t;

/**
 * @brief CNFG_EMUX  (0x14)
 * page 50
 */
typedef union max30001_cnfg_emux_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1 : 16; // 0-15 
    uint32_t caln_sel  :  2; // 16,17 ECGN calibration selection
    uint32_t calp_sel  :  2; // 18,19 ECGP calibration selection
    uint32_t openn     :  1; // 20 open the ECG input switch
    uint32_t openp     :  1; // 21 open the ECG input switch
    uint32_t reserved2 :  1; // 22 
    uint32_t pol       :  1; // 23 ECG input polarity selection
    uint32_t reserved3 :  8; // 24-31
  } bit;

} max30001_cnfg_emux_reg_t;

/**
 * @brief CNFG_ECG   (0x15)
 * page 51
 */
typedef union max30001_cnfg_ecg_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1  : 12; //  0-11
    uint32_t dlpf       :  2; // 12,13 ecg digital low pass cut off
    uint32_t dhpf       :  1; // 14 ecg digtial high pass cut off 
    uint32_t reserved2  :  1; // 15
    uint32_t gain       :  2; // 16,17 ecg gain
    uint32_t reserved3  :  4; // 18-21 
    uint32_t rate       :  2; // 22,23 ecg rate
    uint32_t reserved   :  8;
  } bit;

} max30001_cnfg_ecg_reg_t;

/**
 * @brief CNFG_BMUX   (0x17)
 * page 52
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t fbist      : 2; //  0,1  BIOZ frequency selection
    uint32_t reserved1  : 2; //  2,3  
    uint32_t rmod       : 3; //  4-6  BIOZ modulated resistance selection
    uint32_t reserved2  : 1; //  7    
    uint32_t rnom       : 3; //  8-10 BIOZ nominal resistance selection
    uint32_t en_bist    : 1; // 11    BIOZ bistable mode enable
    uint32_t cg_mode    : 2; // 12,13 BIOZ current generator mode selection
    uint32_t reserved3  : 2; // 14,15  
    uint32_t caln_sel   : 2; // 16,17 BI n calibration selection
    uint32_t calp_sel   : 2; // 18,19 BI p calibration selection
    uint32_t openn      : 1; // 20    open the BIOZ N input switch
    uint32_t openp      : 1; // 21    open the BIOZ P input switch 
    uint32_t reserved4  : 2; // 22,23 
    uint32_t reserved   : 8; // 24-31
  } bit;

} max30001_cnfg_bmux_reg_t;

/**
 * @brief CNFG_BIOZ   (0x18)
 * page 54
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t phoff      : 4; // 0-3   BIOZ modulation phase offset
    uint32_t cgmag      : 3; // 4-6   BIOZ current generator magnitude 8,16,32,48,64,80,96 microA
    uint32_t cgmon      : 1; // 7     BIOZ current generator monitor
    uint32_t fcgen      : 4; // 8-11  BIOZ current generator frequency
    uint32_t dlpf       : 2; // 12,13 BIOZ digital low pass cut off 4,8,16Hz
    uint32_t dhpf       : 2; // 14,15 BIOZ digital high pass cut off 0.05, 0.5Hz
    uint32_t gain       : 2; // 16,17 BIOZ gain 10,20,40,80V/V
    uint32_t ln_bioz    : 1; // 18    BIOZ INA power mode
    uint32_t ext_rbias  : 1; // 19    BIOZ external resistive bias enable
    uint32_t ahpf       : 3; // 20-22 BIOZ analog high pass cut off, at 60, 150, 500, 1000, 2000, 4000Hz
    uint32_t rate       : 1; // 23    BIOZ rate samples per second
    uint32_t reserved   : 8; // 24-31 
  } bit;

} max30001_cnfg_bioz_reg_t;

/**
 * @brief CNFG_BIOZ_LC (0x1A)
 * page 57
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t cmag_lc    : 4; // 0-3   Bioz low current manitude selection
    uint32_t cmres      : 4; // 4-7   Bioz low current mode resistor selection
    uint32_t reserved1  : 4; // 8-11
    uint32_t bistr      : 2; // 12,13 High resistance programmable load value selection
    uint32_t en_bistr   : 1; // 14    High resistance programmable load enable
    uint32_t reserved2  : 4; // 15-18
    uint32_t lc2x       : 1; // 19    Bioz low current 2x mode enable
    uint32_t reserved3  : 3; // 20-22
    uint32_t hi_lob     : 1; // 23    Bioz high or low current mode selection
    uint32_t reserved3  : 8; // 24-31 
  } bit;

} max30001_cnfg_bioz_lc_reg_t;

/**
 * @brief CNFG_RTOR1   (0x1D)
 * page 59
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t reserved1  : 8; // 0-7   
    uint32_t ptsf       : 4; // 8-11  R tp R peak threshold scaling facto
    uint32_t pavg       : 2; // 12,13 R to R peak averaging weight factor
    uint32_t reserved2  : 1; // 14    
    uint32_t en_rtor    : 1; // 15    ECG R to R detection enable
    uint32_t gain       : 4; // 16-19 R to R gain 
    uint32_t wndw       : 4; // 20-23 Width of averaging window
    uint32_t reserved   : 8; // 24-31
  } bit;

} max30001_cnfg_rtor1_reg_t;

/**
 * @brief CNFG_RTOR2 (0x1E)
 * page 59
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t reserved1  : 8; // 0-7
    uint32_t rhsf       : 3; // 8-10 R to R peak threshold hold off scaling factor
    uint32_t reserved2  : 1; // 11
    uint32_t ravg       : 2; // 12,13 R to R peak averaging interval weight factor
    uint32_t reserved3  : 2; // 14,15
    uint32_t hoff       : 6; // 16-21 R to R peak detection threshold hold off
    uint32_t reserved4  : 2; // 22,23
    uint32_t reserved   : 8; // 24-31
  } bit;

} max30001_cnfg_rtor2_reg_t;

/**
 * @brief ECG_FIFO_BURST
 * page 61
 */
typedef union max30001_ecg_burst_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved :   3; //  0-2
    uint32_t etag     :   3; //  3-5 ECG tag
    uint32_t data      :  18; //  6-23 ECG data  
  } bit;

} max30001_ecg_burst_reg_t;

/**
 * @brief BIOZ_FIFO_BURST
 * page 61
 */
typedef union
{
  uint32_t all;
  struct
  {
    uint32_t btag     :   3; //  0-2 BIOZ tag
    uint32_t reserved :   1; //  3
    uint32_t data     :  20; //  4-23 BIOZ data  
  } bit;

} max30001_bioz_burst_reg_t;


/**
 * @brief R to R FIFO
 * page 61
 */
typedef union max30001_rtor_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved :  10; // 
    uint32_t data     :  14; //  10-23 BIOZ data  
  } bit;

} max30001_rtor_reg_t;

#endif

