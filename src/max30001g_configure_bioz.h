/******************************************************************************************************/
// Configure ECG
/******************************************************************************************************/
#ifndef MAX30001G_CONFIG_BIOZ_H
#define MAX30001G_CONFIG_BIOZ_H

#include <stdint.h>

class MAX30001G {
    public:
    void setDefaultBIOZImpedanceTest(); // 5kOhm modulation at 1Hz with 3 Ohm
    void setDefaultNoBIOZImpedanceTest();
    
    private:
    void setBIOZSamplingRate(uint8_t BIOZ);
    /* 0 =  low   25-32 sps * default
       1 =  high  50-64 sps
    */
    void setBIOZgain(uint8_t gain, bool lowNoise);
    /* 0 = 10V/V * default
       1 = 20V/V
       2 = 40V/V
       3 = 80V/V
    */

    /*idx    freq          nA
        0 128 000  55 - 96000
        1  80 000  55 - 96000
        2  40 000  55 - 96000
        3  18 000  55 - 96000
        4   8 000  55 - 80000
        5   4 000  55 - 32000
        6   2 000  55 - 16000
        7   1 000  55 -  8000
        8     500  55 -  8000
        9     250  55 -  8000
       10     125  55 -  8000
    */

    void setBIOZModulationFrequencyByFrequency(uint16_t frequency);
    /* 0..10 frequency selectable */

    void setBIOZModulationFrequencybyIndex(uint8_t frequency_selector);
    /* 0..10 with 0 approx 128kHz and 10 approx 125Hz*/

    void setBIOZmag(uint32_t current);
    /* Set modulation frequency first, then you can set the modulation current
         nA
       high current
       96 000
       80 000
       64 000
       48 000
       32 000
       16 000
        8 000 works with all frequencies
      low current
        1 100
          880
          660
          440
          330
          220
          110
           55
           0
       also sets common mode current appropriate feedback
     */

    void setBIOZPhaseOffsetbyIndex(uint8_t selector);
    /* phase offset  = selector * phase_incr
       phase_incr = 0.0, 22.5, 45.0 degrees
       selector = 0..15
       128kHz:          0.. 3, 45.00deg
       80kHz:           0.. 7, 22.50deg
       40kHz and lower: 0..15, 11.25deg
     */

     void setBIOZPhaseOffsetbyPhase(uint16_t frequency, float phase);
    /* phase offset  = selector * phase_incr
       phase_incr = 0.0, 22.5, 45.0 degrees
       selector = 0..15
       128kHz:          0.. 3, 45.00deg
       80kHz:           0.. 7, 22.50deg
       40kHz and lower: 0..15, 11.25deg
     */

    void setBIOZfilter(uint8_t ahpf, uint8_t lpf, uint8_t hpf);
    /*
      analog high pass is 
        [6,7]  0 Hz
        [0]   60 Hz
        [1]  150 Hz
        [2]  500 Hz * default
        [3] 1000 Hz
        [4] 2000 Hz 
        [5] 4000 Hz
        analog filter is located before demodulator

      digital low pass is approx
        [0] 0  Hz
        [1] 4  Hz * default
        [2] 8  Hz
        [3] 16 Hz 
        higher lowpass frequencies will introduce more noise but would allow better visualization
        of faster changing signals such as cardiac impedance changes 

      digital high pass is 
        [0] 0    Hz * default
        [1] 0.05 Hz 
        [2] 0.5  Hz
        if you use digital highpass filter this will remove slow variations of the impedance
        applying digital high pass will remove absolute impedance values and at 0.5Hz impedance changes 
        due to breathing artifacts would not longer be visible.
      
      digital filters are after demodulator
    */
    void setBIOZmodulation(uint8_t mode);0
    /*
     0 = Un chopped Sources with Low Pass Filter * default
         (higher noise, excellent 50/60Hz rejection, recommended for ECG, BioZ applications)
     1 = Chopped Sources without Low Pass Filter
         (low noise, no 50/60Hz rejection, recommended for BioZ applications with digital LPF, possibly battery powered ECG, BioZ applications)
     2 = Chopped Sources with Low Pass Filter
         (low noise, excellent 50/60Hz rejection)
     3 = Chopped Sources with Resistive CM
         (Not recommended to be used for drive currents >32µA, low noise, excellent 50/60Hz rejection, lower input impedance)
     */

    void setBIOZCurrentMonitor(bool enable);
    /* enable or disable current monitor, results are in status register */

    void setBIOZImpedanceTest(bool enable, bool useHighResistance, bool enableModulation, uint8_t resistance, uint8_t rmodValue,  uint8_t modFreq);
    /*
      - enable/disable use of any internal impedance test
      - useHighResistance: true to use high-resistance load (27 kΩ to 1029 kΩ), false to use low-resistance modulated load.
      - resistance
          high resistance
            0 00 =   27 kΩ
            1 01 =  108 kΩ
            2 10 =  487 kΩ
            3 11 = 1029 kΩ
          low resistance
            0 000 =    5 k
            1 001 =    2.5k
            2 010 =    1.667k
            3 011 =    1.25k
            4 100 =    1k
            5 101 =    0.833k
            6 110 =    0.714k
            7 111 =    0.625k

      - enableModulation: true to enable modulated resistance, false to disable.
          For low resistance values between 625Ω and 5kΩ, the resistance can be modulated. 
          Resistance will switch between RNOM (resistance) and (RNOM - RMOD) at the selected modulation rate. 
      - rmodValue: Modulated resistance setting (0 to 7, RMOD). Only applicable when using low resistance.    
      - modFreq: resistance modulation frequency (0 to 3). Only applicable when using low resistance.
                0=4Hz, 1=1Hz, 2=0.25Hz, 3=0.0625Hz

      BMUX_RNOM[2:0]  BMUX_RMOD[2:0]  NOMINAL RESISTANCE (Ω)  MODULATED RESISTANCE (mΩ)
      000             000             5000                    2960
                      001                                     980.5
                      010                                     247.5
                      1xx                                     none
      001             000             2500                    740.4
                      001                                     245.2
                      010                                     61.9
                      1xx                                     none
      010             000             1667                    329.1
                      001                                     109.0
                      010                                     27.5
                      1xx                                     none
      011             000             1250                    185.1
                      001                                     61.3
                      010                                     none
                      1xx                                     none
      100             000             1000                    118.5
                      001                                     39.2
                      010                                     none
                      1xx                                     none
      101             000              833                    82.3
                      001                                     27.2
                      010                                     none
                      1xx                                     none
      110             000              714                    60.5
                      001                                     20.0
                      010                                     none
                      1xx                                     none
      111             000              625                    46.3
                      001                                     15.3
                      010                                     none
                      1xx                                     none

    */

};

#endif