#ifndef PIC_OSC_H_
#define PIC_OSC_H_

#include <xc.h> // include processor files - each processor file is guarded.  

typedef enum PIC12_OSC_SPEEDS {
    osc_31_khz        = 1,
    osc_31_25_khz_mf  = 2,
    osc_31_25_khz_hf  = 3,
    osc_62_5_khz_mf   = 4,
    osc_125_khz_mf    = 5,
    osc_250_khz_mf    = 6,
    osc_500_khz_mf    = 7,
    osc_125_khz_hf    = 8,
    osc_250_khz_hf    = 9,
    osc_500_khz_hf    = 10,
    osc_1_mhz_hf      = 11,
    osc_2_mhz_hf      = 12,
    osc_4_mhz_hf      = 13,
    osc_8_mhz_hf      = 14,
    osc_16_mhz_hf     = 15,
} PIC12_OSC_SPEEDS_t;

/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  Initialization of pic with specific internal oscillator speed
 *
 */
void pic_osc_init(PIC12_OSC_SPEEDS_t speed);


void pic_osc_init(PIC12_OSC_SPEEDS_t speed) {
    OSCCONbits.SPLLEN=0;     // PLL is disabled
    OSCCONbits.IRCF = speed; //set OSCCON IRCF bits to specified speed
    OSCCONbits.SCS=0x02;     //set the SCS bits to select internal oscillator block
}

#endif
