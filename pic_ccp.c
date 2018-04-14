#include "pic_ccp.h"

/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  Initialization of ccp module in pwm mode.
 *
 */
void pic_ccp_pwm_init(PIC12_PIN_t pin) {
    switch (pin) {  // disable pwm pin output for the moment
      case pin_RA1:
        TRISAbits.TRISA1 = 1;
        break;
      case pin_RA2:
        TRISAbits.TRISA2 = 1;
        break;
      case pin_RA3:
        TRISAbits.TRISA3 = 1;
        break;
      case pin_RA4:
        TRISAbits.TRISA4 = 1;
        break;
      case pin_RA5:
        TRISAbits.TRISA5 = 1;
        break;
      default:
        break;
    }

    CCP1CONbits.CCP1M=0x0C;     // select PWM mode for CCP module
    CCP1CONbits.P1M=0x00;	      // select single output on CCP1 pin (RA5)

    CCPR1L =  0x00;             // clear high 8 bits of PWM duty cycle
    CCP1CONbits.DC1B=0x00;	    // clear low 2 bits of PWM Duty cycle

    PIR1bits.TMR2IF=0;		      // clear TMR2 interrupt flag
    T2CONbits.T2CKPS=0x00;      // select TMR2 prescalar as divide by 1 as per our example above

    switch (pin) {  // turn PWM output back on
      case pin_RA1:
        TRISAbits.TRISA1 = 0;
        break;
      case pin_RA2:
        TRISAbits.TRISA2 = 0;
        break;
      case pin_RA3:
        TRISAbits.TRISA3 = 0;
        break;
      case pin_RA4:
        TRISAbits.TRISA4 = 0;
        break;
      case pin_RA5:
        TRISAbits.TRISA5 = 0;
        break;
      default:
        break;
    }

    return;
}


/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  PWM uses TMR2 so we need to enable it.
 *
 */
void pic_ccp_pwm_enable_clock(void) {
    T2CONbits.TMR2ON=1;		      // turn TMR2 on
    return;
}


/**
 * @param uint16_t period match value
 * @return
 *    None
 *
 *  Description:
 *  Sets the timer period.
 *  PWM Period = (PR2 + 1) * 4 * TOSC * (TMR2 Prescale Value)
 *
 */
void pic_ccp_set_period(uint8_t period_val) {
    PR2 = period_val;                 // set PWM period as 255 per our example above
    return;
}

/**
 * @param uint16_t period match value
 * @return
 *    None
 *
 *  Description:
 *  Sets the timer period.
 *  PWM Period = (CCP1L:CCP1CON<5:4>) * TOSC * (TMR2 Prescale Value)
 *
 */
void pic_ccp_set_compare(uint16_t compare_val) {
    CCP1CONbits.DC1B=  compare_val & 0x03;        // set the 2 lsb bits of PWM duty cycle
    CCPR1L =  (compare_val >> 2);                 // set the 8 msb bits of PWM duty cycle
    return;
}
