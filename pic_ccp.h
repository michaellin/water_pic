#ifndef PIC_CCP_H_
#define PIC_CCP_H_

typedef enum PIC12_PINS {
    RA1,
    RA2,
    RA3,
    RA4,
    RA5
}


/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  Initialization of ccp module in pwm mode.
 *
 */
void pic_ccp_pwm_init(PIC12_PINS pin);

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
void pic_ccp_set_period(uint8_t period_val);

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
void pic_ccp_set_compare(uint8_t compare_val);

#endif /* PIC_CCP_H_ */
