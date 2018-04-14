#ifndef PIC_CCP_H_
#define PIC_CCP_H_

#include <xc.h>
#include "pic_common.h"


/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  Initialization of ccp module in pwm mode.
 *
 */
void pic_ccp_pwm_init(PIC12_PIN_t pin);

/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  PWM uses TMR2 so we need to enable it.
 *
 */
void pic_ccp_pwm_enable_clock(void);

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
void pic_ccp_set_compare(uint16_t compare_val);

#endif /* PIC_CCP_H_ */
