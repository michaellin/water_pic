#ifndef PIC_ADC_H_
#define PIC_ADC_H_

#include <xc.h> // include processor files - each processor file is guarded.  

/**
 * @param none
 * @return
 *    None
 *
 *  Description:
 *  Initialization of pic with specific internal oscillator speed
 *
 */
void pic_adc_init(PIC12_PIN_t pin);


void pic_adc_init(PIC12_PIN_t pin) {
	ADCON0bits.ADON=0;		// turn ADC off
	switch (pin) {			// This selects which analog input to use for the ADC conversion
		case pin_RA0:
			ADCON0bits.CHS=0x00;	
			ANSELAbits.ANSELA=0x01;
			break;
		case pin_RA1:
			ADCON0bits.CHS=0x01;	
			ANSELAbits.ANSELA=0x02;
			break;
		case pin_RA2:
			ADCON0bits.CHS=0x02;	
			ANSELAbits.ANSELA=0x04;
			break;
		case pin_RA3:
			ADCON0bits.CHS=0x03;
			ANSELAbits.ANSELA=0x10;
			break;
		default:
			break;
	}
	ADCON1bits.ADCS=0x01;	// select ADC conversion clock select as Fosc/8
    ADCON1bits.ADFM=0x01;	// results are right justified
	ADCON0bits.ADON=1;		// turn ADC on

	PIR1bits.ADIF=0;     	// make sure the peripheral interrupt flag is clear
    PIE1bits.ADIE=1;        // enable ADC interrupt
	INTCONbits.PEIE=1;      // Enable peripheral interrupt
    INTCONbits.GIE=1;       // enable global interrupt
}

#endif
