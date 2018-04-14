//  Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated (the "Company")
// for its PICmicro® Microcontroller is intended and supplied to you, the Company?s
// customer, for use solely and exclusively on Microchip PICmicro Microcontroller
// products.
//
// The software is owned by the Company and/or its supplier, and is protected under
// applicable copyright laws. All rights are reserved. Any use in violation of the
// foregoing restrictions may subject the user to criminal sanctions under applicable
// laws, as well as to civil liability for the breach of the terms and conditions of
// this license.
//
// THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, WHETHER EXPRESS,
// IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE
// COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
////


//**********************************************************************************
// Example program using the standard PWM on a PIC12F1822 to dim an LED
//
// Device: PIC12F1822
// Compiler: Microchip XC8 v1.11
// IDE: MPLAB X v1.5
// Created: January 2013
//
// In this simple example we show how to set up the standard PWM peripheral and
// calculate/set the PWM frequency and duty cycle.
//**********************************************************************************
//          Pinout for this example
//          ----------
//      Vdd |1      8| GND
//      RA5 |2      7| RA0
//      RA4 |3      6| RA1 -> ADC input connected to humidity sensor
//      RA3 |4      5| RA2 -> PWM output connected to pump
//          ----------
//
//***********************************************************************************

#include <xc.h> // include standard header file
#include "pic_ccp.h"
#include "pic_osc.h"
#include "pic_adc.h"

// set Config bits
#pragma config FOSC=INTOSC, PLLEN=OFF, WDTE=OFF, MCLRE=ON,
#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,CP=OFF, CPD=OFF,BOREN=OFF
#pragma config WRT=OFF,STVREN=ON,BORV=LO,LVP=OFF

// Definitions
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions

uint16_t ADC_result;

void interrupt ADC_ISR()
{
//	ADC_result = (uint16_t)(ADRESH << 2U) | (uint16_t)(ADRESL >> 6U); // read adc value
    ADC_result = ((uint16_t) ADRESL) | ((uint16_t)ADRESH) << 8U;
    PIR1bits.ADIF = 0;	// clear the the interrupt flag
}


//**********************************************************************************
//*****************   main routine   ***********************************************
//**********************************************************************************
void main ( ) 
{
    /* PIC Init */
    // set up oscillator control register
    pic_osc_init(osc_16_mhz_hf);

    // Set up I/O pins
    ANSELAbits.ANSELA=0;    // set all analog pins to digital I/O

    // PORT A Assignments
    TRISAbits.TRISA0 = 0;	// RA0 = nc
    TRISAbits.TRISA1 = 0;	// RA1 = Analog Voltage In 
    TRISAbits.TRISA2 = 1;	// RA2 = PWM Output (CCP1) connected to LED
    TRISAbits.TRISA3 = 0;	// RA3 = nc (MCLR)
    TRISAbits.TRISA4 = 0;	// RA4 = nc
    TRISAbits.TRISA5 = 0;	// RA5 = nc

	// Init ADC 
    pic_adc_init(pin_RA2);

	// Init PWM
    pic_ccp_pwm_init(pin_RA5);
    pic_ccp_set_period(255);
    pic_ccp_set_compare(255);
    pic_ccp_pwm_enable_clock();


	ADCON0bits.ADGO = 1;		// Trigger an ADC conversion

    do{
        pic_ccp_set_compare(1000);

    } while (1);


}




