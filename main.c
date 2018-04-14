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
//      RA4 |3      6| RA1
//      RA3 |4      5| RA2 -> PWM output connected to LED thru 200 Ohm resistor
//          ----------
//
//***********************************************************************************

#include <xc.h> // include standard header file
#include "pic_ccp.h"
#include "pic_osc.h"

// set Config bits
#pragma config FOSC=INTOSC, PLLEN=OFF, WDTE=OFF, MCLRE=ON,
#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,CP=OFF, CPD=OFF,BOREN=OFF
#pragma config WRT=OFF,STVREN=ON,BORV=LO,LVP=OFF

// Definitions
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions


//**********************************************************************************
//*****************   main routine   ***********************************************
//**********************************************************************************
void main ( ) 
{
    unsigned int DutyCycle;
    
    /* PIC Init */
    // set up oscillator control register
    pic_osc_init(osc_16_mhz_hf);

    // Set up I/O pins
    ANSELAbits.ANSELA=0;    // set all analog pins to digital I/O
    ADCON0bits.ADON=0;      // turn ADC off
    DACCON0bits.DACEN=0;    // turn DAC off

    // PORT A Assignments
    TRISAbits.TRISA0 = 0;	// RA0 = nc
    TRISAbits.TRISA1 = 0;	// RA1 = nc
    TRISAbits.TRISA2 = 0;	// RA2 = PWM Output (CCP1) connected to LED
    TRISAbits.TRISA3 = 0;	// RA3 = nc (MCLR)
    TRISAbits.TRISA4 = 0;	// RA4 = nc
    TRISAbits.TRISA5 = 0;	// RA5 = nc
    
    APFCONbits.CCP1SEL=0;       // The CCP1SEL bit selects which pin the PWM output is on.
                                // The default value for this bit is 0 which sets the
                                // PWM output on RA2.  If you want the PWM output on
                                // RA5 instead then set this bit to a 1.

    
    pic_ccp_pwm_init(pin_RA2);
    pic_ccp_set_period(255);
    pic_ccp_set_compare(255);
    pic_ccp_pwm_enable_clock();

    //***********************************************************************************************************
    // Now lets look at adjusting the brightness of the LED by changing the PWM duty cycle.
    // The PWM duty cycle is a 10bit value.  The lower 8 bits are set using the CCPR1L register
    // and the 2 msb bits are set using the DC1B<1:0> bits in the CCP1CONregisters.
    //***********************************************************************************************************
    // duty cycle register value = (PWM Period)*(fraction of full pulse desired)/[(TMR2 Prescaler)*(1/Fosc)]
    //***********************************************************************************************************
    // Here are example duty duty cycles and corresponding duty cycle register values
    // based on Fosc=16Mhz, TMR2 Prescaler=divide by 1 and PWM Period set to 64us (15.63 khz)
    //*********************************************************************************
    // for 100% duty: Duty Cycle Value = (64us)(1.0)/[1*(1/16000000)] =  1024
    // for 90% duty: Duty Cycle Value  = (64us)(0.9)/[1*(1/16000000)] =  921.6
    // for 50% duty: Duty Cycle Value  = (64us)(0.5)/[1*(1/16000000)] =  512
    // for 10% duty: Duty Cycle Value  = (64us)(0.1)/[1*(1/16000000]) =  102.4
    // for 1% duty: Duty Cycle Value   = (64us)(0.01)/[1*(1/16000000]) = 10.24


    // in this example, we will ramp the PWM duty cycle from 0% duty (duty cycle register =0)
    // up to 100% duty (duty cycle register = 1024) in steps of 32 with a delay of 500ms
    // in between each step. After the ramp is up to 100%, the loop starts over again at 0%

    do{
        pic_ccp_set_compare(1000);
//       DutyCycle=0;
//       while (DutyCycle <= 1023)
//       {
//            pic_ccp_set_compare(DutyCycle);
//            DutyCycle = DutyCycle + 32;
//            __delay_ms(100);
//       }

    } while (1);


}




