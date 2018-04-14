#ifndef PIC_COMMON_H_
#define	PIC_COMMON_H_

#include <xc.h> // include processor files - each processor file is guarded.  

typedef unsigned char   uint8_t;
typedef signed char     int8_t;
typedef unsigned int    uint16_t;
typedef signed int      int16_t;

typedef enum PIC12_PINS {
    pin_RA1,
    pin_RA2,
    pin_RA3,
    pin_RA4,
    pin_RA5
} PIC12_PIN_t;


#endif	/* PIC_COMMON_H_ */

