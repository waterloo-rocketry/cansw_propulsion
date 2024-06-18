#ifndef _XC_H
#define _XC_H

#include <stdint.h>

#define __interrupt()

extern uint8_t TRISA2;
extern uint8_t TRISA3;
extern uint8_t TRISA4;
extern uint8_t LATA2;
extern uint8_t LATA3;
extern uint8_t LATA4;
extern uint8_t TRISC1;
extern uint8_t RC1PPS;
extern uint8_t TRISC0;
extern uint8_t ANSELC0;
extern uint8_t CANRXPPS;
extern uint8_t PIR5;

struct INTCON0bits_t {
    uint8_t GIE;
};
extern struct INTCON0bits_t INTCON0bits;

struct PIE3bits_t {
    uint8_t TMR0IE;
};
extern struct PIE3bits_t PIE3bits;

struct PIR3bits_t {
    uint8_t TMR0IF;
};
extern struct PIR3bits_t PIR3bits;

void CLRWDT();
void RESET();

#endif
