#ifndef __TIMER0_H__
#define __TIMER0_H__

#include <REGX52.H>

/* Timer 0, mode 1 (16 bit), 0.5ms tick at 12MHz.
   12MHz -> 1 machine cycle = 1us, so 0.5ms = 500 counts:
       65536 - 500 = 65036 = 0xFE0C                                */
#define TIMER0_RELOAD_H     0xFE
#define TIMER0_RELOAD_L     0x0C

void Timer0Init(void);
void Timer0Set(unsigned char i);

#endif
