#include "Timer.h"

/* Timer 0: mode 1 (16 bit), interrupts every 0.5ms.
   The interrupt service routine lives in main.c (Timer0_ISR) and handles the
   display multiplexing, the buzzer square wave and the 10ms/0.5s time base.

   NOTE: TR0 must be set to 1, otherwise the timer never counts and no
   interrupt ever fires (the display would stay dark). */
void Timer0Init(void)
{
    TMOD &= 0xF0;               /* keep the Timer 1 bits (UART baud rate) */
    TMOD |= 0x01;               /* Timer 0: mode 1, 16 bit */

    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;
    TF0 = 0;

    ET0 = 1;                    /* enable Timer 0 interrupt */
    TR0 = 1;                    /* <-- start the timer */
    EA  = 1;                    /* global interrupt enable */
}

void Timer0Set(unsigned char i)
{
    TR0 = i;
}
