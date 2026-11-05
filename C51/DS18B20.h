#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <REGX52.H>
#include <INTRINS.H>
#include "delay.h"

/* DS18B20 data line.
   On the HC6800-ES V2.0 the on-board sensor DS2 is wired to P3.7.
   Note: the XPT2046 touch chip also drives P3.7, but only while its CS
   (P3.5, the DS1302 CE line) is low, so the two are never used at the same
   time - see the comment in main.c. */
#define DATAPIN P3_7

/* Low level 1-wire helpers (timing from the course driver, proven on this
   board). Every call blocks for a few hundred microseconds, so the caller
   must disable the timer interrupt around them. */
unsigned char DS18B20_init(void);       /* reset + presence, 1 = found       */
void DS18B20_write(unsigned char dat);  /* write one byte, LSB first         */
unsigned char DS18B20_read(void);       /* read one byte, LSB first          */

void DS18B20_transfer(void);            /* skip ROM + start conversion (0x44) */
void DS18B20_acquire(void);             /* skip ROM + read scratchpad (0xBE)  */

/* Raw temperature in 1/16 degC (signed). Blocks ~750ms because it converts
   and reads in one go - main.c uses the split transfer/acquire pair
   instead, so the display keeps running while the sensor converts. */
int DS18B20_temp(void);

#endif
