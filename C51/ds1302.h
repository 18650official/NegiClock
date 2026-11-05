#ifndef __DS1302_H_
#define __DS1302_H_

/*--- includes ---*/
#include <REGX52.H>
#include <intrins.h>

/*--- type shortcuts ---*/
#ifndef uchar
#define uchar unsigned char
#endif

#ifndef uint
#define uint unsigned int
#endif

/*--- DS1302 pins (HC6800-ES V2.0 schematic) ---*/
sbit DSIO = P3^4;               /* I/O  */
sbit RST  = P3^5;               /* CE   */
sbit SCLK = P3^6;               /* SCLK */

/*--- API ---*/
void  Ds1302Write(uchar addr, uchar dat);
uchar Ds1302Read(uchar addr);
void  Ds1302Init(void);
void  Ds1302ReadTime(void);
void  Ds1302WriteTime(void);    /* write TIME[7], handles the write protect bit */

/*--- shared buffer: sec min hour date month week year, all BCD ---*/
extern uchar TIME[7];

#endif
