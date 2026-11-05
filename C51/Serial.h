#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <REGX52.H>

#ifndef uchar
#define uchar unsigned char
#endif

/*--- init UART at 9600 ---*/
void SerialBegin(void);

/*--- transmit ---*/
void SerialSendByte(uchar ch);
void SerialSendString(char *ch);
void SerialSendFrame(uchar *buf, uchar len);

#endif
