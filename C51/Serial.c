#include "Serial.h"

/*==============================================================================
  UART, 9600 8N1  (transmit side only - the receive parser lives in the UART
  interrupt handler inside main.c)

  Note: with a 12MHz crystal and SMOD=1 + TH1=0xFA the real baud rate is
        10417bps; 9600 exact needs an 11.0592MHz crystal.
        The course code uses these values, so they are kept here.
        The PC program auto-detects: it tries 9600 first, then 10417.

  Frame from the PC to the MCU:
      A5 5A LEN CMD DATA... XOR      (XOR = xor of all previous bytes)
      CMD = 0x00  ping, MCU answers with a 5 byte ACK
      CMD = 0x01  set time, DATA = YY MM DD W HH MI SS (binary)
  Answer from the MCU to the PC:
      A5 5A 01 06 XOR
==============================================================================*/

/*------------------------------------------------------------------------------
  UART init, 9600
------------------------------------------------------------------------------*/
void SerialBegin(void)
{
    PCON |= 0x80;                   /* SMOD = 1 */
    SCON  = 0x50;                   /* 8 data bits, variable baud, REN = 1 */
    TMOD &= 0x0F;                   /* keep the Timer 0 settings */
    TMOD |= 0x20;                   /* Timer 1: mode 2, 8 bit auto reload */
    TL1 = 0xFA;
    TH1 = 0xFA;
    ET1 = 0;
    TR1 = 1;

    PS  = 1;                        /* UART interrupt has high priority */
    ES  = 1;
}

/*------------------------------------------------------------------------------
  Send one byte.
  ES is disabled while sending, otherwise the UART ISR would clear TI and the
  while(!TI) loop below would hang forever.
------------------------------------------------------------------------------*/
void SerialSendByte(uchar ch)
{
    ES = 0;
    TI = 0;
    SBUF = ch;
    while (!TI);
    TI = 0;
    ES = 1;
}

void SerialSendString(char *ch)
{
    while (*ch != '\0')
    {
        SerialSendByte((uchar)*ch);
        ch++;
    }
}

void SerialSendFrame(uchar *buf, uchar len)
{
    uchar i;
    for (i = 0; i < len; i++)
    {
        SerialSendByte(buf[i]);
    }
}
