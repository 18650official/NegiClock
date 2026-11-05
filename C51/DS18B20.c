#include "DS18B20.h"

/*==============================================================================
  1-wire driver for the DS18B20 (data line P3.7)

  The bus timing below comes from the course driver (Keil_project\寒假作业\
  10.进阶电子钟) which is known to work on this board. All delays are busy
  loops, so the caller MUST disable the timer 0 interrupt while these run,
  otherwise the 0.5ms display interrupt would stretch the bit slots.

  In this project the read is only done once every 25 seconds, and the
  display is blanked for those few milliseconds, so nothing visible happens.
==============================================================================*/

/* reset pulse + presence detection, returns 1 when the sensor answered */
unsigned char DS18B20_init(void)
{
    unsigned char i;

    DATAPIN = 0;                /* pull the bus low for ~480us..960us */
    i = 70;
    while (i--);                /* ~640us at 12MHz */

    DATAPIN = 1;                /* release; the sensor pulls it low after 15..60us */
    i = 0;
    while (DATAPIN != 0)
    {
        delay(1);
        i++;
        if (i > 5) return 0;    /* no answer within 5ms -> not connected */
    }
    return 1;
}

/* write one byte, LSB first */
void DS18B20_write(unsigned char dat)
{
    unsigned int i, j;

    for (j = 0; j < 8; j++)
    {
        DATAPIN = 0;            /* start of the time slot */
        i++;
        DATAPIN = dat & 0x01;   /* write 0: stay low, write 1: release */
        i = 6;
        while (i--);
        DATAPIN = 1;            /* release the bus */
        dat >>= 1;
    }
}

/* read one byte, LSB first */
unsigned char DS18B20_read(void)
{
    unsigned char result, resultbit;
    int i, j;

    j = 0;
    result = 0;
    for (i = 0; i < 8; i++)
    {
        DATAPIN = 0;
        j++;
        DATAPIN = 1;
        j++;
        j++;
        resultbit = DATAPIN;
        result |= (resultbit << i);
        j = 4;
        while (j--);
    }
    return result;
}

/* skip ROM + start a temperature conversion (0x44) */
void DS18B20_transfer(void)
{
    DS18B20_init();
    delay(1);
    DS18B20_write(0xcc);
    DS18B20_write(0x44);
}

/* skip ROM + read scratchpad (0xBE), must follow a conversion */
void DS18B20_acquire(void)
{
    DS18B20_init();
    delay(1);
    DS18B20_write(0xcc);
    DS18B20_write(0xbe);
}

/* convert and read in one go (blocking, kept for reference/tests) */
int DS18B20_temp(void)
{
    unsigned char TH, TL;
    int temp;

    DS18B20_transfer();
    DS18B20_acquire();
    TL = DS18B20_read();
    TH = DS18B20_read();
    temp = TH;
    temp <<= 8;
    temp |= TL;
    return temp;
}
