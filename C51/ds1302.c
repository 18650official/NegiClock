#include "ds1302.h"

/*--- register addresses, LSB = read/write bit ---*/
/*--- order: sec min hour date month week year ---*/
uchar code READ_RTC_ADDR[7]  = {0x81, 0x83, 0x85, 0x87, 0x89, 0x8b, 0x8d};
uchar code WRITE_RTC_ADDR[7] = {0x80, 0x82, 0x84, 0x86, 0x88, 0x8a, 0x8c};

/*--- time buffer, order: sec min hour date month week year, BCD ---*/
uchar TIME[7] = {0x00, 0x00, 0x12, 0x01, 0x01, 0x02, 0x24};

/*******************************************************************************
* Name   : Hc595AllOff
* Brief  : On this board the 74HC595 shares P3.4/P3.5/P3.6 with the DS1302:
*              SER   = P3.4 (DSIO)
*              SRCLK = P3.6 (SCLK)
*              RCLK  = P3.5 (RST/CE)
*          Every DS1302 transaction shifts garbage into the 595 and latches it,
*          which lights D9..D16 and the 8x8 dot matrix.
*          Here we clock 8 ones in and latch them: all outputs high, all dark.
*******************************************************************************/
static void Hc595AllOff(void)
{
    uchar n;

    DSIO = 1;                   /* SER = 1, feed all ones */
    for (n = 0; n < 8; n++)
    {
        SCLK = 1;
        _nop_();
        SCLK = 0;
        _nop_();
    }
    RST = 1;                    /* rising edge on RCLK latches */
    _nop_();
    RST = 0;
    _nop_();
}

/*******************************************************************************
* Name   : Ds1302Write
* Brief  : write one byte (command + data) to the DS1302
*******************************************************************************/
void Ds1302Write(uchar addr, uchar dat)
{
    uchar n;

    RST = 0;
    _nop_();

    SCLK = 0;                   /* start with SCLK low */
    _nop_();
    RST = 1;                    /* then pull CE high */
    _nop_();

    for (n = 0; n < 8; n++)     /* 8 address bits, LSB first */
    {
        DSIO = addr & 0x01;
        addr >>= 1;
        SCLK = 1;               /* DS1302 samples on the rising edge */
        _nop_();
        SCLK = 0;
        _nop_();
    }
    for (n = 0; n < 8; n++)     /* 8 data bits */
    {
        DSIO = dat & 0x01;
        dat >>= 1;
        SCLK = 1;
        _nop_();
        SCLK = 0;
        _nop_();
    }

    RST = 0;
    _nop_();
}

/*******************************************************************************
* Name   : Ds1302Read
* Brief  : read one byte from the DS1302
*******************************************************************************/
uchar Ds1302Read(uchar addr)
{
    uchar n, dat, dat1;

    RST = 0;
    _nop_();

    SCLK = 0;
    _nop_();
    RST = 1;
    _nop_();

    for (n = 0; n < 8; n++)
    {
        DSIO = addr & 0x01;
        addr >>= 1;
        SCLK = 1;
        _nop_();
        SCLK = 0;
        _nop_();
    }

    DSIO = 1;                   /* release the I/O line before reading */
    _nop_();

    for (n = 0; n < 8; n++)
    {
        dat1 = DSIO;            /* LSB first */
        dat  = (uchar)((dat >> 1) | (dat1 << 7));
        SCLK = 1;
        _nop_();
        SCLK = 0;
        _nop_();
    }

    RST = 0;
    _nop_();
    SCLK = 1;                   /* required DS1302 reset settling time */
    _nop_();
    DSIO = 0;
    _nop_();
    DSIO = 1;
    _nop_();

    return dat;
}

/*******************************************************************************
* Name   : Ds1302Init
* Brief  : initialise the DS1302 with the time stored in TIME[]
*******************************************************************************/
void Ds1302Init(void)
{
    Ds1302WriteTime();
}

/*******************************************************************************
* Name   : Ds1302WriteTime
* Brief  : clear write protect -> write the 7 time registers -> set protect
*******************************************************************************/
void Ds1302WriteTime(void)
{
    uchar n;

    Ds1302Write(0x8E, 0x00);                    /* write protect off */
    for (n = 0; n < 7; n++)
    {
        Ds1302Write(WRITE_RTC_ADDR[n], TIME[n]);
    }
    Ds1302Write(0x8E, 0x80);                    /* write protect on */

    Hc595AllOff();                              /* keep the 595 LEDs dark */
}

/*******************************************************************************
* Name   : Ds1302ReadTime
* Brief  : read the 7 time registers into TIME[]
*******************************************************************************/
void Ds1302ReadTime(void)
{
    uchar n;

    for (n = 0; n < 7; n++)
    {
        TIME[n] = Ds1302Read(READ_RTC_ADDR[n]);
    }

    Hc595AllOff();                              /* keep the 595 LEDs dark */
}
