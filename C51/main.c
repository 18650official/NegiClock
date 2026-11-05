/*******************************************************************************
 * NegiClock - desktop clock firmware
 * Board : Puzhong HC6800-ES V2.0  (STC89C52, 12MHz)
 *
 * ---------------------------------------------------------------------------
 * Pin map (checked against the HC6800-ES V2.0 schematic)
 * ---------------------------------------------------------------------------
 *  8-digit LED display
 *      segments a..g,dp : P0.0..P0.7 via 74HC573, common cathode, HIGH = lit
 *      digit select     : 74LS138  A=P2.2 B=P2.3 C=P2.4, active LOW
 *                         Y7 = leftmost digit ... Y0 = rightmost digit
 *  DS1302   SCLK=P3.6  I/O=P3.4  CE=P3.5
 *  4 keys   left to right:  [ - ] P3.1   [ SET ] P3.0   [ + ] P3.2   [ NEXT ] P3.3
 *           pressed = LOW
 *  DS18B20  DQ = P3.7   (on board sensor DS2)
 *  Buzzer   P2.5 (HIGH = on) and P1.5 (ULN2003 input) driven with the same
 *           waveform; both LOW when silent
 *  DAC LED  P2.1 held LOW -> DAC0 = 0V -> DA indicator off
 *  LEDs     D1=P2.0 (used as RX indicator), D6=P2.5, D7=P2.6 (chime on)
 *           D1..D8 = P2.0..P2.7, LOW = lit
 *
 * ---------------------------------------------------------------------------
 * Pin conflicts
 * ---------------------------------------------------------------------------
 * 1) P2.2/P2.3/P2.4 drive the 74LS138 and LED D3/D4/D5. The display is
 *    multiplexed, so these always toggle and D3/D4/D5 cannot be turned off.
 * 2) P2.1 is the DAC input and LED D2 -> DAC must be 0V, so D2 is always lit.
 * 3) P2.5 is the buzzer and LED D6 -> the buzzer must be LOW to stay silent,
 *    so D6 is always lit (a HIGH P2.5 would heat the buzzer coil silently).
 * 4) P3.4/P3.5/P3.6 are shared by DS1302, 74HC595 and XPT2046. After every
 *    DS1302 access ds1302.c clocks 0xFF into the 595 so D9..D16 stay dark.
 * 5) P3.7 is shared by the DS18B20 and the XPT2046 DOUT. The XPT2046 only
 *    drives DOUT while its CS is low, and its CS is P3.5 (the DS1302 CE),
 *    so it is only active during DS1302 transactions - exactly when the
 *    DS18B20 is NOT being read. The firmware never does both at once.
 * 6) P3.0/P3.1 are shared by the UART and keys SET/-, keys debounce 30ms,
 *    a UART byte only holds the line low for 1ms.
 *
 * ---------------------------------------------------------------------------
 * Pages
 * ---------------------------------------------------------------------------
 *  PAGE 0 : clock  - 25s cycle: 15s time, 5s date, 5s temperature
 *  PAGE 1 : alarm 1
 *  PAGE 2 : alarm 2
 *  DOWN (P3.1) in run mode cycles the pages.
 *
 *  Alarm page layout:      A1 13-22
 *      digit 0..7   : 'A', number, blank, hh, '-', mm
 *      decimal dots : dot 0..6 = Monday..Sunday enabled, dot 7 = "once"
 *
 * ---------------------------------------------------------------------------
 * Keys in run mode
 * ---------------------------------------------------------------------------
 *  SET   : enter setting (clock or the alarm that is shown)
 *  DOWN  : next page
 *  UP    : display brightness, 3 levels
 *  NEXT  : clock page  -> hourly chime on/off (D7 shows the state)
 *          alarm page  -> cycle ring mode: workday / every day / once / off
 *
 * Keys in setting mode: UP/DOWN adjust, NEXT next field, SET save & exit.
 *
 * ---------------------------------------------------------------------------
 * Serial protocol (see Serial.c)
 * ---------------------------------------------------------------------------
 *  PC -> MCU  A5 5A LEN CMD DATA XOR
 *      0x00 ping                -> ACK
 *      0x01 set time  (7 bytes) -> ACK
 *      0x02 read alarms         -> A5 5A 07 82 <a1><a2> XOR
 *      0x03 set alarms (6 bytes)-> ACK + beep
 *  MCU -> PC  A5 5A 01 06 XOR   (ACK)
 ******************************************************************************/

#include <REGX52.H>
#include <intrins.h>
#include "ds1302.h"
#include "Serial.h"
#include "Timer.h"
#include "delay.h"
#include "DS18B20.h"

/*============================== pins ==============================*/
#define SEG_PORT P0             /* segment data port (via 74HC573) */

sbit DIG_A0 = P2^2;             /* 74LS138 A0 */
sbit DIG_A1 = P2^3;             /* 74LS138 A1 */
sbit DIG_A2 = P2^4;             /* 74LS138 A2 */

sbit DAC_PIN = P2^1;            /* DAC PWM input, keep LOW -> DA LED off */

sbit KEY_ENTER = P3^0;          /* 2nd key: enter setting / save       */
sbit KEY_MINUS = P3^1;          /* 1st key: decrease / next page       */
sbit KEY_PLUS  = P3^2;          /* 3rd key: increase / brightness      */
sbit KEY_NEXT  = P3^3;          /* 4th key: next field / chime / mode  */

sbit BUZ_A = P2^5;              /* buzzer, HIGH = sound */
sbit BUZ_B = P1^5;              /* same signal at the ULN2003 input */

sbit LED_D1 = P2^0;             /* spare LED D1, LOW = lit (RX indicator) */
sbit LED_D7 = P2^6;             /* spare LED D7, LOW = lit (chime enabled) */

/* Set to 1 if the display comes out mirrored (e.g. "43-22-41"). */
#define DIG_ORDER_REVERSE   0

/*============================ constants ============================*/
#define MODE_SHOW       0
#define MODE_SET        1

#define PAGE_CLOCK      0
#define PAGE_AL1        1
#define PAGE_AL2        2

/* alarm ring modes */
#define AL_OFF          0
#define AL_WORKDAY      1       /* Monday..Friday */
#define AL_ALL          2       /* every day */
#define AL_ONCE         3       /* next match only, then off */

#define KEY_DBNC        3       /* 3 x 10ms = 30ms debounce */
#define LONG_CNT        60      /* 0.6s -> start auto repeat */
#define REP_CNT         12      /* repeat every 0.12s */

/* Timer 0 reload: 12MHz -> 1 cycle = 1us, so 0.5ms = 500 counts */
#define T0_TICK         65036   /* 65536 - 500 = 0xFE0C */

/* note half periods from the music demo's FreqTable (65536 - half_us) */
#define NOTE_M1         64580   /* C5 523Hz */
#define NOTE_M3         64777   /* E5 659Hz */
#define NOTE_M5         64898   /* G5 784Hz */

/* common-cathode segment map: bit0=a ... bit6=g, bit7=dp */
static uchar code SEG_TAB[11] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00
};
#define SEG_DASH        0x40    /* only segment g */
#define SEG_BLANK       0x00
#define SEG_CH_A        0x77    /* 'A' */
#define SEG_CH_T        0x78    /* 't' */
#define SEG_CH_C        0x58    /* 'c' */

/*============================= types ==============================*/
typedef struct {
    uchar year;                 /* 00..99 (20xx) */
    uchar mon;                  /* 1..12 */
    uchar day;                  /* 1..31 */
    uchar week;                 /* 1..7, 1 = Monday */
    uchar hour;                 /* 0..23 */
    uchar min;                  /* 0..59 */
    uchar sec;                  /* 0..59 */
} TIME_T;

typedef struct {
    uchar hour;                 /* 0..23 */
    uchar min;                  /* 0..59 */
    uchar mode;                 /* AL_OFF / AL_WORKDAY / AL_ALL / AL_ONCE */
} ALARM_T;

TIME_T  rt;                     /* running time (also edited in set mode) */
ALARM_T al[2];                  /* the two alarms */

/*============================= globals =============================*/
uchar DispBuf[8];               /* segment codes, [0] = leftmost digit */
uchar ScanIdx;                  /* scan position */
uchar BeepLeft;                 /* remaining beep ticks (0.5ms each) */
bit   Flag10ms;                 /* 10ms tick flag */
uchar BlinkPhase;               /* 0.5s blink phase */
uchar Tick10;                   /* 10ms divider */
uint  T0Reload;                 /* current timer 0 reload value */
bit   BuzzOn;                   /* 1 = chime: toggle the buzzer every tick */
uchar LedCnt;                   /* remaining LED D1 half periods */
uchar LedTick;                  /* LED D1 divider */
uchar FrameCnt;                 /* display frame counter, for dimming */

/* UART frame parser state (only touched inside the UART interrupt and by
   handle_rx in the main loop) */
uchar RxBuf[8];                 /* payload without header/length/checksum */
uchar RxLen;                    /* payload length, 0 = nothing pending */
static uchar RxSt;              /* parser state */
static uchar RxCnt;             /* received payload bytes */
static uchar RxN;               /* expected payload bytes */
static uchar RxSum;             /* xor accumulator */

static uchar g_mode;            /* MODE_SHOW / MODE_SET */
static uchar g_page;            /* PAGE_CLOCK / PAGE_AL1 / PAGE_AL2 */
static uchar g_field;           /* clock: 0..5, alarm: 0..1 */
static uchar g_buf[8];          /* display builder */
static uchar g_tick250;         /* 250ms divider */
static uchar g_blink_div;       /* 0.5s blink divider (10ms units) */
static uchar g_dim;             /* brightness 0..2 -> 4/2/1 frames out of 4 */
static bit   g_chime;           /* hourly chime enabled */
static bit   g_sec0;            /* already handled second 0 of this minute */

/* temperature */
static int   g_temp;            /* tenths of a degree */
static bit   g_temp_ok;         /* a valid reading has been taken */
static uchar g_temp_wait;       /* >0: ticks left before reading the sensor */
static bit   g_temp_done;       /* conversion already done for this page */

static uchar KeyRaw;            /* last sampled level */
static uchar KeyCnt[4];         /* debounce counters */
static uchar KeyDown;           /* pressed bitmap */
static uchar KeyHold;           /* hold time of the repeating key, 10ms units */
static uchar KeyRep;            /* repeat divider */
static uchar RepKey;            /* key that is auto repeating: 0 = none      */

/*========================= forward decls =========================*/
static void hw_init(void);
static uchar bcd2dec(uchar b);
static uchar dec2bcd(uchar d);
static uchar bcd_ok(uchar b);
static uchar days_of(uchar y, uchar m);
static uchar calc_week(uchar y, uchar m, uchar d);
static uchar time_valid(void);
static void rtc_to_ram(void);
static void ram_to_rtc(void);
static uchar wrapu(int v, uchar lo, uchar hi);
static void key_task(void);
static void key_action(uchar idx, uchar rep);
static void render(void);
static void set_display(uchar *src);
static void beep(uchar ms);
static void tone(uint reload, uchar ms);
static void chime(void);
static void send_ack(void);
static void handle_rx(void);
static void clock_task(void);
static void temp_task(void);
static void alarm_ring(uchar idx);
static uchar akey(void);

/*============================= main ==============================*/
void main(void)
{
    hw_init();

    g_mode = MODE_SHOW;
    g_page = PAGE_CLOCK;
    g_field = 0;
    g_tick250 = 0;

    rt.year = 24; rt.mon = 1; rt.day = 1; rt.week = 1;
    rt.hour = 12; rt.min = 0; rt.sec = 0;

    rtc_to_ram();
    render();

    while (1)
    {
        if (Flag10ms)
        {
            Flag10ms = 0;
            key_task();

            /* blink time base: separate from the 0.5ms refresh tick, derived
               from the 10ms task (50 x 10ms = 0.5s) */
            g_blink_div++;
            if (g_blink_div >= 50)
            {
                g_blink_div = 0;
                BlinkPhase  = (uchar)(!BlinkPhase);
            }

            clock_task();
            temp_task();
            render();
            LED_D7 = (uchar)(!g_chime);     /* D7 on = hourly chime enabled */
        }
        handle_rx();
    }
}

/*========================== initialization =======================*/
static void hw_init(void)
{
    /* 1. safe port states first */
    P2 = 0xFF;                  /* all LEDs off ... */
    P0 = 0x00;                  /* segments off */
    P1 = 0xFF;

    BUZ_A = 0;                  /* ... except the buzzer, keep it silent */
    BUZ_B = 0;
    DAC_PIN = 0;                /* DAC = 0V -> DA LED off */

    T0Reload  = T0_TICK;
    BuzzOn    = 0;
    LedCnt    = 0;
    LedTick   = 0;
    BeepLeft  = 0;
    FrameCnt  = 0;
    g_dim     = 0;              /* full brightness */
    g_chime   = 0;
    g_sec0    = 0;
    g_temp_ok = 0;
    g_temp_wait = 0;
    g_temp_done = 0;

    al[0].hour = 7;  al[0].min = 30; al[0].mode = AL_OFF;
    al[1].hour = 22; al[1].min = 0;  al[1].mode = AL_OFF;

    /* 2. timer: 0.5ms tick, drives the display scan and the buzzer */
    Timer0Init();

    /* 3. serial 9600 */
    SerialBegin();

    /* 4. RTC: if the chip holds impossible data (no backup battery), load a
          default time and make sure the oscillator is running */
    Ds1302ReadTime();
    if (!time_valid())
    {
        TIME[0] = dec2bcd(0);
        TIME[1] = dec2bcd(0);
        TIME[2] = dec2bcd(12);
        TIME[3] = dec2bcd(1);
        TIME[4] = dec2bcd(1);
        TIME[5] = dec2bcd(calc_week(24, 1, 1));
        TIME[6] = dec2bcd(24);
        Ds1302WriteTime();
        Ds1302ReadTime();
    }
    else
    {
        rtc_to_ram();
        ram_to_rtc();           /* rewrite once, this clears the CH bit */
    }

    EA = 1;

    /* 5. initialization finished: play the three tone chime (do-mi-sol) */
    chime();

    Tick10     = 0;
    BlinkPhase = 0;
    Flag10ms   = 0;
}

/*======================== BCD / date helpers =====================*/
static uchar bcd2dec(uchar b)
{
    return (uchar)(((b >> 4) * 10) + (b & 0x0F));
}

static uchar dec2bcd(uchar d)
{
    return (uchar)(((d / 10) << 4) | (d % 10));
}

static uchar bcd_ok(uchar b)
{
    if ((b & 0x0F) > 0x09) return 0;
    if ((b >> 4) > 0x09)   return 0;
    return 1;
}

/* days in a month, years 2000..2099 */
static uchar days_of(uchar y, uchar m)
{
    if (m == 2)
        return ((y & 0x03) == 0) ? 29 : 28;   /* 2000..2099: y%4==0 is leap */
    if (m == 4 || m == 6 || m == 9 || m == 11) return 30;
    return 31;
}

/* Weekday of a date: 1 = Monday ... 7 = Sunday (Sakamoto's method).
 *   y -= (m < 3);
 *   w  = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;   // 0 = Sunday
 * Note: y/400 is ADDED. */
static uchar calc_week(uchar y, uchar m, uchar d)
{
    static uchar code t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint yy, a;
    uchar w;

    if (m < 1 || m > 12) return 1;

    yy = (uint)2000 + y;
    if (m < 3) yy--;                    /* 2000+y-1, never underflows */

    a = yy + (yy / 4) - (yy / 100) + (yy / 400) + t[m - 1] + d;
    w = (uchar)(a % 7);                 /* 0 = Sunday */
    return (w == 0) ? 7 : w;
}

/*=========================== DS1302 ==============================*/
static uchar time_valid(void)
{
    if (TIME[0] & 0x80) return 0;               /* CH = 1, oscillator halted */
    if (!bcd_ok(TIME[0]) || !bcd_ok(TIME[1]) || !bcd_ok(TIME[2])) return 0;
    if (!bcd_ok(TIME[3]) || !bcd_ok(TIME[4]) || !bcd_ok(TIME[6])) return 0;
    if (bcd2dec(TIME[0]) > 59) return 0;
    if (bcd2dec(TIME[1]) > 59) return 0;
    if (bcd2dec(TIME[2]) > 23) return 0;
    if (bcd2dec(TIME[4]) < 1 || bcd2dec(TIME[4]) > 12) return 0;
    if (bcd2dec(TIME[3]) < 1 || bcd2dec(TIME[3]) > 31) return 0;
    return 1;
}

/* chip -> RAM */
static void rtc_to_ram(void)
{
    Ds1302ReadTime();
    if (!time_valid()) return;      /* keep the previous value if the read is bad */

    rt.sec  = bcd2dec(TIME[0]);
    rt.min  = bcd2dec(TIME[1]);
    rt.hour = bcd2dec(TIME[2]);
    rt.day  = bcd2dec(TIME[3]);
    rt.mon  = bcd2dec(TIME[4]);
    rt.year = bcd2dec(TIME[6]);

    if (rt.mon < 1)  rt.mon = 1;
    if (rt.mon > 12) rt.mon = 12;
    if (rt.day < 1)  rt.day = 1;
    if (rt.day > days_of(rt.year, rt.mon))
        rt.day = days_of(rt.year, rt.mon);
    rt.week = calc_week(rt.year, rt.mon, rt.day);
}

/* RAM -> chip */
static void ram_to_rtc(void)
{
    TIME[0] = dec2bcd(rt.sec);
    TIME[1] = dec2bcd(rt.min);
    TIME[2] = dec2bcd(rt.hour);
    TIME[3] = dec2bcd(rt.day);
    TIME[4] = dec2bcd(rt.mon);
    TIME[5] = dec2bcd(calc_week(rt.year, rt.mon, rt.day));
    TIME[6] = dec2bcd(rt.year);
    Ds1302WriteTime();
}

/*============================= keys ==============================*/
static uchar wrapu(int v, uchar lo, uchar hi)
{
    if (v > (int)hi) return lo;     /* above max -> wrap to min */
    if (v < (int)lo) return hi;     /* below min -> wrap to max */
    return (uchar)v;
}

/* clock setting: 0=hour 1=minute 2=second 3=day 4=month 5=year */
static void adjust_clock(uchar f, signed char d)
{
    uchar maxd;

    switch (f)
    {
    case 0:
        rt.hour = wrapu((int)rt.hour + d, 0, 23);
        break;
    case 1:
        rt.min = wrapu((int)rt.min + d, 0, 59);
        break;
    case 2:
        rt.sec = wrapu((int)rt.sec + d, 0, 59);
        break;
    case 3:
        maxd = days_of(rt.year, rt.mon);
        rt.day = wrapu((int)rt.day + d, 1, maxd);
        break;
    case 4:
        rt.mon = wrapu((int)rt.mon + d, 1, 12);
        maxd = days_of(rt.year, rt.mon);
        if (rt.day > maxd) rt.day = maxd;
        break;
    case 5:
        rt.year = wrapu((int)rt.year + d, 0, 99);
        maxd = days_of(rt.year, rt.mon);
        if (rt.day > maxd) rt.day = maxd;
        break;
    default:
        break;
    }
    rt.week = calc_week(rt.year, rt.mon, rt.day);
}

/* alarm setting: 0 = hour, 1 = minute */
static void adjust_alarm(uchar f, signed char d)
{
    uchar k = (uchar)(g_page - 1);

    if (f == 0)
        al[k].hour = wrapu((int)al[k].hour + d, 0, 23);
    else
        al[k].min = wrapu((int)al[k].min + d, 0, 59);
}

static void alarm_next_mode(void)
{
    uchar k = (uchar)(g_page - 1);

    al[k].mode++;
    if (al[k].mode > AL_ONCE) al[k].mode = AL_OFF;
}

static void key_action(uchar idx, uchar rep)
{
    /* idx: 0 = P3.0 SET, 1 = P3.1 DOWN, 2 = P3.2 UP, 3 = P3.3 NEXT */

    if (g_mode == MODE_SET)
    {
        if (idx == 0)                           /* SET: save and leave */
        {
            if (rep) return;
            if (g_page == PAGE_CLOCK) ram_to_rtc();
            g_mode = MODE_SHOW;
            beep(60);
            return;
        }
        if (idx == 3)                           /* NEXT: next field */
        {
            if (rep) return;
            g_field++;
            if (g_page == PAGE_CLOCK)
            {
                if (g_field > 5) g_field = 0;
            }
            else
            {
                if (g_field > 1) g_field = 0;
            }
            beep(30);
            return;
        }
        /* UP / DOWN adjust */
        if (g_page == PAGE_CLOCK) adjust_clock(g_field, (idx == 2) ? 1 : -1);
        else                      adjust_alarm(g_field, (idx == 2) ? 1 : -1);
        if (!rep) beep(30);
        return;
    }

    /*---------------- run mode ----------------*/
    if (idx == 1)                               /* DOWN: next page */
    {
        if (rep) return;
        g_page++;
        if (g_page > PAGE_AL2) g_page = PAGE_CLOCK;
        g_mode = MODE_SHOW;
        g_field = 0;
        beep(30);
        return;
    }

    if (idx == 0)                               /* SET: enter setting */
    {
        if (rep) return;
        g_field = 0;
        g_mode  = MODE_SET;
        beep(60);
        return;
    }

    if (idx == 2)                               /* UP: brightness */
    {
        if (rep) return;
        g_dim++;
        if (g_dim > 2) g_dim = 0;
        beep(30);
        return;
    }

    /* idx == 3 : NEXT */
    if (rep) return;
    if (g_page == PAGE_CLOCK)                   /* chime on/off */
    {
        g_chime = (uchar)(!g_chime);
    }
    else                                        /* alarm ring mode */
    {
        alarm_next_mode();
    }
    beep(30);
}

/* called every 10ms */
static void key_task(void)
{
    uchar i, lv, raw;

    raw = (uchar)(P3 & 0x0F);                   /* bit0..3 = SET,-,+,NEXT */

    for (i = 0; i < 4; i++)
    {
        lv = (uchar)((raw >> i) & 0x01);        /* 1 = released, 0 = pressed */

        if (lv != ((KeyRaw >> i) & 0x01))
        {
            KeyRaw = (uchar)((KeyRaw & (uchar)~(1 << i)) | (uchar)(lv << i));
            KeyCnt[i] = 0;
            continue;
        }

        if (KeyCnt[i] < KEY_DBNC)
        {
            KeyCnt[i]++;
            if (KeyCnt[i] == KEY_DBNC)
            {
                if (lv == 0)                    /* stable press */
                {
                    KeyDown |= (uchar)(1 << i);
                    if ((i == 1) || (i == 2))   /* UP/DOWN can auto repeat */
                    {
                        RepKey  = (uchar)(i + 1);
                        KeyHold = 0;
                        KeyRep  = 0;
                    }
                    key_action(i, 0);
                }
                else                            /* stable release */
                {
                    KeyDown &= (uchar)~(1 << i);
                    if (RepKey == (uchar)(i + 1)) RepKey = 0;
                }
            }
            continue;
        }
    }

    /* auto repeat for the key that is still held down */
    if (RepKey)
    {
        uchar k = (uchar)(RepKey - 1);

        if (KeyDown & (1 << k))
        {
            if (KeyHold < 200) KeyHold++;

            if (KeyHold == LONG_CNT)
            {
                KeyRep = 0;
            }
            else if (KeyHold > LONG_CNT)
            {
                KeyRep++;
                if (KeyRep >= REP_CNT)
                {
                    KeyRep = 0;
                    key_action(k, 1);
                }
            }
        }
        else
        {
            RepKey = 0;
        }
    }
}

/*=========================== display =============================*/
/* short beep for key feedback: 1kHz on the normal 0.5ms tick */
static void beep(uchar ms)
{
    if (ms > 120) ms = 120;
    BeepLeft = (uchar)(ms * 2);                 /* 0.5ms per tick */
}

/* Start-up chime, played once after initialization.
 * While a tone is playing the timer 0 reload is reprogrammed to the note's
 * half period, so the interrupt toggles the buzzer at the audio rate. The
 * display scan keeps running in the same interrupt (one digit per tick,
 * still ~130Hz at the lowest note), so nothing flickers. */
static void tone(uint reload, uchar ms)
{
    T0Reload = reload;
    BuzzOn   = 1;
    delay(ms);

    BuzzOn   = 0;
    BUZ_A = 0;
    BUZ_B = 0;
    T0Reload = T0_TICK;                         /* back to 0.5ms */
    delay(25);                                  /* small gap between notes */
}

/* do - mi - sol  (C5 - E5 - G5) */
static void chime(void)
{
    tone(NOTE_M1, 120);
    tone(NOTE_M3, 120);
    tone(NOTE_M5, 200);
}

/* copy 8 segment codes into the scan buffer (timer interrupt off) */
static void set_display(uchar *src)
{
    uchar i;
    ET0 = 0;
    for (i = 0; i < 8; i++) DispBuf[i] = src[i];
    ET0 = 1;
}

/*---- layout builders, they only fill g_buf[] ----*/
static void fill_time(uchar h, uchar m, uchar s)
{
    g_buf[0] = SEG_TAB[h / 10];
    g_buf[1] = SEG_TAB[h % 10];
    g_buf[2] = SEG_DASH;
    g_buf[3] = SEG_TAB[m / 10];
    g_buf[4] = SEG_TAB[m % 10];
    g_buf[5] = SEG_DASH;
    g_buf[6] = SEG_TAB[s / 10];
    g_buf[7] = SEG_TAB[s % 10];
}

static void fill_date(uchar mo, uchar d)
{
    uchar w = calc_week(rt.year, mo, d);

    g_buf[0] = SEG_TAB[mo / 10];
    g_buf[1] = SEG_TAB[mo % 10];
    g_buf[2] = SEG_DASH;
    g_buf[3] = SEG_TAB[d / 10];
    g_buf[4] = SEG_TAB[d % 10];
    g_buf[7] = SEG_TAB[w];                      /* weekday 1..7 */
}

/*   t  -23.1 c   ->  [0]'t'  [1..5] number right aligned  [6]'c'   */
static void fill_temp(void)
{
    uint u;
    int  v;
    uchar neg;

    g_buf[0] = SEG_CH_T;
    g_buf[6] = SEG_CH_C;
    if (!g_temp_ok) return;                     /* nothing measured yet */

    v = g_temp;
    neg = 0;
    if (v < 0)
    {
        neg = 1;
        v = -v;
    }
    if (v > 1250) v = 1250;                     /* DS18B20 range */

    u = (uint)v;
    g_buf[5] = SEG_TAB[u % 10]; u /= 10;        /* tenths  */
    g_buf[4] = (uchar)(SEG_TAB[u % 10] | 0x80); /* ones + dp */
    u /= 10;
    if (u > 0) { g_buf[3] = SEG_TAB[u % 10]; u /= 10; }  /* tens     */
    if (u > 0) { g_buf[2] = SEG_TAB[u % 10]; }           /* hundreds */

    if (neg)
    {
        if (g_buf[3] == SEG_BLANK) g_buf[3] = SEG_DASH;
        else                       g_buf[2] = SEG_DASH;
    }
}

/*   A1 13-22   */
static void fill_alarm(uchar k)
{
    g_buf[0] = SEG_CH_A;
    g_buf[1] = SEG_TAB[k + 1];
    g_buf[3] = SEG_TAB[al[k].hour / 10];
    g_buf[4] = SEG_TAB[al[k].hour % 10];
    g_buf[5] = SEG_DASH;
    g_buf[6] = SEG_TAB[al[k].min / 10];
    g_buf[7] = SEG_TAB[al[k].min % 10];

    /* decimal dots: 0..6 = Monday..Sunday, 7 = once */
    switch (al[k].mode)
    {
    case AL_WORKDAY:
        g_buf[0] |= 0x80; g_buf[1] |= 0x80; g_buf[2] |= 0x80;
        g_buf[3] |= 0x80; g_buf[4] |= 0x80;
        break;
    case AL_ALL:
        g_buf[0] |= 0x80; g_buf[1] |= 0x80; g_buf[2] |= 0x80;
        g_buf[3] |= 0x80; g_buf[4] |= 0x80; g_buf[5] |= 0x80;
        g_buf[6] |= 0x80;
        break;
    case AL_ONCE:
        g_buf[7] |= 0x80;
        break;
    default:
        break;
    }
}

static void render(void)
{
    uchar i, ph, k, time_layout;

    for (i = 0; i < 8; i++) g_buf[i] = SEG_BLANK;
    time_layout = 0;
    ph = 0;

    if (g_page != PAGE_CLOCK)
    {
        k = (uchar)(g_page - 1);
        fill_alarm(k);
    }
    else if (g_mode == MODE_SET)
    {
        if (g_field < 3)
        {
            fill_time(rt.hour, rt.min, rt.sec);
            time_layout = 1;
        }
        else
        {
            fill_date(rt.mon, rt.day);
            if (g_field == 5)                   /* editing the year */
            {
                g_buf[6] = SEG_TAB[rt.year / 10];
                g_buf[7] = SEG_TAB[rt.year % 10];
            }
        }
    }
    else
    {
        ph = (uchar)(rt.sec % 25);
        if (ph < 15)
        {
            fill_time(rt.hour, rt.min, rt.sec);
            time_layout = 1;
        }
        else if (ph < 20)
        {
            fill_date(rt.mon, rt.day);
        }
        else
        {
            fill_temp();
        }
    }

    /* blink the field being edited, 0.5s on / 0.5s off */
    if ((g_mode == MODE_SET) && (BlinkPhase == 0))
    {
        if (g_page != PAGE_CLOCK)
        {
            if (g_field == 0) { g_buf[3] = SEG_BLANK; g_buf[4] = SEG_BLANK; }
            else              { g_buf[6] = SEG_BLANK; g_buf[7] = SEG_BLANK; }
        }
        else
        {
            switch (g_field)
            {
            case 0: g_buf[0] = SEG_BLANK; g_buf[1] = SEG_BLANK; break;
            case 1: g_buf[3] = SEG_BLANK; g_buf[4] = SEG_BLANK; break;
            case 2: g_buf[6] = SEG_BLANK; g_buf[7] = SEG_BLANK; break;
            case 3: g_buf[3] = SEG_BLANK; g_buf[4] = SEG_BLANK; break;
            case 4: g_buf[0] = SEG_BLANK; g_buf[1] = SEG_BLANK; break;
            case 5: g_buf[6] = SEG_BLANK; g_buf[7] = SEG_BLANK; break;
            default: break;
            }
        }
    }

    /* dp of the last digit: 1Hz heartbeat on the time layout only */
    if (time_layout)
    {
        if (rt.sec & 0x01) g_buf[7] |= 0x80;
        else               g_buf[7] &= 0x7F;
    }

    /* only touch the scan buffer when something changed */
    for (i = 0; i < 8; i++)
    {
        if (g_buf[i] != DispBuf[i])
        {
            set_display(g_buf);
            break;
        }
    }
}

/*========================== clock & alarms =======================*/
/* hourly chime + alarm check, called once per minute at second 0 */
static void on_minute(void)
{
    uchar k, hit;

    /* look for an alarm first */
    hit = 0xFF;
    for (k = 0; k < 2; k++)
    {
        if (al[k].mode == AL_OFF) continue;
        if (al[k].hour != rt.hour || al[k].min != rt.min) continue;

        if (al[k].mode == AL_WORKDAY)
        {
            if (rt.week > 5) continue;
        }
        hit = k;
        break;
    }

    if (hit != 0xFF)
    {
        alarm_ring(hit);
        return;
    }

    /* hourly chime, 08:00 .. 23:00 */
    if (g_chime && rt.min == 0 && rt.hour >= 8 && rt.hour <= 23) beep(120);
}

static void clock_task(void)
{
    if (g_mode != MODE_SHOW) return;            /* frozen while setting */

    /* While the DS18B20 converts, CE (P3.5) is held HIGH to keep the XPT2046
       deselected - it shares P3.7 with the sensor. So no DS1302 access now. */
    if (g_temp_wait) return;

    g_tick250++;
    if (g_tick250 < 25) return;                 /* 25 x 10ms = 250ms */
    g_tick250 = 0;

    rtc_to_ram();

    if (rt.sec == 0)
    {
        if (!g_sec0)
        {
            g_sec0 = 1;
            on_minute();
        }
    }
    else
    {
        g_sec0 = 0;
    }
}

/*--------------------------- temperature -------------------------*/
static void temp_task(void)
{
    uchar ph;

    if (g_mode != MODE_SHOW || g_page != PAGE_CLOCK)
    {
        g_temp_done = 0;                /* re-measure next time we come back */
        return;
    }

    ph = (uchar)(rt.sec % 25);

    if (ph >= 20)                               /* temperature page */
    {
        if (!g_temp_done && g_temp_wait == 0)
        {
            /* Start a conversion.
               P3.5 is the DS1302 CE and at the same time the XPT2046 CS;
               DS1302 transactions leave CE low, which SELECTS the XPT2046 and
               lets it drive P3.7 (its DOUT) - that would fight the DS18B20.
               Driving CE high here deselects it for the whole conversion, and
               clock_task() keeps the DS1302 away until we have read the
               result. The bus is timing critical, so the display interrupt is
               stopped and the display blanked during the bit banging. */
            ET0 = 0;
            SEG_PORT = 0x00;
            RST = 1;                            /* deselect the XPT2046 */
            DS18B20_transfer();
            SEG_PORT = 0x00;
            ET0 = 1;
            g_temp_wait = 80;                   /* 800ms > 750ms conversion */
        }
    }
    else
    {
        g_temp_done = 0;
    }

    if (g_temp_wait)
    {
        g_temp_wait--;
        if (g_temp_wait == 0)                   /* conversion finished */
        {
            int raw;

            ET0 = 0;
            SEG_PORT = 0x00;
            RST = 1;                            /* still deselected */
            DS18B20_acquire();
            raw  = DS18B20_read();              /* TL */
            raw |= ((int)DS18B20_read() << 8);  /* TH */
            ET0 = 1;

            if (raw >= -880 && raw <= 2000)
            {
                if (raw < 0) g_temp = -(int)((((-raw) * 10) + 8) / 16);
                else         g_temp =  (int)((( raw * 10) + 8) / 16);
                g_temp_ok = 1;
            }
            else
            {
                g_temp_ok = 0;                  /* disconnected / out of range */
            }
            g_temp_done = 1;
        }
    }
}

/*----------------------------- alarms ----------------------------*/
/* any key pressed? used by the ringing loop only */
static uchar akey(void)
{
    if ((P3 & 0x0F) == 0x0F) return 0;
    delay(15);
    if ((P3 & 0x0F) == 0x0F) return 0;
    return 1;
}

static void alarm_ring(uchar idx)
{
    uchar i, n, stop, cyc;

    stop = 0;
    cyc  = 0;
    while (!stop && cyc < 60)                    /* give up after ~70s */
    {
        /* "beep beep beep beep" */
        for (i = 0; i < 4 && !stop; i++)
        {
            beep(70);
            for (n = 0; n < 11 && !stop; n++)   /* 110ms */
            {
                delay(10);
                if (akey()) stop = 1;
            }
        }
        /* pause "-" */
        for (n = 0; n < 50 && !stop; n++)       /* 500ms */
        {
            delay(10);
            if (akey()) stop = 1;
        }
        cyc++;
    }

    /* the key press that stopped the alarm must not do anything else */
    for (n = 0; n < 200 && ((P3 & 0x0F) != 0x0F); n++) delay(10);

    BeepLeft = 0;
    BUZ_A = 0;
    BUZ_B = 0;

    KeyRaw   = (uchar)(P3 & 0x0F);
    KeyDown  = 0;
    RepKey   = 0;
    KeyCnt[0] = KEY_DBNC; KeyCnt[1] = KEY_DBNC;
    KeyCnt[2] = KEY_DBNC; KeyCnt[3] = KEY_DBNC;

    if (al[idx].mode == AL_ONCE) al[idx].mode = AL_OFF;
}

/*========================= serial sync ===========================*/
static void send_ack(void)
{
    uchar f[5];
    f[0] = 0xA5;
    f[1] = 0x5A;
    f[2] = 0x01;
    f[3] = 0x06;                    /* 0x06 = ACK */
    f[4] = (uchar)(f[0] ^ f[1] ^ f[2] ^ f[3]);
    SerialSendFrame(f, 5);
}

static void send_alarms(void)
{
    uchar f[11];
    uchar i;
    uchar x;

    f[0] = 0xA5;
    f[1] = 0x5A;
    f[2] = 0x07;                    /* length: command + 6 data bytes */
    f[3] = 0x82;                    /* 0x82 = alarm data */
    f[4] = al[0].hour;
    f[5] = al[0].min;
    f[6] = al[0].mode;
    f[7] = al[1].hour;
    f[8] = al[1].min;
    f[9] = al[1].mode;

    x = 0;
    for (i = 0; i < 10; i++) x ^= f[i];
    f[10] = x;

    SerialSendFrame(f, 11);
}

static void handle_rx(void)
{
    uchar cmd, y, mo, d, hh, mi, ss, maxd;
    uchar *p;

    if (RxLen == 0) return;

    p   = RxBuf;
    cmd = p[0];

    if (cmd == 0x00)                            /* ping: reply only */
    {
        RxLen = 0;
        send_ack();
        return;
    }

    if (cmd == 0x01 && RxLen >= 8)              /* set time: cmd YY MM DD W HH MI SS */
    {
        y  = p[1]; mo = p[2]; d = p[3];
        hh = p[5]; mi = p[6]; ss = p[7];

        if (mo >= 1 && mo <= 12 && y <= 99 && hh <= 23 && mi <= 59 && ss <= 59)
        {
            maxd = days_of(y, mo);
            if (d >= 1 && d <= maxd)
            {
                rt.year = y; rt.mon = mo; rt.day = d;
                rt.hour = hh; rt.min = mi; rt.sec = ss;
                rt.week = calc_week(y, mo, d);

                ram_to_rtc();                   /* write the DS1302 */
                RxLen = 0;
                send_ack();
                return;
            }
        }
        RxLen = 0;                              /* bad data: no ACK */
        return;
    }

    if (cmd == 0x02)                            /* read the two alarms */
    {
        RxLen = 0;
        send_alarms();
        return;
    }

    if (cmd == 0x03 && RxLen >= 7)              /* set alarms: 2 x hour/min/mode */
    {
        al[0].hour = p[1];
        al[0].min  = p[2];
        al[0].mode = p[3];
        al[1].hour = p[4];
        al[1].min  = p[5];
        al[1].mode = p[6];

        if (al[0].hour > 23) al[0].hour = 23;
        if (al[0].min  > 59) al[0].min  = 59;
        if (al[0].mode > AL_ONCE) al[0].mode = AL_OFF;
        if (al[1].hour > 23) al[1].hour = 23;
        if (al[1].min  > 59) al[1].min  = 59;
        if (al[1].mode > AL_ONCE) al[1].mode = AL_OFF;

        RxLen = 0;
        beep(60);                               /* audible confirmation */
        send_ack();
        return;
    }

    RxLen = 0;                                  /* unknown command / bad frame */
}

/*========================= interrupts ============================*/
/* Timer 0: every 0.5ms - display scan + buzzer square wave + time base.
   C51 pushes every register this function uses, so it is safe without an
   extra register bank (and 128 bytes of RAM are precious here). */
void Timer0_ISR(void) interrupt 1
{
    uchar sel;

    TH0 = (uchar)(T0Reload >> 8);
    TL0 = (uchar)T0Reload;

    /* buzzer */
    if (BuzzOn)
    {
        BUZ_A = (uchar)(!BUZ_A);
        BUZ_B = (uchar)(!BUZ_B);
    }
    else if (BeepLeft)
    {
        BeepLeft--;
        sel = (uchar)(BeepLeft & 0x01);
        BUZ_A = sel;
        BUZ_B = sel;
        if (BeepLeft == 0)
        {
            BUZ_A = 0;
            BUZ_B = 0;
        }
    }
    else
    {
        BUZ_A = 0;
        BUZ_B = 0;
    }

    /* LED D1: 3 quick flashes after a serial frame arrived */
    if (LedCnt)
    {
        if (LedTick)
        {
            LedTick--;
        }
        else
        {
            LedTick = 30;                       /* 30 x 0.5ms = 15ms */
            LED_D1  = (uchar)(!LED_D1);
            LedCnt--;
            if (LedCnt == 0) LED_D1 = 1;        /* finish with the LED off */
        }
    }

    /* display multiplexing */
    SEG_PORT = 0x00;                            /* blank first, no ghosting */
    ScanIdx++;
    if (ScanIdx >= 8)
    {
        ScanIdx  = 0;
        FrameCnt = (uchar)((FrameCnt + 1) & 0x03);
    }
#if DIG_ORDER_REVERSE
    sel = ScanIdx;
#else
    sel = (uchar)(7 - ScanIdx);                 /* index 0 -> 138 output Y7 */
#endif
    DIG_A0 = sel & 0x01;
    DIG_A1 = (sel >> 1) & 0x01;
    DIG_A2 = (sel >> 2) & 0x01;

    /* brightness: only show the current frame when FrameCnt is below the
       on-count (4 = full, 2 = half, 1 = quarter) */
    if (FrameCnt < (uchar)(4 >> g_dim))
        SEG_PORT = DispBuf[ScanIdx];
    else
        SEG_PORT = 0x00;

    /* time base */
    Tick10++;
    if (Tick10 >= 20)                           /* 20 x 0.5ms = 10ms */
    {
        Tick10   = 0;
        Flag10ms = 1;
    }
}

/* UART interrupt: receives and parses the frame in place.
   It is a leaf function (no calls) and C51 saves the registers it uses. */
void Uart_ISR(void) interrupt 4
{
    uchar ch;

    if (RI)
    {
        RI = 0;
        ch = SBUF;

        if (RxLen == 0)                 /* previous frame already handled? */
        {
            switch (RxSt)
            {
            case 0:                     /* wait for the first header byte */
                if (ch == 0xA5)
                {
                    RxSum = ch;
                    RxSt  = 1;
                }
                break;

            case 1:                     /* second header byte */
                if (ch == 0x5A)
                {
                    RxSum ^= ch;
                    RxSt  = 2;
                }
                else if (ch == 0xA5)
                {
                    RxSum = ch;         /* still in sync */
                }
                else
                {
                    RxSt = 0;
                }
                break;

            case 2:                     /* length */
                if (ch >= 1 && ch <= 8)
                {
                    RxN   = ch;
                    RxCnt = 0;
                    RxSum ^= ch;
                    RxSt  = 3;
                }
                else
                {
                    RxSt = 0;
                }
                break;

            default:                    /* payload, then checksum */
                if (RxCnt < RxN)
                {
                    if (RxCnt < 8) RxBuf[RxCnt] = ch;
                    RxSum ^= ch;
                    RxCnt++;
                }
                else
                {
                    if (ch == RxSum)    /* checksum OK */
                    {
                        RxLen   = RxN;
                        LedCnt  = 6;    /* 3 quick flashes on D1 */
                        LedTick = 0;
                    }
                    RxSt  = 0;
                    RxCnt = 0;
                }
                break;
            }
        }
    }

    if (TI)
    {
        /* SerialSendByte disables ES while sending, so normally we never
           get here with TI set */
        TI = 0;
    }
}
