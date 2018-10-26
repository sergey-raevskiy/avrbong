/* Name: main.c
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-06-21
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/*
General Description:
This module implements hardware initialization and the USB interface
*/

#include "hardware.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

#include "utils.h"
#include "oddebug.h"
#include "stk500protocol.h"
#include "vreg.h"
#include "timer.h"

/*
20 pin HVSP / PP connector:
    1 .... GND
    2 .... Vtarget
    3 .... HVSP SCI
    4 .... RESET
    16 ... HVSP SDO
    18 ... HVSP SII
    20 ... HVSP SDI

 * Timer usage:
 * Timer 0 [8 bit]:
 *   1/64 prescaler for timer interrupt
 * Timer 1 [16 bit]:
 *   PWM for voltage supply -> fastPWM mode
 *   f = 23.4 kHz -> prescaler = 1, 9 bit
 * Timer 2 [8 bit]:
 *   Clock generation for target device
 */

static void hardwareInit(void)
{
uchar   portB = 0, portC = 0, portD = 0, ddrB = 0, ddrC = 0, ddrD = 0;

#if ENABLE_HVPROG
    UTIL_PBIT_SET(port, HWPIN_HVSP_SUPPLY);
    UTIL_PBIT_SET(ddr, HWPIN_HVSP_SUPPLY);
    UTIL_PBIT_CLR(port, HWPIN_SMPS_OUT);
    UTIL_PBIT_SET(ddr, HWPIN_SMPS_OUT);
    UTIL_PBIT_CLR(port, HWPIN_ADC_SMPS);
    UTIL_PBIT_CLR(ddr, HWPIN_ADC_SMPS);
    UTIL_PBIT_CLR(port, HWPIN_HVSP_HVRESET);
    UTIL_PBIT_SET(ddr, HWPIN_HVSP_HVRESET);
    UTIL_PBIT_CLR(port, HWPIN_HVSP_SCI);
    UTIL_PBIT_SET(ddr, HWPIN_HVSP_SCI);
    UTIL_PBIT_CLR(port, HWPIN_HVSP_SII);
    UTIL_PBIT_SET(ddr, HWPIN_HVSP_SII);
    UTIL_PBIT_CLR(port, HWPIN_HVSP_SDI);
    UTIL_PBIT_SET(ddr, HWPIN_HVSP_SDI);
    UTIL_PBIT_CLR(port, HWPIN_HVSP_SDO);
    UTIL_PBIT_CLR(ddr, HWPIN_HVSP_SDO);
#endif /* ENABLE_HVPROG */

#ifdef HWPIN_ISP_CLK
    UTIL_PBIT_CLR(port, HWPIN_ISP_CLK);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_CLK);
#endif

#ifdef HWPIN_GREEN_LED
    UTIL_PBIT_SET(ddr, HWPIN_GREEN_LED);    /* green LED on USBASP */
    UTIL_PBIT_SET(port, HWPIN_GREEN_LED);   /* turn green LED on USBASP off, turned on in the usbFunctionSetup */
#endif

#if METABOARD_HARDWARE
    UTIL_PBIT_SET(ddr, HWPIN_LED);
    UTIL_PBIT_SET(port, HWPIN_LED);
    /* keep all I/O pins and DDR bits for ISP on low level */
    UTIL_PBIT_CLR(port, HWPIN_ISP_SUPPLY1);
    UTIL_PBIT_CLR(port, HWPIN_ISP_SUPPLY2);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_SUPPLY1);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_SUPPLY2);
#else /* METABOARD_HARDWARE */
    UTIL_PBIT_CLR(port, HWPIN_ISP_DRIVER);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_DRIVER);
    UTIL_PBIT_SET(ddr, HWPIN_LED);
    UTIL_PBIT_CLR(port, HWPIN_LED);
    UTIL_PBIT_CLR(port, HWPIN_ADC_VTARGET);
    UTIL_PBIT_CLR(ddr, HWPIN_ADC_VTARGET);
    UTIL_PBIT_CLR(port, HWPIN_ISP_SCK);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_SCK);
    UTIL_PBIT_SET(port, HWPIN_ISP_MISO);
    UTIL_PBIT_CLR(ddr, HWPIN_ISP_MISO);
    UTIL_PBIT_CLR(port, HWPIN_ISP_MOSI);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_MOSI);
    UTIL_PBIT_CLR(port, HWPIN_ISP_RESET);
    UTIL_PBIT_SET(ddr, HWPIN_ISP_RESET);
#endif /* METABOARD_HARDWARE */

    UTIL_PBIT_SET(port, HWPIN_JUMPER);
    UTIL_PBIT_CLR(ddr, HWPIN_JUMPER);

    PORTB = portB;
    DDRB = ddrB;
    PORTC = portC;
    DDRC = ddrC;
    PORTD = portD;
    DDRD = ddrD;

    /* timer 0 configuration: ~ 1.365 ms interrupt @ 12 MHz */
    TCCR0 = 3;              /* 1/64 prescaler */
    TIMSK = (1 << TOIE0);   /* enable timer0 overflow interrupt */

#if USBASP_HARDWARE
    /* Do not configure Timer 1 for original USBasp hardware */
#elif METABOARD_HARDWARE
    /* timer 1 configuration (used for target clock): */
    TCCR1A = UTIL_BIN8(1000, 0010); /* OC1A = PWM out, OC1B disconnected */
    TCCR1B = UTIL_BIN8(0001, 1001); /* wgm 14: TOP = ICR1, prescaler = 1 */
    ICR1 = F_CPU / 1000000 - 1;     /* TOP value for 1 MHz */
    OCR1A = F_CPU / 2000000 - 1;    /* 50% duty cycle */
#else /* METABOARD_HARDWARE */
    /* timer 1 configuration (used for high voltage generator): ~23.4 kHz PWM (9 bit) */
    TCCR1A = UTIL_BIN8(1000, 0010);  /* OC1A = PWM, OC1B disconnected, 9 bit */
    TCCR1B = UTIL_BIN8(0000, 1001);  /* 9 bit, prescaler=1 */
    OCR1A = 1;      /* set duty cycle to minimum */
    
    /* timer 2 configuration (used for target clock) */
#ifdef TCCR2A
    TCCR2A = UTIL_BIN8(0000, 0010); /* OC2A disconnected, CTC mode (WGM 0,1) */
    TCCR2B = UTIL_BIN8(0000, 0001); /* prescaler=1, CTC mode (WGM 2) */
#else
    TCCR2 = UTIL_BIN8(0000, 1001);  /* OC2 disconnected, prescaler=1, CTC mode */
#endif
    OCR2 = 2;       /* should give 3 MHz clock */
#endif /* METABOARD_HARDWARE */
}

static void s_open()
{
#define BAUD HW_DEBUG_BAUDRATE
#include <util/setbaud.h>
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X0);
#else
    UCSRA &= ~(1 << U2X0);
#endif

    UCSRB = (1 << TXEN) | (1 << RXEN); /* enable rx, tx and rx interrupt */
}

static void s_putc(uchar c)
{
    while(!(UCSRA & (1 << UDRE)));    /* wait for data register empty */
    UDR = c;
}

static uchar s_avail()
{
    return UCSR0A & (1 <<RXC0);
}

static uchar s_getch()
{
    while (!s_avail()) ;
    return UDR0;
}

int main(void)
{
    //wdt_enable(WDTO_1S);
    s_open();
    odDebugInit();
    DBG1(0x00, 0, 0);
    hardwareInit();
    vregInit();
    sei();
    DBG1(0x01, 0, 0);
    for(;;) { 
        /* main event loop */
        //wdt_reset();
        while (s_avail())
            stkSetRxChar(s_getch());
        stkPoll();
        int b;
        while ((b = stkGetTxByte()) >= 0)
            s_putc((uchar) b);
    }
    return 0;
}
