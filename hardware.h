/*
 * Name: hardware.h
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-07-05
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/*
General Description:
This module defines hardware properties and configuration choices.
*/

#ifndef __hardware_h_included__
#define __hardware_h_included__

/* Arduino Nano clock */
#define F_CPU (16UL*1000*1000)

/* configuration options: */
#define USE_DCD_REPORTING       0
/* If this option is defined to 1, the driver will report carrier detect when
 * the serial device is opened. This is useful on some Unix platforms to allow
 * using the /dev/tty* instead of /dev/cu* devices.
 * Setting this option to 0 saves a couple of bytes in flash and RAM memory.
 */
//#define ENABLE_DEBUG_INTERFACE  1
/* If this option is defined to 1, the device buffers serial data (read from
 * the the ISP connector) and makes it available through vendor specific
 * requests to host based software. This is useful to display debug information
 * sent by the target.
 * Setting this option to 0 saves a couple of bytes in flash memory.
 */
#define HW_DEBUG_BAUDRATE       115200
/* Bit-rate for the UART (for reading debug data from the target).
 */
#ifndef F_CPU
#error "F_CPU must be defined in compiler command line!"
/* #define F_CPU                   12000000 */
/* Our CPU frequency.
 */
#endif


/*
Port        | Function                | dir | value
------------+-------------------------+-----+-------
PORT B
  0         | HVSP Supply               [O]    1
  1 OC1A    | SMPS                      [O]    0
  2         | HVSP RESETHV / LEDHV      [O]    0
  3 OC2     | HVSP SCI / ISP target clk [O]    0
  4         | ISP driver enable         [O]    0
  5         | LED Prog active           [O]    1
  6 XTAL1   | XTAL
  7 XTAL2   | XTAL
PORT C (ADC)
  0         | SMPS feedback             [i]    0
  1         | ISP voltage sense         [i]    0
  2         | ISP SCK                   [O]    0
  3         | ISP MISO                  [I]    1
  4         | ISP MOSI                  [O]    0
  5         | ISP RESET / HVSP RESET    [O]    0
  6 RESET   | Reset
  7 n/a     | *
PORT D
  0 RxD     | ISP TxD                   [I]    1
  1 TxD     | ISP RxD                   [I]    1
  2 Int0    | USB D+                    [i]    0
  3 Int1    | USB D-                    [i]    0
  4 T0      | JUMPER Low Speed          [I]    1
  5 T1      | HVSP SII (PPD1)           [I]    1
  6 AIN0    | HVSP SDI (PPD0)           [I]    1
  7 AIN1    | HVSP SDO (PPD2)           [I]    1
*/

/* The following defines can be used with the PORT_* macros from utils.h */

#define HWPIN_HVSP_SUPPLY   B, 0
#define HWPIN_SMPS_OUT      B, 1
#define HWPIN_HVSP_HVRESET  B, 2
#define HWPIN_HVSP_SCI      B, 3
#define HWPIN_ISP_CLK       B, 3
#define HWPIN_ISP_DRIVER    B, 4
#define HWPIN_LED           B, 5

#define HWPIN_ADC_SMPS      C, 0
#define HWPIN_ADC_VTARGET   C, 1
#define HWPIN_ISP_SCK       C, 2
#define HWPIN_ISP_MISO      C, 3
#define HWPIN_ISP_MOSI      C, 4
#define HWPIN_ISP_RESET     C, 5
#define HWPIN_HVSP_RESET    C, 5

#define HWPIN_JUMPER        D, 4
#define HWPIN_HVSP_SII      D, 5
#define HWPIN_HVSP_SDI      D, 6
#define HWPIN_HVSP_SDO      D, 7

/* Device compatibility: Allow both, ATMega8 and ATMega88/168. The following
 * macros (almost) mimic an ATMega8 based on the ATMega88/168 defines.
 */
#include <avr/io.h>

#ifdef TCCR2A
#   define TCCR2    TCCR2A
#   define COM20    COM2A0
#   define OCR2     OCR2A
#   define HW_SET_T2_PRESCALER(value)   (TCCR2B = (TCCR2B & ~7) | (value & 7))
#   define HW_GET_T2_PRESCALER()        (TCCR2B & 7)
#else
#   define HW_SET_T2_PRESCALER(value)   (TCCR2 = (TCCR2 & ~7) | (value & 7))
#   define HW_GET_T2_PRESCALER()        (TCCR2 & 7)
#endif

#ifdef TCCR0B
#   define TCCR0    TCCR0B
#   define TIMSK    TIMSK0
#endif

#ifdef UBRR0L
#   define UBRRH    UBRR0H
#   define UBRRL    UBRR0L
#   define UCSRA    UCSR0A
#   define UCSRB    UCSR0B
#   define UDR      UDR0
#   define RXEN     RXEN0
#   define TXEN     TXEN0
#   define UDRE     UDRE0
#   define RXCIE    RXCIE0
#endif

#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_PIN  PORTB5

#define LED_ON()  sbi(LED_PORT, LED_PIN)
#define LED_OFF() cbi(LED_PORT, LED_PIN)
#define LED_CONFIGURE_PIN() (sbi(LED_DDR, LED_PIN), cbi(LED_PORT, LED_PIN))

#endif /* __hardware_h_included__ */
