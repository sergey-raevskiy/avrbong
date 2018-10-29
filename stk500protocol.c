/*
 * Name: stk500protocol.c
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-06-19
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

#include "hardware.h"
#include <avr/pgmspace.h>
#include <string.h>
#include "oddebug.h"
#include "stk500protocol.h"
#include "utils.h"
#include "isp.h"
#include "timer.h"

/* The following versions are reported to the programming software: */
#define STK_VERSION_HW      1
#define STK_VERSION_MAJOR   2
#define STK_VERSION_MINOR   4

enum {
    stk_msg_start = 0,
    stk_msg_seq_num,
    stk_msg_body_len_hi,
    stk_msg_body_len_lo,
    stk_msg_token,
    stk_msg_header_len,
    stk_msg_body_start = stk_msg_header_len
};

#define stk_msg_len(body_len) (stk_msg_header_len + (body_len) + 1)

enum {
    stk_msg_body_maxlen = 275,
    stk_msg_maxlen = stk_msg_len(stk_msg_body_maxlen)
};

#define RX_TIMEOUT      200 /* timeout in milliseconds */

#define STK_TXMSG_START 5


static uchar        rxBuffer[stk_msg_maxlen];
static uint         rxPos;
static uint         rxLen;
static uchar        rxBlockAvailable;

static uchar        txBuffer[stk_msg_maxlen];
static uint         txPos, txLen;

stkParam_t      stkParam = {{
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    STK_VERSION_HW, STK_VERSION_MAJOR, STK_VERSION_MINOR, 0, 50, 0, 1, 0x80,
                    2, 0, 0xaa, 0, 0, 0, 0, 0,
                }};
utilDword_t     stkAddress;

/* ------------------------------------------------------------------------- */

void    stkIncrementAddress(void)
{
    stkAddress.dword++;
}

/* ------------------------------------------------------------------------- */

static void stkSetTxMessage(uint len)
{
    uchar sum = 0;
    uint i;

    txBuffer[stk_msg_start] = STK_STX;
    txBuffer[stk_msg_seq_num] = rxBuffer[stk_msg_seq_num];
    txBuffer[stk_msg_body_len_hi] = hibyte(len);
    txBuffer[stk_msg_body_len_lo] = lobyte(len);
    txBuffer[stk_msg_token] = STK_TOKEN;

    txPos = 0;
    txLen = stk_msg_len(len);

    for (i = 0; i < txLen - 1; i++)
        sum = sum ^ txBuffer[i];

    txBuffer[txLen - 1] = sum;

    DBG1(0xe0, txBuffer, txLen);
}

/* ------------------------------------------------------------------------- */

static void setParameter(uchar index, uchar value)
{
    if(index == STK_PARAM_OSC_PSCALE){
        HW_SET_T2_PRESCALER(value);
    }else if(index == STK_PARAM_OSC_CMATCH){
        OCR2 = value;
    }
    index &= 0x1f;
    stkParam.bytes[index] = value;
}

static uchar getParameter(uchar index)
{
    if(index == STK_PARAM_OSC_PSCALE)
        return HW_GET_T2_PRESCALER();
    if(index == STK_PARAM_OSC_CMATCH)
        return OCR2;
    index &= 0x1f;
    return stkParam.bytes[index];
}

/* ------------------------------------------------------------------------- */

/* Use defines for the switch statement so that we can choose between an
 * if()else if() and a switch/case based implementation. switch() is more
 * efficient for a LARGE set of sequential choices, if() is better in all other
 * cases.
 */
#if 0
#define SWITCH_START        if(0){
#define SWITCH_CASE(value)  }else if(cmd == (value)){
#define SWITCH_CASE2(v1,v2) }else if(cmd == (v1) || cmd == (v2)){
#define SWITCH_CASE3(v1,v2,v3) }else if(cmd == (v1) || cmd == (v2) || (cmd == v3)){
#define SWITCH_CASE4(v1,v2,v3,v4) }else if(cmd == (v1) || cmd == (v2) || cmd == (v3) || cmd == (v4)){
#define SWITCH_DEFAULT      }else{
#define SWITCH_END          }
#else
#define SWITCH_START        switch(cmd){{
#define SWITCH_CASE(value)  }break; case (value):{
#define SWITCH_CASE2(v1,v2) }break; case (v1): case(v2):{
#define SWITCH_CASE3(v1,v2,v3) }break; case (v1): case(v2): case(v3):{
#define SWITCH_CASE4(v1,v2,v3,v4) }break; case (v1): case(v2): case(v3): case(v4):{
#define SWITCH_DEFAULT      }break; default:{
#define SWITCH_END          }}
#endif

void stkEvaluateRxMessage(void) /* not static to prevent inlining */
{
uchar       i, cmd;
utilWord_t  len = {2};  /* defaults to cmd + error code */
void        *param;

    DBG1(0xf1, rxBuffer, rxLen.bytes[0]);
    cmd = rxBuffer[STK_TXMSG_START];
    txBuffer[STK_TXMSG_START] = cmd;
    txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_OK;
    param = &rxBuffer[STK_TXMSG_START + 1];
    SWITCH_START
    SWITCH_CASE(STK_CMD_SIGN_ON)
        static const PROGMEM char string[] = {8, 'S', 'T', 'K', '5', '0', '0', '_', '2', 0};
        char *p = (char *)&txBuffer[STK_TXMSG_START + 2];
        strcpy_P(p, string);
        len.bytes[0] = 11;
    SWITCH_CASE(STK_CMD_SET_PARAMETER)
        setParameter(rxBuffer[STK_TXMSG_START + 1], rxBuffer[STK_TXMSG_START + 2]);
    SWITCH_CASE(STK_CMD_GET_PARAMETER)
        txBuffer[STK_TXMSG_START + 2] = getParameter(rxBuffer[STK_TXMSG_START + 1]);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_OSCCAL)
        txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_FAILED;
        /* not yet implemented */
    SWITCH_CASE(STK_CMD_LOAD_ADDRESS)
        for(i=0;i<4;i++){
            stkAddress.bytes[3-i] = rxBuffer[STK_TXMSG_START + 1 + i];
        }
    SWITCH_CASE(STK_CMD_SET_CONTROL_STACK)
        /* AVR Studio sends:
        1b 08 00 21 0e 2d 
        4c 0c 1c 2c 3c 64 74 66
        68 78 68 68 7a 6a 68 78
        78 7d 6d 0c 80 40 20 10
        11 08 04 02 03 08 04 00
        bf
        */
        /* dummy: ignore */
    SWITCH_CASE(STK_CMD_ENTER_PROGMODE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispEnterProgmode(param);
    SWITCH_CASE(STK_CMD_LEAVE_PROGMODE_ISP)
        ispLeaveProgmode(param);
    SWITCH_CASE(STK_CMD_CHIP_ERASE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispChipErase(param);
    SWITCH_CASE(STK_CMD_PROGRAM_FLASH_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramMemory(param, 0);
    SWITCH_CASE(STK_CMD_READ_FLASH_ISP)
        len.word = 1 + ispReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 0);
    SWITCH_CASE(STK_CMD_PROGRAM_EEPROM_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramMemory(param, 1);
    SWITCH_CASE(STK_CMD_READ_EEPROM_ISP)
        len.word = 1 + ispReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 1);
    SWITCH_CASE(STK_CMD_PROGRAM_FUSE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramFuse(param);
    SWITCH_CASE4(STK_CMD_READ_FUSE_ISP, STK_CMD_READ_LOCK_ISP, STK_CMD_READ_SIGNATURE_ISP, STK_CMD_READ_OSCCAL_ISP)
        txBuffer[STK_TXMSG_START + 2] = ispReadFuse(param);
        txBuffer[STK_TXMSG_START + 3] = STK_STATUS_CMD_OK;
        len.bytes[0] = 4;
    SWITCH_CASE(STK_CMD_PROGRAM_LOCK_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramFuse(param);
    SWITCH_CASE(STK_CMD_SPI_MULTI)
        len.word = 1 + ispMulti(param, (void *)&txBuffer[STK_TXMSG_START + 1]);
    SWITCH_DEFAULT  /* unknown command */
        DBG1(0xf8, 0, 0);
        txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_FAILED;
    SWITCH_END
    stkSetTxMessage(len.word);
}

/* ------------------------------------------------------------------------- */

void    stkSetRxChar(uchar c)
{
    if (timerLongTimeoutOccurred())
    {
        /* reset state */
        rxPos = 0;
    }

    if (rxPos == stk_msg_start)
    {
        /* only accept STX as the first character */
        if (c == STK_STX)
            rxBuffer[rxPos++] = c;
    }
    else
    {
        if (rxPos < stk_msg_maxlen)
        {
            rxBuffer[rxPos++] = c;

            if (rxPos == stk_msg_token)
            {
                /* do we have length byte? */
                rxLen = makeword(rxBuffer[stk_msg_body_len_hi], rxBuffer[stk_msg_body_len_lo]);

                if (rxLen > stk_msg_body_maxlen)
                {
                    /* illegal length */
                    rxPos = 0;      /* reset state */
                }

                rxLen = stk_msg_len(rxLen);
            }
            else if (rxPos == stk_msg_body_start)
            {
                /* check whether this is the token byte */
                if (c != STK_TOKEN)
                    rxPos = 0;  /* reset state */
            }
            else if (rxPos >= stk_msg_body_start && rxPos == rxLen)
            {
                /* message is complete */
                uchar sum = 0;
                uchar *p = rxBuffer;
                while(rxPos)
                {
                    /* decrement rxPos down to 0 -> reset state */
                    sum ^= *p++;
                    rxPos--;
                }

                if(sum == 0)
                {
                    /* check sum is correct, evaluate rx message */
                    rxBlockAvailable = 1;
                }
                else
                {
                    /* checksum error */
                    DBG2(0xf2, rxBuffer, rxLen);
                    txBuffer[stk_msg_body_start + 0] = STK_ANSWER_CKSUM_ERROR;
                    txBuffer[stk_msg_body_start + 1] = STK_ANSWER_CKSUM_ERROR;
                    stkSetTxMessage(2);
                }
            }
        }
        else
        {
            /* overflow */
            rxPos = 0;  /* reset state */
        }
    }

    timerSetupLongTimeout(RX_TIMEOUT);
}

int stkGetTxCount(void)
{
    return txLen - txPos;
}

int stkGetTxByte(void)
{
uchar   c;

    if(txLen == 0){
        return -1;
    }
    c = txBuffer[txPos++];
    if(txPos >= txLen){         /* transmit ready */
        txPos = txLen = 0;
    }
    return c;
}

void    stkPoll(void)
{
    if(rxBlockAvailable){
        rxBlockAvailable = 0;
        stkEvaluateRxMessage();
    }
}

/* ------------------------------------------------------------------------- */
