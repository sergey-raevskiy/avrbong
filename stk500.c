#include "stk500.h"

enum {
    stk_msg_stx   = 0x1B,
    stk_msg_token = 0x0E,
};

enum {
    stk_rx_msg_body_maxlen = 275,

    stk_rx_msg_start = 0,       // Always 0x1B (stk_msg_stx)
    stk_rx_msg_sequence_number,
    stk_rx_msg_body_size_hi,
    stk_rx_msg_body_size_lo,
    stk_rx_msg_token,           // Always 0x0E (stk_msg_token)
    stk_rx_msg_body_start,

    stk_rx_msg_maxlen = stk_rx_msg_body_maxlen + stk_rx_msg_body_start + 1,
};

static uint8_t rx_buffer[stk_rx_msg_maxlen];
static uint16_t rx_pos;

#define rx_put(b) (rx_buffer[rx_pos++] = b)

#define rx_msg_body() (rx_buffer + stk_rx_msg_body_start)
#define rx_msg_body_len() (makeu16(rx_buffer[stk_rx_msg_body_size_hi], rx_buffer[stk_rx_msg_body_size_lo]))
#define rx_msg_checksum() (rx_buffer + stk_rx_msg_body_start + rx_msg_body_len())

static void stk500_recv()
{
    rx_pos = stk_rx_msg_body_start;
}

static void stk500_put_byte(uint8_t b)
{
    switch (rx_pos)
    {
    case stk_rx_msg_start:
        if (b != stk_msg_stx)
            goto error;

    case stk_rx_msg_token:
        if (b != stk_msg_stx)
            goto error;

    default:
        rx_put(b);
    }

    if (rx_pos == stk_rx_msg_start && b == stk_msg_stx)
    {
        rx_put(b);
    }
    else if (rx_pos == stk_rx_msg_token && b == stk_msg_token)
    {
        rx_put(b);
    }
    else if (rx_pos < stk_rx_msg_maxlen)
    {
        rx_put(b);
    }
    else
    {
        // error
        rx_pos = stk_rx_msg_start;
    }
}
