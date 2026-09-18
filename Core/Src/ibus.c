/*
 * ibus.c  -  FlySky iBUS receiver parser (FS-iA6B) for STM32 HAL
 * See ibus.h for the protocol description.
 */
#include "ibus.h"
#include <string.h>

volatile ibus_t ibus;

static UART_HandleTypeDef *ibus_huart;
static uint8_t ibus_buf[IBUS_FRAME_LEN];

/* (Re)arm one DMA reception that completes on the IDLE line after the burst. */
static void ibus_arm(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(ibus_huart, ibus_buf, IBUS_FRAME_LEN);
    /* The half-transfer interrupt would fire mid-frame and call us with a
     * partial Size. We only care about the complete 32-byte burst, so mute it. */
    __HAL_DMA_DISABLE_IT(ibus_huart->hdmarx, DMA_IT_HT);
}

void ibus_init(UART_HandleTypeDef *huart)
{
    ibus_huart = huart;
    memset((void *)&ibus, 0, sizeof(ibus));
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        ibus.channel[i] = 1500u;   /* safe neutral until the first real frame */
    }
    ibus_arm();
}

/* Validate header + checksum, and copy channels out if the frame is good. */
static bool ibus_validate_and_parse(const uint8_t *f)
{
    if (f[0] != 0x20 || f[1] != 0x40) {
        return false;                       /* not a frame start */
    }

    uint16_t checksum = 0xFFFFu;
    for (uint8_t i = 0; i < 30; i++) {
        checksum -= f[i];                   /* 0xFFFF - sum(byte[0..29]) */
    }
    uint16_t rx_checksum = (uint16_t)(f[30] | (f[31] << 8));
    if (checksum != rx_checksum) {
        return false;                       /* corrupted / misaligned */
    }

    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        ibus.channel[i] = (uint16_t)(f[2 + i * 2] | (f[3 + i * 2] << 8));
    }
    return true;
}

void ibus_on_rx_event(uint16_t size)
{
    if (size == IBUS_FRAME_LEN && ibus_validate_and_parse(ibus_buf)) {
        ibus.last_update_ms = HAL_GetTick();
        ibus.frame_count++;
    } else {
        ibus.error_count++;                 /* short frame or bad checksum -> drop */
    }
    ibus_arm();                             /* re-arm during the ~7 ms idle gap */
}

void ibus_on_error(void)
{
    /* An overrun/framing/noise error aborts the DMA reception; restart it so
     * the link recovers instead of going permanently silent. */
    __HAL_UART_CLEAR_OREFLAG(ibus_huart);
    ibus_arm();
}

bool ibus_is_alive(uint32_t timeout_ms)
{
    return (HAL_GetTick() - ibus.last_update_ms) < timeout_ms;
}

uint16_t ibus_get_channel(uint8_t ch)
{
    if (ch >= IBUS_NUM_CHANNELS) {
        return 1500u;
    }
    return ibus.channel[ch];
}
