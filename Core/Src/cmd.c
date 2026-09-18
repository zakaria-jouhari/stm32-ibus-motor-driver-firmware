/*
 * cmd.c  -  Pi -> STM32 command receiver over UART7. See cmd.h for the frame layout.
 * Reception pattern mirrors ibus.c's (DMA + IDLE-line completion) exactly.
 */
#include "cmd.h"
#include <string.h>

volatile uint32_t cmd_frame_count;
volatile uint32_t cmd_error_count;

static UART_HandleTypeDef *cmd_huart;
static uint8_t  cmd_buf[CMD_FRAME_LEN];

static volatile int16_t  target_left;
static volatile int16_t  target_right;
static volatile bool     enable_flag;
static volatile uint32_t last_update_ms;

/* (Re)arm one IT-driven reception that completes on the IDLE line after the
 * burst. UART7 has no DMA stream assigned in this project (only USART6/ibus
 * does -- see the .ioc), so this uses the interrupt-only variant. */
static void cmd_arm(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(cmd_huart, cmd_buf, CMD_FRAME_LEN);
}

void cmd_init(UART_HandleTypeDef *huart)
{
    cmd_huart = huart;
    target_left = 0;
    target_right = 0;
    enable_flag = false;
    last_update_ms = 0;
    cmd_frame_count = 0;
    cmd_error_count = 0;
    cmd_arm();
}

static bool cmd_validate_and_parse(const uint8_t *f)
{
    if (f[0] != 0xC3u || f[1] != 0x3Cu) {
        return false;                        /* not a frame start */
    }

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        checksum = (uint8_t)(checksum + f[i]);
    }
    if (checksum != f[8]) {
        return false;                        /* corrupted / misaligned */
    }

    target_left  = (int16_t)(f[2] | (f[3] << 8));
    target_right = (int16_t)(f[4] | (f[5] << 8));
    enable_flag  = (f[6] & 0x01u) != 0u;
    return true;
}

void cmd_on_rx_event(uint16_t size)
{
    if (size == CMD_FRAME_LEN && cmd_validate_and_parse(cmd_buf)) {
        last_update_ms = HAL_GetTick();
        cmd_frame_count++;
    } else {
        cmd_error_count++;                   /* short frame or bad checksum -> drop */
    }
    cmd_arm();                                /* re-arm during the inter-frame gap */
}

void cmd_on_error(void)
{
    /* An overrun/framing/noise error aborts the DMA reception; restart it so
     * the link recovers instead of going permanently silent. */
    __HAL_UART_CLEAR_OREFLAG(cmd_huart);
    cmd_arm();
}

bool cmd_is_alive(uint32_t timeout_ms)
{
    return (HAL_GetTick() - last_update_ms) < timeout_ms;
}

bool cmd_enable(void)
{
    return enable_flag;
}

int16_t cmd_target_left(void)
{
    return target_left;
}

int16_t cmd_target_right(void)
{
    return target_right;
}
