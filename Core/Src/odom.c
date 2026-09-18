/*
 * odom.c  -  STM32 -> Pi odometry TX bridge over UART7. See odom.h for the layout.
 */
#include "odom.h"
#include "encoder.h"
#include "motor.h"
#include "ibus.h"
#include <string.h>

#define ODOM_TX_MS     (1000u / ODOM_TX_HZ)
#define ODOM_FRAME_LEN 15u

static UART_HandleTypeDef *odom_huart;
static uint32_t            last_tx;
static uint8_t             seq;

void odom_init(UART_HandleTypeDef *huart)
{
    odom_huart = huart;
    last_tx = HAL_GetTick();
    seq = 0;
}

static void put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

void odom_update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - last_tx;
    if (dt < ODOM_TX_MS) {
        return;
    }
    last_tx = now;

    uint16_t dt_ms = (dt > 0xFFFFu) ? 0xFFFFu : (uint16_t)dt;
    uint8_t  flags = (uint8_t)((motor_is_armed()   ? 0x01 : 0x00) |
                              (ibus_is_alive(100)  ? 0x02 : 0x00));

    uint8_t frame[ODOM_FRAME_LEN];
    frame[0] = 0xC3;
    frame[1] = 0x3C;
    put_i32(&frame[2], encoder_position(MOTOR_LEFT));
    put_i32(&frame[6], encoder_position(MOTOR_RIGHT));
    frame[10] = (uint8_t)dt_ms;
    frame[11] = (uint8_t)(dt_ms >> 8);
    frame[12] = flags;
    frame[13] = seq++;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 14; i++) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[14] = checksum;

    HAL_UART_Transmit(odom_huart, frame, ODOM_FRAME_LEN, 5u);
}
