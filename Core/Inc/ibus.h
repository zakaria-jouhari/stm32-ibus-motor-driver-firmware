/*
 * ibus.h  -  FlySky iBUS receiver parser (FS-iA6B) for STM32 HAL
 *
 * Protocol: 115200 baud, 8N1, NOT inverted.
 * Frame: 32 bytes, sent as a burst every ~7 ms.
 *   [0]      = 0x20  (frame length = 32)
 *   [1]      = 0x40  (command / type)
 *   [2..29]  = 14 channels, 2 bytes each, little-endian, value ~1000..2000 us
 *   [30..31] = checksum, little-endian = 0xFFFF - sum(byte[0..29])
 */
#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

#define IBUS_FRAME_LEN     32u
#define IBUS_NUM_CHANNELS  14u

typedef struct {
    uint16_t channel[IBUS_NUM_CHANNELS]; /* channel values in microseconds (~1000..2000) */
    uint32_t last_update_ms;             /* HAL_GetTick() at last VALID frame             */
    uint32_t frame_count;                /* number of valid frames received               */
    uint32_t error_count;                /* bad header / bad checksum / wrong length       */
} ibus_t;

extern volatile ibus_t ibus;

/* Call once after MX_USART6_UART_Init(). Starts the DMA + IDLE reception. */
void ibus_init(UART_HandleTypeDef *huart);

/* Call from HAL_UARTEx_RxEventCallback() when huart->Instance == USART6. */
void ibus_on_rx_event(uint16_t size);

/* Call from HAL_UART_ErrorCallback() when huart->Instance == USART6. */
void ibus_on_error(void);

/* True if a valid frame arrived within the last timeout_ms (RC failsafe check). */
bool ibus_is_alive(uint32_t timeout_ms);

/* Safe channel read (0-based: ch 0 = CH1). Returns 1500 (center) if out of range. */
uint16_t ibus_get_channel(uint8_t ch);

#endif /* IBUS_H */
