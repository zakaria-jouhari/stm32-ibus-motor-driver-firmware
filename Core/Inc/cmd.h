/*
 * cmd.h  -  Pi -> STM32 command receiver (closed-loop drive over UART7).
 *
 * Receives the Pi's command frame and exposes the latest wheel-speed targets +
 * an enable flag + a freshness watchdog. The safety arbitration (RC stays master)
 * and the setpoint->actuation mapping live in motor.c (motor_rc_update).
 *
 * Command frame (Pi -> STM32, UART7 115200 8N1, 9 bytes, little-endian,
 * ~30 Hz heartbeat):
 *   [0]     uint8  sync0    0xC3
 *   [1]     uint8  sync1    0x3C
 *   [2..3]  int16  vL_mm_s  target LEFT  wheel speed, mm/s (signed, + = forward)
 *   [4..5]  int16  vR_mm_s  target RIGHT wheel speed, mm/s (signed)
 *   [6]     uint8  flags    bit0 = enable (autonomy requests motion)
 *   [7]     uint8  seq      rolling 0..255 (gap detection)
 *   [8]     uint8  checksum plain 8-bit sum of bytes [0..7], truncated
 *
 * Reverted from the Ethernet/UDP transport (2026-07 detour) back to this
 * UART7 link -- the Ethernet MAC's transmit path turned out to have a
 * hardware fault on this board (confirmed receiving fine, but transmitted
 * frames never arrived at two different peer devices over two different
 * cables, nor in the PHY's own internal loopback). This link was fully
 * working and tested before that detour started; reverting to it unblocks
 * the robot now. Revisit Ethernet later only with a spare board or bench
 * equipment to inspect the PHY's TX output directly.
 */
#ifndef CMD_H
#define CMD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

#define CMD_FRAME_LEN 9u

void    cmd_init(UART_HandleTypeDef *huart);   /* starts the DMA + IDLE reception on UART7 */

/* Call from HAL_UARTEx_RxEventCallback() when huart->Instance == UART7. */
void    cmd_on_rx_event(uint16_t size);
/* Call from HAL_UART_ErrorCallback() when huart->Instance == UART7. */
void    cmd_on_error(void);

bool    cmd_is_alive(uint32_t timeout_ms);    /* true if a valid frame arrived recently  */
bool    cmd_enable(void);                      /* the enable bit of the latest frame      */
int16_t cmd_target_left(void);                 /* latest LEFT  target, mm/s               */
int16_t cmd_target_right(void);                /* latest RIGHT target, mm/s               */

extern volatile uint32_t cmd_frame_count;      /* valid frames (debug) */
extern volatile uint32_t cmd_error_count;      /* bad sync / bad checksum / wrong length (debug) */

#endif /* CMD_H */
