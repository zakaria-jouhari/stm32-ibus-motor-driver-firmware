/*
 * odom.h  -  STM32 -> Raspberry Pi odometry TX bridge over UART7.
 *
 * Sends a fixed 15-byte frame at ODOM_TX_HZ. The Pi's base_driver node
 * differences the signed wheel positions over dt to get velocity, then
 * integrates to /odom for the EKF.
 *
 * Frame (little-endian):
 *   [0]      uint8  sync0            0xC3
 *   [1]      uint8  sync1            0x3C
 *   [2..5]   int32  left_position    signed cumulative ticks
 *   [6..9]   int32  right_position   signed cumulative ticks
 *   [10..11] uint16 dt_ms            ms since previous frame
 *   [12]     uint8  flags            bit0=armed, bit1=rc_alive
 *   [13]     uint8  seq              rolling sequence counter
 *   [14]     uint8  checksum         plain 8-bit sum of bytes [0..13], truncated
 *
 * Reverted from the Ethernet/UDP transport -- see cmd.h for why.
 */
#ifndef ODOM_H
#define ODOM_H

#include "stm32f7xx_hal.h"

#define ODOM_TX_HZ   50u   /* odometry frame rate */

void odom_init(UART_HandleTypeDef *huart);  /* call once after MX_UART7_Init() */
void odom_update(void);                     /* call each loop; sends at ODOM_TX_HZ */

#endif /* ODOM_H */
