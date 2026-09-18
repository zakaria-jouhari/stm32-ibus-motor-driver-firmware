/*
 * motor.h  -  Differential-drive motor control for the UGV
 *
 * Two ZS-X11H BLDC controllers, one per driven wheel. Each board takes:
 *   - PWM  -> speed   (TIM1: CH1=PE9 left, CH2=PE11 right; board "P" pin, jumper set)
 *   - DIR  -> direction (ACTIVE LOW)
 *   - STOP -> run/stop  (ACTIVE LOW: HIGH = run, LOW = stop)
 * Speed channel = CH2 (right-stick vertical), steering = CH1 (right-stick horizontal),
 * both spring-centered so releasing the stick stops the vehicle.
 *
 * Drive source arbitration (RC always stays master, see motor_rc_update()):
 *   not armed, or RC link lost           -> stop
 *   RC stick outside its deadband        -> RC wins (manual override)
 *   stick centered + a fresh, enabled
 *     Pi cmd_vel frame (see cmd.h)       -> Pi's target_left/right drives
 *   stick centered, no valid Pi command  -> stop (no drift)
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1
} motor_side_t;

typedef enum {
    DRIVE_NONE = 0,   /* stopped: disarmed, RC lost, or no valid command */
    DRIVE_RC   = 1,   /* RC stick is driving (override) */
    DRIVE_CMD  = 2,   /* Pi cmd_vel is driving */
} drive_source_t;

/* Configure TIM1 PWM + DIR/STOP GPIOs and leave both motors stopped. */
void motor_init(void);

/* Drive one side. cmd in -1000..+1000 (sign = direction, magnitude = speed). */
void motor_set(motor_side_t side, int16_t cmd);

/* Immediately stop both motors (assert STOP, 0% PWM). */
void motor_stop_all(void);

/* Arbitrate RC vs. Pi cmd_vel (see arbitration order above) and drive motors. */
void motor_rc_update(void);

/* Which source actually drove the last motor_rc_update() call — for debug/VCP. */
drive_source_t motor_drive_source(void);

/* Poll the arm button (debounced). Each press toggles armed/disarmed. Call ~100 Hz. */
void motor_button_update(void);

/* 1 = armed (drive enabled), 0 = disarmed (motors forced stop). */
int motor_is_armed(void);

/* Last commanded direction for a side: +1 forward, -1 reverse, 0 stopped.
 * Used to sign the encoder speed (the S pin gives magnitude only). */
int motor_dir_sign(motor_side_t side);

#endif /* MOTOR_H */
