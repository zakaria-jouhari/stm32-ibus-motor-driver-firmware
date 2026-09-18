/*
 * encoder.h  -  Wheel odometry from the ZS-X11H "S" (speed) tachometer pins.
 *
 * Each board's S pin emits a pulse train whose frequency is proportional to wheel
 * rotation (derived from the motor Hall sensors). We count those pulses with a timer
 * per wheel (external-clock counter mode) and convert to wheel speed.
 *
 *   Left  wheel S pin -> TIM4_ETR (PE0), External Clock Mode 2
 *   Right wheel S pin -> TIM3_ETR (PD2), External Clock Mode 2
 *
 * The S pin gives speed MAGNITUDE only; direction comes from the commanded DIR
 * (motor_dir_sign), which is fine for a commanded drive (no quadrature).
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "motor.h"   /* motor_side_t, motor_dir_sign() */

/* ---- CALIBRATE THIS ----------------------------------------------------------
 * Rotate one wheel EXACTLY one full turn (slowly) and read the encoder_pulses()
 * delta. That count is ENC_PULSES_PER_REV. Do both wheels; they should match.
 * Get this wrong and every odometry velocity is off by a constant factor.        */
#define ENC_PULSES_PER_REV   46.0f     /* measured: 460 pulses over 10 turns (right wheel) */

#define WHEEL_RADIUS_M       0.130f    /* from the diff-drive model                */
#define ENC_SAMPLE_MS        20u       /* odometry sample period (50 Hz)           */

void     encoder_init(void);                  /* start both timers as pulse counters     */
void     encoder_update(void);                /* call ~100 Hz; samples at ENC_SAMPLE_MS  */
float    encoder_speed_mps(motor_side_t s);   /* SIGNED wheel speed (m/s)                */
float    encoder_freq_hz(motor_side_t s);     /* raw pulse frequency, magnitude (Hz)     */
uint32_t encoder_pulses(motor_side_t s);      /* cumulative pulses, unsigned (calibration) */
int32_t  encoder_position(motor_side_t s);    /* SIGNED cumulative ticks (odometry, for Pi) */

#endif /* ENCODER_H */
