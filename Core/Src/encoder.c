/*
 * encoder.c  -  Wheel odometry from the ZS-X11H S (speed) pins. See encoder.h.
 */
#include "encoder.h"
#include "stm32f7xx_hal.h"
#include <math.h>

/* Defined by CubeMX once TIM3/TIM4 are added (external clock mode). */
extern TIM_HandleTypeDef htim3;   /* Right wheel: TIM3_ETR = PD2  (External Clock Mode 2) */
extern TIM_HandleTypeDef htim4;   /* Left  wheel: TIM4_ETR = PE0  (External Clock Mode 2) */

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

typedef struct {
    uint16_t last_cnt;     /* timer count at previous sample */
    uint32_t total;        /* accumulated pulses, unsigned (calibration) */
    int32_t  position;     /* signed cumulative ticks (+= delta*dir) for odometry */
    float    freq_hz;      /* pulse frequency, magnitude */
    float    speed_mps;    /* wheel speed magnitude */
} enc_t;

static enc_t    enc[2];    /* [MOTOR_LEFT], [MOTOR_RIGHT] */
static uint32_t last_sample;

static TIM_HandleTypeDef *htim_of(motor_side_t s)
{
    return (s == MOTOR_LEFT) ? &htim4 : &htim3;
}

void encoder_init(void)
{
    for (int i = 0; i < 2; i++) {
        enc[i].last_cnt = 0;
        enc[i].total = 0;
        enc[i].freq_hz = 0.0f;
        enc[i].speed_mps = 0.0f;
    }
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim3);   /* counts external edges on TI1 (no internal clock) */
    HAL_TIM_Base_Start(&htim4);
    last_sample = HAL_GetTick();
}

void encoder_update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - last_sample;
    if (dt < ENC_SAMPLE_MS) {
        return;                         /* not time to sample yet */
    }
    last_sample = now;

    for (int i = 0; i < 2; i++) {
        TIM_HandleTypeDef *h = htim_of((motor_side_t)i);
        uint16_t cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(h);
        uint16_t delta = (uint16_t)(cnt - enc[i].last_cnt);   /* 16-bit wrap-safe */
        enc[i].last_cnt = cnt;
        enc[i].total   += delta;
        enc[i].position += (int32_t)delta * motor_dir_sign((motor_side_t)i); /* signed */

        float freq = (float)delta * 1000.0f / (float)dt;      /* pulses/s */
        enc[i].freq_hz = freq;
        float omega = 2.0f * M_PI * freq / ENC_PULSES_PER_REV; /* rad/s */
        enc[i].speed_mps = omega * WHEEL_RADIUS_M;             /* m/s, magnitude */
    }
}

float encoder_speed_mps(motor_side_t s)
{
    return (float)motor_dir_sign(s) * enc[s].speed_mps;       /* sign from commanded DIR */
}

float encoder_freq_hz(motor_side_t s)
{
    return enc[s].freq_hz;
}

uint32_t encoder_pulses(motor_side_t s)
{
    return enc[s].total;
}

int32_t encoder_position(motor_side_t s)
{
    return enc[s].position;
}
