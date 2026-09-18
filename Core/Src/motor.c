/*
 * motor.c  -  Differential-drive motor control for the UGV. See motor.h.
 */
#include "motor.h"
#include "ibus.h"
#include "cmd.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;   /* from main.c: TIM1 CH1=PE9 (left), CH2=PE11 (right) */

/* ---------- Tunables ---------- */
#define MOTOR_PWM_PERIOD   1599u   /* TIM1 ARR -> 16 MHz/(1599+1) = 10 kHz PWM.
                                      VERIFY the ZS-X11H accepts this; tune if needed. */
#define RC_SPEED_CH        1u      /* CH2 elevator  (right-stick vertical),  0-based   */
#define RC_STEER_CH        0u      /* CH1 aileron   (right-stick horizontal), 0-based  */
#define RC_CENTER          1500
#define RC_DEADBAND        30      /* us around center treated as 0 (no creep)         */
#define RC_RANGE           500     /* stick travel from center (1500 +/- 500)          */
#define RC_FAILSAFE_MS     100u    /* no valid frame within this -> stop               */

#define CMD_FAILSAFE_MS    300u    /* Pi sends ~30Hz (cmd.h); several missed frames'
                                      margin, comfortably under the Pi README's ~0.5s
                                      total-stop budget (its own cmd_timeout + this). */
#define CMD_MAX_MM_S       800     /* mm/s that maps to the +/-1000 command range.
                                      NOT a true top-speed calibration: the Pi side
                                      caps its own requests at 0.3-0.4 m/s, so this only
                                      needs headroom above that -- too low just drives
                                      slower, never faster than commanded. VERIFY/tune
                                      once real top speed is known. */

#define RAMP_STEP          25      /* max change in command units per motor_set() call.
                                      Main loop runs ~100 Hz (HAL_Delay(10)), so 25/tick
                                      -> 1000 (full scale) in ~40 ticks, ~0.4s zero-to-full.
                                      Applies to BOTH RC and Pi commands (motor_rc_update
                                      funnels both through motor_set()) -- soft start/stop
                                      for normal driving. Does NOT apply to motor_stop_all()
                                      (disarm/failsafe): that stays an immediate hard stop
                                      on purpose, a safety cutoff should never be a glide.
                                      VERIFY/tune by feel once the real motors are running. */

/* Direction calibration: flip a side to 1 if that wheel spins the wrong way.
 * (Left and right motors usually mount mirrored, so one often needs inverting.) */
#define L_DIR_INVERT       0
#define R_DIR_INVERT       0

/* DIR + STOP output pins (all on GPIOE, next to the PWM pins PE9/PE11). */
#define DIR_STOP_PORT      GPIOE
#define L_DIR_PIN          GPIO_PIN_10
#define L_STOP_PIN         GPIO_PIN_12
#define R_DIR_PIN          GPIO_PIN_13
#define R_STOP_PIN         GPIO_PIN_14

/* Momentary arm button: one leg -> PE15 (input, pull-up), other leg -> GND.
 * Pressed = LOW. Each press toggles the armed flag. */
#define BTN_PORT           GPIOE
#define BTN_PIN            GPIO_PIN_15
#define BTN_DEBOUNCE_MS    30u

static int armed = 0;              /* start DISARMED -> no drive until first press */
static int last_sign[2] = {0, 0}; /* last commanded direction per side (+1/-1/0)   */
static int16_t ramped_cmd[2] = {0, 0}; /* current soft-start output per side, see RAMP_STEP */

void motor_init(void)
{
    /* Configure DIR/STOP as push-pull outputs. If the board's 5 V inputs don't
     * recognize the STM32's 3.3 V "high" (motor won't run / wrong dir), switch
     * these to GPIO_MODE_OUTPUT_OD with a pull-up to the board's 5 V. */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = L_DIR_PIN | L_STOP_PIN | R_DIR_PIN | R_STOP_PIN;
    HAL_GPIO_Init(DIR_STOP_PORT, &g);

    /* Arm button: input with pull-up (pressed pulls to GND). */
    GPIO_InitTypeDef b = {0};
    b.Mode  = GPIO_MODE_INPUT;
    b.Pull  = GPIO_PULLUP;
    b.Pin   = BTN_PIN;
    HAL_GPIO_Init(BTN_PORT, &b);

    /* 10 kHz PWM, both channels start at 0% duty. */
    __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    motor_stop_all();
}

/* DIR is ACTIVE LOW: forward -> pin LOW (after per-side invert). */
static void set_dir(motor_side_t side, int forward)
{
    int fwd = forward;
    if (side == MOTOR_LEFT  && L_DIR_INVERT) fwd = !fwd;
    if (side == MOTOR_RIGHT && R_DIR_INVERT) fwd = !fwd;

    GPIO_PinState lvl = fwd ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(DIR_STOP_PORT,
                      (side == MOTOR_LEFT) ? L_DIR_PIN : R_DIR_PIN, lvl);
}

/* STOP is ACTIVE LOW: run -> pin HIGH, stop -> pin LOW. */
static void set_run(motor_side_t side, int run)
{
    GPIO_PinState lvl = run ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DIR_STOP_PORT,
                      (side == MOTOR_LEFT) ? L_STOP_PIN : R_STOP_PIN, lvl);
}

static void set_pwm(motor_side_t side, uint16_t duty)   /* duty 0..MOTOR_PWM_PERIOD */
{
    __HAL_TIM_SET_COMPARE(&htim1,
                          (side == MOTOR_LEFT) ? TIM_CHANNEL_1 : TIM_CHANNEL_2, duty);
}

/* Drives the hardware immediately at exactly `cmd` -- no ramping. Only two
 * callers: motor_set() (after it has computed this tick's ramped step) and
 * motor_stop_all() (which bypasses the ramp entirely -- see RAMP_STEP). */
static void apply_output(motor_side_t side, int16_t cmd)
{
    last_sign[side] = (cmd > 0) ? 1 : ((cmd < 0) ? -1 : 0);

    if (cmd == 0) {                 /* stop: 0% PWM and assert STOP */
        set_pwm(side, 0);
        set_run(side, 0);
        return;
    }

    set_dir(side, cmd > 0);
    set_run(side, 1);               /* release STOP -> allow running */
    uint32_t duty = (uint32_t)abs(cmd) * MOTOR_PWM_PERIOD / 1000u;
    set_pwm(side, (uint16_t)duty);
}

/* Soft start/stop: steps the actual output toward `cmd` by at most RAMP_STEP
 * per call, instead of jumping straight to it. Both the RC path and the Pi
 * cmd_vel path in motor_rc_update() call this same function, so the ramp
 * applies identically no matter which one is driving. */
void motor_set(motor_side_t side, int16_t cmd)
{
    if (cmd >  1000) cmd =  1000;
    if (cmd < -1000) cmd = -1000;

    int16_t delta = (int16_t)(cmd - ramped_cmd[side]);
    if (delta >  RAMP_STEP) delta =  RAMP_STEP;
    if (delta < -RAMP_STEP) delta = -RAMP_STEP;
    ramped_cmd[side] = (int16_t)(ramped_cmd[side] + delta);

    apply_output(side, ramped_cmd[side]);
}

void motor_stop_all(void)
{
    /* Immediate hard stop, deliberately NOT ramped -- disarm/failsafe must cut
     * power now, not glide down. Reset the ramp state too, so the next real
     * command ramps up fresh from an output that's actually zero. */
    ramped_cmd[MOTOR_LEFT]  = 0;
    ramped_cmd[MOTOR_RIGHT] = 0;
    apply_output(MOTOR_LEFT,  0);
    apply_output(MOTOR_RIGHT, 0);
}

int motor_is_armed(void)
{
    return armed;
}

int motor_dir_sign(motor_side_t side)
{
    return last_sign[side];
}

/* Debounced momentary button -> toggle armed state on each press (falling edge). */
void motor_button_update(void)
{
    static int      stable = 1;     /* last debounced level (1 = released) */
    static int      last_raw = 1;
    static uint32_t last_change = 0;

    int raw = (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint32_t now = HAL_GetTick();

    if (raw != last_raw) {          /* level moved -> restart debounce window */
        last_raw = raw;
        last_change = now;
    }

    if ((now - last_change) >= BTN_DEBOUNCE_MS && raw != stable) {
        stable = raw;
        if (stable == 0) {          /* confirmed press (pulled to GND) */
            armed = !armed;
            if (!armed) {
                motor_stop_all();   /* disarm -> stop immediately */
            }
        }
    }
}

static int16_t deadband(int16_t v)
{
    return (v > -RC_DEADBAND && v < RC_DEADBAND) ? 0 : v;
}

static int32_t clamp1000(int32_t v)
{
    if (v >  1000) return  1000;
    if (v < -1000) return -1000;
    return v;
}

static drive_source_t last_source = DRIVE_NONE;

drive_source_t motor_drive_source(void)
{
    return last_source;
}

/* RC stays master: an out-of-deadband stick always overrides a Pi command,
 * and disarm / RC-link-loss always wins over everything (see motor.h). */
void motor_rc_update(void)
{
    if (!armed) {
        motor_stop_all();           /* not armed -> no drive */
        last_source = DRIVE_NONE;
        return;
    }
    if (!ibus_is_alive(RC_FAILSAFE_MS)) {
        motor_stop_all();           /* RC signal lost -> stop, regardless of any cmd */
        last_source = DRIVE_NONE;
        return;
    }

    int16_t speed = deadband((int16_t)ibus_get_channel(RC_SPEED_CH) - RC_CENTER); /* -500..500 */
    int16_t turn  = deadband((int16_t)ibus_get_channel(RC_STEER_CH) - RC_CENTER); /* -500..500 */

    if (speed != 0 || turn != 0) {
        /* Differential mix, then scale +/-RC_RANGE to the +/-1000 command range. */
        int32_t l = clamp1000(((int32_t)(speed + turn) * 1000) / RC_RANGE);
        int32_t r = clamp1000(((int32_t)(speed - turn) * 1000) / RC_RANGE);
        motor_set(MOTOR_LEFT,  (int16_t)l);
        motor_set(MOTOR_RIGHT, (int16_t)r);
        last_source = DRIVE_RC;
        return;
    }

    if (cmd_is_alive(CMD_FAILSAFE_MS) && cmd_enable()) {
        /* Stick is centered and the Pi has a fresh, enabled command -- drive it. */
        int32_t l = clamp1000(((int32_t)cmd_target_left()  * 1000) / CMD_MAX_MM_S);
        int32_t r = clamp1000(((int32_t)cmd_target_right() * 1000) / CMD_MAX_MM_S);
        motor_set(MOTOR_LEFT,  (int16_t)l);
        motor_set(MOTOR_RIGHT, (int16_t)r);
        last_source = DRIVE_CMD;
        return;
    }

    motor_stop_all();               /* centered stick, no valid Pi command -> no drift */
    last_source = DRIVE_NONE;
}
