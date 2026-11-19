/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Dual DC motor driver with fixed-ratio linear speed coupling.
  *
  *          The host sends one signed speed for motor A. Motor B is derived:
  *
  *              vB = (ratio_permille * vA) / 1000
  *
  *          Both speeds pass through an independent first-order ramp so the
  *          wheels never jump discontinuously. A ramp step of 25 per 5 ms
  *          gives a full -1000..+1000 swing in 80 ms (= 400 ms total travel).
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"

/* ---- private state ---- */
static int  s_targetA   = 0;                    /* commanded speed A (per mille) */
static int  s_outA      = 0;                    /* ramped actual speed A         */
static int  s_outB      = 0;                    /* ramped actual speed B         */
static int  s_ratio     = MOTOR_RATIO_DEFAULT;  /* permille, 0..1000             */
static int  s_rampStep  = 25;                   /* per mille per 5 ms tick       */

/* ---- helpers ---- */
static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Move "cur" one ramp step toward "tgt". */
static int ramp_i(int cur, int tgt, int step)
{
    if (step <= 0) return tgt;
    if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

/* Drive both H-bridges. Speeds are per-mille in [-1000, 1000]; the magnitude
 * maps to the TIM3 compare register (ARR = 9999). TIM3 handles both channels;
 * the handle comes from tim.c. */
static void motor_apply(void)
{
    uint32_t dA, dB;
    int magA = s_outA < 0 ? -s_outA : s_outA;
    int magB = s_outB < 0 ? -s_outB : s_outB;
    if (magA > MOTOR_SPEED_MAX) magA = MOTOR_SPEED_MAX;
    if (magB > MOTOR_SPEED_MAX) magB = MOTOR_SPEED_MAX;
    dA = (uint32_t)magA * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;
    dB = (uint32_t)magB * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;

    /* Motor A direction: PB12 / PB13, PWM on TIM3_CH1 (PA6) */
    if (s_outA > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else if (s_outA < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    /* Motor B direction: PB14 / PB15, PWM on TIM3_CH4 (PB1) */
    if (s_outB > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else if (s_outB < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dA);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, dB);
}

/* ---- public API ---- */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12/13/14/15 -> push-pull outputs for the two H-bridges */
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 |
                      GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /* Start PWM on the two motor channels. TIM3 CH1/CH3/CH4 are already
     * configured by MX_TIM3_Init(); CH3 is left at 0 (unused). */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    s_targetA = 0;
    s_outA = 0;
    s_outB = 0;
    s_ratio = MOTOR_RATIO_DEFAULT;
    s_rampStep = 25;
}

void Motor_SetSpeedA(int speed_a)
{
    s_targetA = clamp_i(speed_a, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
}

void Motor_SetRatio(int ratio_permille)
{
    s_ratio = clamp_i(ratio_permille, 0, 1000);
}

int Motor_GetRatio(void) { return s_ratio; }
int Motor_GetSpeedA(void) { return s_outA; }
int Motor_GetSpeedB(void) { return s_outB; }

void Motor_SetRampStep(int step)
{
    s_rampStep = clamp_i(step, 0, MOTOR_SPEED_MAX);
}

void Motor_Tick(void)
{
    int targetB;

    /* Linear coupling: vB = k * vA, computed from the (ramped) A output so
     * the two wheels stay locked to the same ratio during acceleration. */
    targetB = (s_ratio * s_outA) / 1000;

    s_outA = ramp_i(s_outA, s_targetA, s_rampStep);
    targetB = (s_ratio * s_outA) / 1000;
    s_outB = ramp_i(s_outB, targetB, s_rampStep);

    motor_apply();
}

void Motor_Stop(void)
{
    s_targetA = 0;
    s_outA = 0;
    s_outB = 0;
    motor_apply();
}
