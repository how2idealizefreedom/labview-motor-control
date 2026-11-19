/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Dual DC motor driver with sine/cosine speed coupling.
  *
  *          Phase of each motor, derived from its per-mille speed command:
  *              v1 =  a      * pi / 2000
  *              v2 = (1000-b) * pi / 2000
  *
  *          Constraint enforced here:
  *              sin(v1) = k * cos(v2)
  *
  *          Solving for b:
  *              cos(v2) = sin(v1) / k
  *              v2      = acos( sin(v1) / k )
  *              b       = 1000 - v2 * 2000 / pi
  *
  *          Real solution needs |sin(v1)| <= k, so command A is limited to
  *              |a| <= (2000/pi) * asin(k)
  *          With k=1 this limit is 1000 and b equals a.
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"
#include <math.h>

#define PI_OVER_2000   0.00157079632679f
#define TWOTHOUSAND_OVER_PI 636.61977236758f

/* ---- private state ---- */
static int  s_targetA   = 0;
static int  s_outA      = 0;
static int  s_outB      = 0;
static int  s_ratio     = MOTOR_RATIO_DEFAULT;   /* permille, 1..1000 */
static int  s_rampStep  = 25;                    /* per mille per 5 ms */

/* ---- helpers ---- */
static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int ramp_i(int cur, int tgt, int step)
{
    if (step <= 0) return tgt;
    if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

/* Largest |a| for which sin(v1) = k*cos(v2) has a real solution. */
static int speed_a_limit(int k_permille)
{
    float k = (float)k_permille / 1000.0f;
    if (k < 0.001f) return 0;
    return (int)(TWOTHOUSAND_OVER_PI * asinf(k));
}

/* Solve sin(v1) = k*cos(v2) for motor B's command. */
static int coupling_b(int a, int k_permille)
{
    float k = (float)k_permille / 1000.0f;
    float c;

    if (k < 0.001f) return 0;
    c = sinf((float)a * PI_OVER_2000) / k;
    if (c > 1.0f)  c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return (int)(1000.0f - acosf(c) * TWOTHOUSAND_OVER_PI);
}

/* Drive both H-bridges. Speeds are per-mille in [-1000, 1000]. */
static void motor_apply(void)
{
    uint32_t dA, dB;
    int magA = s_outA < 0 ? -s_outA : s_outA;
    int magB = s_outB < 0 ? -s_outB : s_outB;

    if (magA > MOTOR_SPEED_MAX) magA = MOTOR_SPEED_MAX;
    if (magB > MOTOR_SPEED_MAX) magB = MOTOR_SPEED_MAX;
    dA = (uint32_t)magA * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;
    dB = (uint32_t)magB * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;

    /* Motor A: PB12/PB13 direction, TIM3_CH1 (PA6) PWM */
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

    /* Motor B: PB14/PB15 direction, TIM3_CH4 (PB1) PWM */
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

    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 |
                      GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

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
    s_ratio = clamp_i(ratio_permille, 1, 1000);
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
    int a_lim = speed_a_limit(s_ratio);
    int targetB;

    s_outA = ramp_i(s_outA, clamp_i(s_targetA, -a_lim, a_lim), s_rampStep);
    targetB = coupling_b(s_outA, s_ratio);
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
