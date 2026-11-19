/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Dual DC motor driver.
  *
  *          Coupling:  sin(v1) = k * cos(v2)
  *              v1 =  a      * pi / 2000
  *              v2 = (1000-b) * pi / 2000
  *
  *          Two control paths:
  *            open loop  (default): ramped command -> PWM directly
  *            closed loop (L1):     ramped command -> PI -> PWM, encoder feedback
  *
  *          A command older than MOTOR_CMD_TIMEOUT_MS brakes both motors.
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"
#include "coupling.h"
#include "encoder.h"
#include "pid.h"

/* ---- private state ---- */
static int  s_targetA   = 0;
static int  s_cmdA      = 0;
static int  s_cmdB      = 0;
static int  s_outA      = 0;
static int  s_outB      = 0;
static int  s_ratio     = MOTOR_RATIO_DEFAULT;
static int  s_rampStep  = 25;
static int  s_closedLoop = 0;
static int  s_state     = MOTOR_STATE_IDLE;
static uint32_t s_lastCmd = 0;

static PID_t s_pidA;
static PID_t s_pidB;

/* ---- helpers ---- */
static void motor_apply(void)
{
    uint32_t dA, dB;
    int magA = s_outA < 0 ? -s_outA : s_outA;
    int magB = s_outB < 0 ? -s_outB : s_outB;

    if (magA > MOTOR_SPEED_MAX) magA = MOTOR_SPEED_MAX;
    if (magB > MOTOR_SPEED_MAX) magB = MOTOR_SPEED_MAX;
    dA = (uint32_t)magA * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;
    dB = (uint32_t)magB * (MOTOR_PWM_ARR - 1u) / MOTOR_SPEED_MAX;

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

static void motor_hard_stop(void)
{
    s_targetA = 0;
    s_cmdA = 0;
    s_cmdB = 0;
    s_outA = 0;
    s_outB = 0;
    PID_Reset(&s_pidA);
    PID_Reset(&s_pidB);
    motor_apply();
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

    Encoder_Init();

    PID_Init(&s_pidA, 0.40f, 1.50f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    PID_Init(&s_pidB, 0.40f, 1.50f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

    s_targetA = 0;
    s_cmdA = 0;
    s_cmdB = 0;
    s_outA = 0;
    s_outB = 0;
    s_ratio = MOTOR_RATIO_DEFAULT;
    s_rampStep = 25;
    s_closedLoop = 0;
    s_state = MOTOR_STATE_IDLE;
    s_lastCmd = HAL_GetTick();
}

void Motor_SetSpeedA(int speed_a)
{
    s_targetA = Coupling_Clamp(speed_a, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    Motor_FeedWatchdog();
}

void Motor_SetRatio(int ratio_permille)
{
    s_ratio = Coupling_Clamp(ratio_permille, 1, 1000);
    Motor_FeedWatchdog();
}

void Motor_SetRampStep(int step)
{
    s_rampStep = Coupling_Clamp(step, 0, MOTOR_SPEED_MAX);
}

void Motor_SetClosedLoop(int on)
{
    int newState = on ? 1 : 0;
    if (newState != s_closedLoop) {
        s_closedLoop = newState;
        s_outA = s_cmdA;
        s_outB = s_cmdB;
        PID_Reset(&s_pidA);
        PID_Reset(&s_pidB);
    }
    Motor_FeedWatchdog();
}

void Motor_FeedWatchdog(void)
{
    s_lastCmd = HAL_GetTick();
}

void Motor_Tick(void)
{
    int a_lim;

    Encoder_Sample();

    if ((uint32_t)(HAL_GetTick() - s_lastCmd) > MOTOR_CMD_TIMEOUT_MS) {
        if (s_state != MOTOR_STATE_TIMEOUT) {
            motor_hard_stop();
            s_state = MOTOR_STATE_TIMEOUT;
        }
        return;
    }

    a_lim = Coupling_ALimit(s_ratio);
    s_cmdA = Coupling_Ramp(s_cmdA, Coupling_Clamp(s_targetA, -a_lim, a_lim), s_rampStep);
    s_cmdB = Coupling_SolveB(s_cmdA, s_ratio);

    if (s_closedLoop) {
        s_outA = (int)PID_Update(&s_pidA, (float)s_cmdA - Encoder_RateA(), ENCODER_DT_SEC);
        s_outB = (int)PID_Update(&s_pidB, (float)s_cmdB - Encoder_RateB(), ENCODER_DT_SEC);
    } else {
        s_outA = s_cmdA;
        s_outB = Coupling_Ramp(s_outB, s_cmdB, s_rampStep);
    }

    if (s_targetA == 0 && s_cmdA == 0 && s_cmdB == 0)
        s_state = MOTOR_STATE_IDLE;
    else
        s_state = MOTOR_STATE_RUN;

    motor_apply();
}

void Motor_Stop(void)
{
    motor_hard_stop();
    s_state = MOTOR_STATE_IDLE;
    Motor_FeedWatchdog();
}

int Motor_GetRatio(void) { return s_ratio; }
int Motor_GetSpeedA(void) { return s_outA; }
int Motor_GetSpeedB(void) { return s_outB; }
int Motor_GetCmdA(void) { return s_cmdA; }
int Motor_GetCmdB(void) { return s_cmdB; }
int Motor_GetMeasA(void) { return (int)Encoder_RateA(); }
int Motor_GetMeasB(void) { return (int)Encoder_RateB(); }
int Motor_GetState(void) { return s_state; }
int Motor_GetClosedLoop(void) { return s_closedLoop; }
