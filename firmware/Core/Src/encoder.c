/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Quadrature encoder readout. TIM2 = motor A, TIM4 = motor B.
  *          Both timers are configured in Cub(e)MX as encoder mode.
  ******************************************************************************
  */
#include "encoder.h"
#include "tim.h"
#include "coupling.h"

static int16_t s_prevA, s_prevB;
static int32_t s_countA, s_countB;
static float   s_rateA, s_rateB;

void Encoder_Init(void)
{
    MX_TIM2_Init();
    MX_TIM4_Init();
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    s_prevA = (int16_t)TIM2->CNT;
    s_prevB = (int16_t)TIM4->CNT;
    s_countA = 0;
    s_countB = 0;
    s_rateA = 0.0f;
    s_rateB = 0.0f;
}

void Encoder_Reset(void)
{
    s_countA = 0;
    s_countB = 0;
    s_rateA = 0.0f;
    s_rateB = 0.0f;
}

void Encoder_Sample(void)
{
    int16_t nowA = (int16_t)TIM2->CNT;
    int16_t nowB = (int16_t)TIM4->CNT;
    int16_t dA = (int16_t)(nowA - s_prevA);
    int16_t dB = (int16_t)(nowB - s_prevB);
    float cpsA, cpsB, perMilleA, perMilleB;

    s_prevA = nowA;
    s_prevB = nowB;
    s_countA += dA * ENCODER_A_DIR;
    s_countB += dB * ENCODER_B_DIR;

    cpsA = (float)(dA * ENCODER_A_DIR) / ENCODER_DT_SEC;
    cpsB = (float)(dB * ENCODER_B_DIR) / ENCODER_DT_SEC;
    perMilleA = cpsA * 1000.0f / ENCODER_FULL_CPS;
    perMilleB = cpsB * 1000.0f / ENCODER_FULL_CPS;

    /* First-order filter, 20% new sample per 5 ms tick. */
    s_rateA += 0.2f * (perMilleA - s_rateA);
    s_rateB += 0.2f * (perMilleB - s_rateB);
    s_rateA = (float)Coupling_Clamp((int)s_rateA, -COUPLING_SPEED_MAX, COUPLING_SPEED_MAX);
    s_rateB = (float)Coupling_Clamp((int)s_rateB, -COUPLING_SPEED_MAX, COUPLING_SPEED_MAX);
}

float Encoder_RateA(void) { return s_rateA; }
float Encoder_RateB(void) { return s_rateB; }
int   Encoder_CountA(void) { return s_countA; }
int   Encoder_CountB(void) { return s_countB; }
