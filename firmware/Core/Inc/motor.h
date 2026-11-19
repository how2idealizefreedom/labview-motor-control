/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Dual DC motor driver with sine/cosine speed coupling.
  *
  *          v1 = a * pi / 2000          (a = motor A command, per mille)
  *          v2 = (1000 - b) * pi / 2000 (b = motor B command, per mille)
  *          The driver keeps:  sin(v1) = k * cos(v2)
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* STM32F103C8T6, existing LabVIEW motor board:
 * Motor A: PWM = TIM3_CH1 (PA6); direction = PB12 (IN1), PB13 (IN2)
 * Motor B: PWM = TIM3_CH4 (PB1); direction = PB14 (IN1), PB15 (IN2)
 * TIM3 ARR = 9999, PSC = 71 -> 10 kHz PWM on 72 MHz. */
#define MOTOR_PWM_ARR        10000u
#define MOTOR_SPEED_MAX      1000     /* command saturation, per mille */

/* k = 1.0 by default. With k=1 the two motors run at the same speed. */
#define MOTOR_RATIO_DEFAULT  1000      /* 1000/1000 = 1.0 */

void     Motor_Init(void);
void     Motor_SetSpeedA(int speed_a);          /* -1000..1000, per mille */
void     Motor_SetRatio(int ratio_permille);    /* 1..1000 -> k = 0.001..1.0 */
int      Motor_GetRatio(void);
int      Motor_GetSpeedA(void);
int      Motor_GetSpeedB(void);
void     Motor_Tick(void);                      /* call every 5 ms */
void     Motor_Stop(void);
void     Motor_SetRampStep(int step);           /* per mille per tick, 0 = off */

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H */
