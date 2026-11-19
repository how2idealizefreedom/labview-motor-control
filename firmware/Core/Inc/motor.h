/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Dual DC motor driver with fixed-ratio linear speed coupling.
  *          Motor A follows the commanded speed; Motor B = k * Motor A.
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- Hardware mapping (STM32F103C8T6, existing LabVIEW motor board) ----
 * Motor A: PWM = TIM3_CH1 (PA6); direction = PB12 (IN1), PB13 (IN2)
 * Motor B: PWM = TIM3_CH4 (PB1); direction = PB14 (IN1), PB15 (IN2)
 * TIM3 ARR = 9999, PSC = 71  ->  10 kHz PWM on 72 MHz.
 * A signed duty in [-1000, +1000] is mapped to compare and direction.
 * ----------------------------------------------------------------------- */
#define MOTOR_PWM_ARR        10000u
#define MOTOR_SPEED_MAX      1000     /* command saturation, per mille */

/* Default coupling ratio: vB = 0.5 * vA (stored as per-mille integer). */
#define MOTOR_RATIO_DEFAULT  500      /* 500/1000 = 0.5 */

void     Motor_Init(void);
void     Motor_SetSpeedA(int speed_a);          /* -1000..1000, sets B = k*A */
void     Motor_SetRatio(int ratio_permille);    /* e.g. 500 -> 0.5, 0..1000 */
int      Motor_GetRatio(void);
int      Motor_GetSpeedA(void);
int      Motor_GetSpeedB(void);
void     Motor_Tick(void);                      /* call every 5 ms: ramp + output */
void     Motor_Stop(void);                      /* immediate brake both motors */
void     Motor_SetRampStep(int step);           /* per-mille per tick, 0 = off */

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H */
