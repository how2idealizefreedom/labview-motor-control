/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Dual DC motor driver with sine/cosine coupling, encoder feedback,
  *          command watchdog and an optional speed closed loop.
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MOTOR_PWM_ARR        10000u
#define MOTOR_SPEED_MAX      1000
#define MOTOR_RATIO_DEFAULT  1000      /* k = 1.0 */
#define MOTOR_CMD_TIMEOUT_MS 500u      /* no command -> brake */

#define MOTOR_STATE_IDLE     0
#define MOTOR_STATE_RUN      1
#define MOTOR_STATE_TIMEOUT  2

void Motor_Init(void);
void Motor_SetSpeedA(int speed_a);
void Motor_SetRatio(int ratio_permille);
void Motor_SetRampStep(int step);
void Motor_SetClosedLoop(int on);
void Motor_FeedWatchdog(void);
void Motor_Tick(void);
void Motor_Stop(void);

int  Motor_GetRatio(void);
int  Motor_GetSpeedA(void);
int  Motor_GetSpeedB(void);
int  Motor_GetCmdA(void);
int  Motor_GetCmdB(void);
int  Motor_GetMeasA(void);
int  Motor_GetMeasB(void);
int  Motor_GetState(void);
int  Motor_GetClosedLoop(void);

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H */
