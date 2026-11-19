/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Discrete PI controller with output clamping and anti-windup.
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float out_min;
    float out_max;
    float integ;
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float out_min, float out_max);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float error, float dt);

#ifdef __cplusplus
}
#endif
#endif /* __PID_H */
