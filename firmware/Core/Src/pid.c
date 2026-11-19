/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Discrete PI controller. Integral term is clamped to the output
  *          range so the integrator cannot wind up while the plant saturates.
  ******************************************************************************
  */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->out_min = out_min;
    p->out_max = out_max;
    p->integ = 0.0f;
}

void PID_Reset(PID_t *p)
{
    p->integ = 0.0f;
}

float PID_Update(PID_t *p, float error, float dt)
{
    float out;
    float ilim = (p->ki > 0.0f) ? (p->out_max / p->ki) : p->out_max;

    p->integ += error * dt;
    if (p->integ > ilim) p->integ = ilim;
    if (p->integ < -ilim) p->integ = -ilim;

    out = p->kp * error + p->ki * p->integ;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}
