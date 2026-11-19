/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Quadrature encoder readout for the two motors (TIM2 / TIM4).
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Set to -1 if the encoder counts backwards on the real board. */
#define ENCODER_A_DIR      1
#define ENCODER_B_DIR      1

/* Counts per second at full speed. Calibrate this on the real board. */
#define ENCODER_FULL_CPS   1000.0f

/* Encoder sampling period, must match the motor tick (seconds). */
#define ENCODER_DT_SEC     0.005f

void  Encoder_Init(void);
void  Encoder_Reset(void);
void  Encoder_Sample(void);
float Encoder_RateA(void);     /* signed, per mille of full speed */
float Encoder_RateB(void);
int   Encoder_CountA(void);    /* accumulated counts */
int   Encoder_CountB(void);

#ifdef __cplusplus
}
#endif
#endif /* __ENCODER_H */
