/**
  ******************************************************************************
  * @file    coupling.h
  * @brief   Motor speed coupling math, no hardware dependencies.
  *
  *          sin(v1) = k * cos(v2)
  *          v1 =  a      * pi / 2000
  *          v2 = (1000-b) * pi / 2000
  ******************************************************************************
  */
#ifndef __COUPLING_H
#define __COUPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#define COUPLING_SPEED_MAX   1000

int Coupling_Clamp(int v, int lo, int hi);
int Coupling_Ramp(int cur, int tgt, int step);

/* Max |a| for which sin(v1) = k*cos(v2) has a real solution. */
int Coupling_ALimit(int k_permille);

/* Solve motor B command from motor A command and k. */
int Coupling_SolveB(int a, int k_permille);

#ifdef __cplusplus
}
#endif
#endif /* __COUPLING_H */
