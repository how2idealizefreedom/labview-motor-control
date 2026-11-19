/**
  ******************************************************************************
  * @file    coupling.c
  * @brief   Speed coupling math. Pure C, no MCU headers.
  ******************************************************************************
  */
#include "coupling.h"
#include <math.h>

#define PI_OVER_2000         0.00157079632679f
#define TWO_K_OVER_PI        636.61977236758f

int Coupling_Clamp(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int Coupling_Ramp(int cur, int tgt, int step)
{
    if (step <= 0) return tgt;
    if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

int Coupling_ALimit(int k_permille)
{
    float k = (float)k_permille / 1000.0f;
    if (k < 0.001f) return 0;
    return (int)(TWO_K_OVER_PI * asinf(k));
}

int Coupling_SolveB(int a, int k_permille)
{
    float k = (float)k_permille / 1000.0f;
    float c;

    if (k < 0.001f) return 0;
    c = sinf((float)a * PI_OVER_2000) / k;
    if (c > 1.0f)  c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return (int)(1000.0f - acosf(c) * TWO_K_OVER_PI);
}
