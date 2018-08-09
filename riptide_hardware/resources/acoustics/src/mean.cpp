/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mean.cpp
 *
 * Code generation for function 'mean'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "detectPhase.h"
#include "mean.h"

/* Function Definitions */
double mean(const int x[2048])
{
  double y;
  int k;
  y = x[0];
  for (k = 0; k < 2047; k++) {
    y += (double)x[k + 1];
  }

  y /= 2048.0;
  return y;
}

/* End of code generation (mean.cpp) */
