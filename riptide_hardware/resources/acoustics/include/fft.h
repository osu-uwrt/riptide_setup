/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fft.h
 *
 * Code generation for function 'fft'
 *
 */

#ifndef FFT_H
#define FFT_H

/* Include files */
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "detectPhase_types.h"

/* Function Declarations */
extern void fft(const int x[1024], creal_T y[1024]);

#endif

/* End of code generation (fft.h) */
