/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * detectPhase.cpp
 *
 * Code generation for function 'detectPhase'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "detectPhase.h"
#include "angle.h"
#include "abs.h"
#include "fft.h"
#include "mean.h"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void detectPhase(int data[1024], double *phase, double *freq_pinger)
{
  double mtmp;
  int ixstart;
  double d0;
  creal_T X[1024];
  int itmp;
  double varargin_1[512];
  int ix;
  boolean_T exitg1;

  /* sample rate, 500 kHz */
  mtmp = mean(data);
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    d0 = rt_roundd_snf((double)data[ixstart] - mtmp);
    if (d0 < 2.147483648E+9) {
      if (d0 >= -2.147483648E+9) {
        itmp = (int)d0;
      } else {
        itmp = MIN_int32_T;
      }
    } else if (d0 >= 2.147483648E+9) {
      itmp = MAX_int32_T;
    } else {
      itmp = 0;
    }

    data[ixstart] = itmp;
  }

  /* normalize data  */
  /*  Signal Verification  */
  fft(data, X);

  /* Zerodatad (Interpolation)  */
  /* Symmetrical */
  b_abs(*(creal_T (*)[512])&X[0], varargin_1);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 0;
  if (rtIsNaN(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 513)) {
      ixstart = ix;
      if (!rtIsNaN(varargin_1[ix - 1])) {
        mtmp = varargin_1[ix - 1];
        itmp = ix - 1;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 512) {
    while (ixstart + 1 < 513) {
      if (varargin_1[ixstart] > mtmp) {
        mtmp = varargin_1[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  *freq_pinger = 500.0 * ((double)(itmp + 1) - 1.0);

  /*  Angle  */
  angle(*(creal_T (*)[512])&X[0], varargin_1);
  *phase = varargin_1[itmp] * 180.0 / 3.1415926535897931;
}

/* End of code generation (detectPhase.cpp) */
