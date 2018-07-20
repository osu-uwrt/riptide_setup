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

void detectPhase(int data[2048], int startFreq, int endFreq, double *phase,
                 double *freq_pinger, double *amplitude)
{
  double d0;
  int n;
  double d1;
  creal_T X[2048];
  int ixstart;
  int trueCount;
  int X_size[2];
  short tmp_data[1024];
  creal_T X_data[1024];
  double angles_data[1024];
  int angles_size[2];
  int itmp;
  boolean_T exitg1;
  short b_tmp_data[1024];
  int b_X_size[2];
  short c_tmp_data[1024];

  /* sample rate, 500 kHz */
  d0 = mean(data);
  for (n = 0; n < 2048; n++) {
    d1 = rt_roundd_snf((double)data[n] - d0);
    if (d1 < 2.147483648E+9) {
      if (d1 >= -2.147483648E+9) {
        ixstart = (int)d1;
      } else {
        ixstart = MIN_int32_T;
      }
    } else if (d1 >= 2.147483648E+9) {
      ixstart = MAX_int32_T;
    } else {
      ixstart = 0;
    }

    data[n] = ixstart;
  }

  /* normalize data  */
  /*  Signal Verification  */
  fft(data, X);

  /* Zerodatad (Interpolation)  */
  /* Symmetrical */
  trueCount = 0;
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    if ((250 * ixstart >= startFreq) && (250 * ixstart <= endFreq)) {
      trueCount++;
    }
  }

  n = 0;
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    if ((250 * ixstart >= startFreq) && (250 * ixstart <= endFreq)) {
      tmp_data[n] = (short)(ixstart + 1);
      n++;
    }
  }

  X_size[0] = 1;
  X_size[1] = trueCount;
  for (n = 0; n < trueCount; n++) {
    X_data[n] = X[tmp_data[n] - 1];
  }

  b_abs(X_data, X_size, angles_data, angles_size);
  ixstart = 1;
  n = angles_size[1];
  *amplitude = angles_data[0];
  itmp = 0;
  if (angles_size[1] > 1) {
    if (rtIsNaN(angles_data[0])) {
      trueCount = 1;
      exitg1 = false;
      while ((!exitg1) && (trueCount + 1 <= n)) {
        ixstart = trueCount + 1;
        if (!rtIsNaN(angles_data[trueCount])) {
          *amplitude = angles_data[trueCount];
          itmp = trueCount;
          exitg1 = true;
        } else {
          trueCount++;
        }
      }
    }

    if (ixstart < angles_size[1]) {
      while (ixstart + 1 <= n) {
        if (angles_data[ixstart] > *amplitude) {
          *amplitude = angles_data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  n = 0;
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    if ((250 * ixstart >= startFreq) && (250 * ixstart <= endFreq)) {
      b_tmp_data[n] = (short)(ixstart + 1);
      n++;
    }
  }

  *freq_pinger = 250.0 * ((double)b_tmp_data[itmp] - 1.0);

  /*  Angle  */
  trueCount = 0;
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    if ((250 * ixstart >= startFreq) && (250 * ixstart <= endFreq)) {
      trueCount++;
    }
  }

  n = 0;
  for (ixstart = 0; ixstart < 1024; ixstart++) {
    if ((250 * ixstart >= startFreq) && (250 * ixstart <= endFreq)) {
      c_tmp_data[n] = (short)(ixstart + 1);
      n++;
    }
  }

  b_X_size[0] = 1;
  b_X_size[1] = trueCount;
  for (n = 0; n < trueCount; n++) {
    X_data[n] = X[c_tmp_data[n] - 1];
  }

  angle(X_data, b_X_size, angles_data, angles_size);
  *phase = angles_data[itmp] * 180.0 / 3.1415926535897931;
}

/* End of code generation (detectPhase.cpp) */
