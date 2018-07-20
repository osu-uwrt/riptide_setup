/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.cpp
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "detectPhase.h"
#include "abs.h"

/* Function Declarations */
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

void b_abs(const creal_T x_data[], const int x_size[2], double y_data[], int
           y_size[2])
{
  int k;
  y_size[0] = 1;
  y_size[1] = (short)x_size[1];
  for (k = 0; k + 1 <= x_size[1]; k++) {
    y_data[k] = rt_hypotd_snf(x_data[k].re, x_data[k].im);
  }
}

/* End of code generation (abs.cpp) */
