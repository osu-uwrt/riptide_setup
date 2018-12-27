#ifndef ATTITUDE_EDKF
#define ATTITUDE_EDKF

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "Math.h"

using namespace Eigen;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Vector<float, 6, 1> Vector6f;

float sec(x)
{
  return 1.0 / cos(x);
}

// Attitude Extended Dynamic Kalman Filter (EDKF)
// This EDKF can be used as a cascaded KF, in which an IMU's EKF output is treated as the measurement
// OR this EDKF can simply use angular rate from a rate gyro as a measurement input.
// Using the cascade feature can lead to more accurate results.
class AttitudeEDKF
{
public:
  // 1 -> Angular Motion (ang. vel and accel.); 2 -> Attitude (ang. pos. and ang. pos. rates)

  KalmanFilter *AngMotKF, AttKF;
  Vector6f X1hat, X1hatPre, X2hat, X2hatPre;          // State vectors
  int x, y, z;                                        // Helper indeces for calculating partial derivatives
  float p, q, r, p_dot, q_dot, r_dot;                 // Helper variables for calculating partial derivatives
  float phi, theta, psi, phi_dot, theta_dot, psi_dot; // More helper variables
  float c1, c2, c3, c4;                               // Common expressions for calculating partial derivatives
  Matrix6f F1, F2;                                    // State-transition matrices
  Matrix<float, 3, 6> H1;                             // Measurement matrix for angular motion
  MatrixXf H2;                                        // Measurement matrix for attitide
  bool cascadeInit;                                   // Turn on/off cascade feature
  Vector3f J;                                         // Inertia Tensor (Assume Jxx, Jyy, and Jzz, no need for cross-terms)
  Vector3f b;                                         // Damping Coefficients
  VectorXf<float, 1, 36> pd;                          // Partial Derivatives (36 total)
  float dt;                                           // Time-step [s]
  int s;                                              // s = 3, half the number of states

private:
  AttitudeEDKF(bool cascade, Vector3f inertia, Vector3f damping,
               Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2);
  void UpdateAttEDKF(float time_step, Vector3f input_states, VectorXf Z); // Input state: p_dot, q_dot, r_dot
  void TimePredictAngMotState(Vector3f input_states);                     // Calculate X1a
  void TimePredictAttState();                                             // Calculate X2a
  void KeepAnglesInRange();
  void CalcPartialDerivatives();
  void CalcAngMotJacobians(); // Update F1 and H1
  void CalcAttJacobians();    // Update F2 and H2
};

#endif