#ifndef ATTITUDE_EDKF
#define ATTITUDE_EDKF

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<float, 6, 1> Vector6f;

// Attitude Extended Dynamic Kalman Filter (EDKF)
// This EDKF is meant to be used as a cascaded KF, in which an IMU's EKF output is treated as the measurement
// The expected input measurement from the IMU EKF is (phi, theta, psi, p, q, r).
// Because this EDKF does not use quaternions, it is still subject to the singularity issue at
// pitch angles of +/- 90 degrees. To ensure complete operation of this EDKF, when the IMU EKF reports
// pitch angles outside the specified range, this EDKF will bypass the calculations and simply report
// the same values from the IMU EKF.
// The EDKF also performs auto-intialization (no need to directly initialize)
class AttitudeEDKF
{
private:
  // 1 -> Angular Motion (ang. vel and accel.); 2 -> Attitude (ang. pos. and ang. pos. rates)

  KalmanFilter *AngMotKF, *AttKF;
  Vector6f X1hat, X1hatPre, X2hat, X2hatPre;          // State vectors
  int x, y, z;                                        // Helper indeces for calculating partial derivatives
  float p, q, r, p_dot, q_dot, r_dot;                 // Helper variables for calculating partial derivatives
  float phi, theta, psi, phi_dot, theta_dot, psi_dot; // More helper variables
  float c1, c2, c3, c4;                               // Common expressions for calculating partial derivatives
  Matrix6f F1, F2;                                    // State-transition matrices
  Matrix<float, 3, 6> H1;                             // Measurement matrix for angular motion
  Matrix6f H2;                                        // Measurement matrix for attitide
  bool init;                                          // Indicates if EDKF is initialized
  Vector3f J;                                         // Inertia Tensor (Assume Jxx, Jyy, and Jzz, no need for cross-terms)
  Vector3f b;                                         // Damping Coefficients
  Matrix<float, 1, 36> pd;                          // Partial Derivatives (36 total)
  float dt;                                           // Time-step [s]
  int s;                                              // s = 3, half the number of states
  float max_theta;                                    // Maximum pitch angle allowed for operation of EDKF [rad]

public:
  AttitudeEDKF(float max_pitch, Vector3f inertia, Vector3f damping,
               Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2); // max_pitch [rad]
  void InitAttEDKF(Vector3f input_states, Vector6f Z);
  void UpdateAttEDKF(float time_step, Vector3f input_states, Vector6f Z); // Input state: p_dot, q_dot, r_dot
  void UpdateAngMotStates(Vector3f input_states, Vector6f Z);
  void UpdateAttStates(Vector6f Z);
  void TimePredictAngMotState(Vector3f input_states);
  void TimePredictAttState();
  void CalcPartialDerivatives();
  void CalcAngMotJacobians(); // Update F1 and H1
  void CalcAttJacobians();    // Update F2 and H2

  float KeepAngleWithinPI(float angle);
  float KeepMsmtWithinPI(float predict, float msmt);

  // Getter Methods
  Vector6f GetAngularMotionXhat();
  Vector6f GetAttitudeXhat();
};

#endif