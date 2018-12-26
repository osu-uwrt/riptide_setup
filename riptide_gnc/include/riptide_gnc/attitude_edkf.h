#ifndef ATTITUDE_EDKF
#define ATTITUDE_EDKF

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Vector<float, 6, 1> Vector6f;

// Attitude Extended Dynamic Kalman Filter (EDKF)
// This EDKF can be used as a cascaded KF, in which an IMU's EKF output is treated as the measurement
// OR this EDKF can simply use angular rate from a rate gyro as a measurement input.
// Using the cascade feature can lead to more accurate results.
class AttitudeEDKF
{
  public:
    // 1 -> Angular Motion (ang. vel and accel.); 2 -> Attitude (ang. pos. and ang. pos. rates)
    // a -> a priori (time-predict step); b -> a posteriori (measurement_update step)

    KalmanFilter *AngMotKF, AttKF;
    Vector6f Xhat1a, Xhat1b, Xhat2a, Xhat2b; // State vectors
    Matrix6f F1, F2; // State-transition matrices
    Matrix<float, 3, 6> H1; // Measurement matrix for angular motion
    MatrixXf H2; // Measurement matrix for attitide
    bool cascadeInit; // Turn on/off cascade feature
  private:
    AttitudeEDKF(bool cascade, Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2);
    void UpdateAttEDKF(Vector3f input_states); // Input state: p_dot, q_dot, r_dot
    void UpdateMotionState(Vector3f input_states);
    void UpdateAttitudeState();
    void TimePredictMotionState();   // Calculate X1a
    void TimePredictAttitudeState(); // Calculate X2a
    void CalcMotionJacobians();      // Update F1 and H1
    void CalcAttitudeJacobians();    // Update F2 and H2
};

#endif