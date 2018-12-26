#include "riptide_gnc/attitude_edkf.h"

AttitudeEDKF::AttitudeEDKF(bool cascade, Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2)
{
    cascadeInit = cascade;
    if(cascadeInit) // Have access to both attitude and angular velocity
    {
        H2.resize(6,6);
        H2.setIdentity();
    }
    else
    {   // Only have access to angular velocity
        H2.resize(3,6);
        H2.setZero();
    }

    F1.setIdentity();
    F2.setIdentity();
    H1.setZero();
    AngMotKF = new KalmanFilter(F1, H1, Q1, R1);
    AttKF = new KalmanFilter(F2, H2, Q2, R2);
}

// Update EDKF
void AttitudeEDKF::UpdateAttEDKF(Vector3f input_states)
{
    AttitudeEDKF::UpdateMotionState(input_states);
    AttitudeEDKF::UpdateAttitudeState();
}

// Update Motion State
void AttitudeEDKF::UpdateMotionState(Vector3f input_states)
{
    for (int i = 0; i < 3; i++) // Update: p_dot, q_dot, r_dot
    {
        X1a(i + 3) = input_states(i);
    }
    AttitudeEDKF::TimePredictMotionState();
}

// Update Attitude State
void AttitudeEDKF::UpdateAttitudeState()
{
}

// Time-predict Step for Angular Motion States
// Constant Acceleration Model: Xk = Xk + Xk_dot*dt
void AttitudeEDKF::TimePredictMotionState()
{
    for (int i = 0; i < 3; i++) // Only update: p, q, r
    {
        X1a(i) = X1b(i) + X1b(i + 3) * dt;
    }
}

// Time-predict Step for Attitude States
// Constant Acceleration Model: Xk = Xk + Xk_dot*dt + (1/2)*xk_ddot*dt^2
void AttitudeEDKF::TimePredictAttitudeState()
{
}

void AttitudeEDKF::CalcMotionJacobians()
{
}

void AttitudeEDKF::CalcAttitudeJacobians()
{
}
