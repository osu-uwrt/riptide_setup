#include "riptide_gnc/pose_ekf.h"

PoseEKF::PoseEKF(Matrix32i posAvail, Matrix32i velAvail, Matrix32i accelAvail)
{
    // Find all combinations of measurements
    
}

void PoseEKF::InitPoseEKF(VectorXf Xo)
{

}

void PoseEKF::UpdatePoseEKF(float time_step, VectorXf Z, Vector3f attitude)
{

}

// Get rotation matrix from world-frame to body-frame using Euler Angles (roll, pitch, yaw)
Matrix3f PoseEKF::GetRotationRPY2Body(float roll, float pitch, float yaw)
{
    Matrix3f R = Matrix3f::Zero();
    float s_phi = sin(roll);
    float c_phi = cos(roll);
    float s_theta = sin(pitch);
    float c_theta = cos(pitch);
    float s_psi = sin(yaw);
    float c_psi = cos(yaw);

    R(0, 0) = c_theta * c_psi;
    R(0, 1) = c_theta * s_psi;
    R(0, 2) = -s_theta;
    R(1, 0) = s_phi * s_theta * c_psi - c_phi * s_psi;
    R(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    R(1, 2) = s_phi * c_theta;
    R(2, 0) = c_phi * s_theta * c_psi + s_phi * s_psi;
    R(2, 1) = c_phi * s_theta * s_psi - s_phi * c_psi;
    R(2, 2) = c_phi * c_theta;

    return R;
}