#ifndef POSE_EKF
#define POSE_EKF

#include "riptide_gnc/linear_motion_ekf_suite.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace std;

// Pose Extended Kalman Filter
// This class is designed to estimate a vehicle's state from a series of sensors running at different
// rates (which is most probable in practice). Based on the indicated vehicle's on-board sensors
// this class will determine all possible combinations of sensory data and will automatically
// determine which EKF to run based on the combination of new sensor data provided.
class PoseEKF
{
  private:
    vector<LinearMotionEKFSuite> EKFSuite;
    Matrix3Xi posMask;     // Available position data
    Matrix3Xi velMask;   // Available velocity data
    Matrix3Xi accelMask; // Available acceleration data

  public:
    PoseEKF(Matrix3Xi posIn, Matrix3Xi bfVelIn, Matrix3Xi bfAccelIn);
    void InitPoseEKF(VectorXf Xo);
    void UpdatePoseEKF(float time_step, Vector3f attitude, Matrix3Xf Zpos, Matrix3Xf Zvel, Matrix3Xf Zaccel);
    Matrix3f GetRotationRPY2Body(float roll, float pitch, float yaw)
};

#endif