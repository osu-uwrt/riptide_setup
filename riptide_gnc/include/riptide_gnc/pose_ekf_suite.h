#ifndef POSE_EKF_SUITE
#define POSE_EKF_SUITE

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "Math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<float, 6, 1> Vector6f;

// Pose Extended Kalman Filter (EKF) Suite
// This class contains a suite of EKFs designed to estimate a vehicle's state from a series of
// sensors running at different rates (which is most probable in practice).
// This class will automatically determine which EKF to run based on what new sensor data was
// provided, b/c it is not a good idea to run old data in a Kalman Filter
class PoseEKFSuite
{
private:
  vector<KalmanFilter> KFSuite;
  Matrix3i dataMask;
  bool needKF[3]; // Indicate if there is an X, Y, and/or Z KF
  bool init;     // Indicates if EKF is initilized

public:
  PoseEKFSuite(Matrix3i dataAvail, Vector3f Rpos, Vector3f Rvel, Vector3f Raccel,
               Matrix3f Qx, Matrix3f Qy, Matrix3f Qz);
  void UpdatePoseEKF(Vector3f Xpredict, Matrix3f data, Matrix3f Anew, Vector3f attitude);
  void UpdateEKFX();
  void UpdateEKFY();
  void UpdateEKFZ();
  void CalcMsmtJacobian(); // Calculate measurement Jacobian

  // Getter Methods
};

#endif