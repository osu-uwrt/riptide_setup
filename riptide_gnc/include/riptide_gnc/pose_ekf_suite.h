#ifndef POSE_EKF_SUITE
#define POSE_EKF_SUITE

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "Math.h"

using namespace Eigen;
using namespace std;
typedef Vector<float, 6, 1> Vector6f;

// Pose Extended Kalman Filter (EKF) Suite
// This class contains a suite of EKFs designed to estimate a vehicle's state from a series of
// sensors running at different rates (which is most probable in practice).
// This class will automatically determine which EKF to run based on what new sensor data was
// provided, b/c it is not a good idea to run old data in a Kalman Filter
class PoseEKFSuite
{
  private:
    vector<KalmanFilter> EKFSuite;
    init; // Indicates if EKF is initilized

  public:
    PoseEKFSuite(Vector3f pos, Vector3f vel, Vector3f accel,
                 Vector3f Rpos, Vector3f Rvel, Vector3f Raccel);

    void Update PoseEKF(Vector3f newData, Vector3f posData, Vector3f velData, Vector3f accelData);
    void InitPoseEKF(Vector3f input_states);
    void CalcJacobians();
    // Getter Methods
};

#endif