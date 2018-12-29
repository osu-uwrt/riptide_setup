#ifndef POSE_EDKF
#define POSE_EDKF

#include "riptide_gnc/pose_ekf_suite.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace std;

// Pose Extended Dynamic Kalman Filter
class PoseEDKF
{
private:
    vector<PoseEKFSuite> poseEKFSuite;
    bool init;
public:
    PoseEDKF();
    void InitPoseEDKF(VectorXf Xo);
    void UpdatePoseEDKF(VectorXf Z);
};

#endif