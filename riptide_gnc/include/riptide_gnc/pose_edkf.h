#ifndef POSE_EDKF
#define POSE_EDKF

#include "riptide_gnc/pose_ekf_suite.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 3, 2> Matrix32i;

// Pose Extended Dynamic Kalman Filter
// This class is designed to estimate a vehicle's state from a series of sensors running at different 
// rates (which is most probable in practice). Based on the indicated vehicle's on-board sensors
// this class will determine all possible combinations of sensory data and will automatically 
// determine which EKF to run based on the combination of new sensor data provided.
class PoseEDKF
{
private:
    vector<PoseEKFSuite> poseEKFSuite;
    int EKFindeces;
    float dt;
public:
    PoseEDKF(Matrix32i posAvail, Matrix32i velAvail, Matrix32i accelAvail);
    void InitPoseEDKF(VectorXf Xo);
    void UpdatePoseEDKF(float time_step, VectorXf Z, Vector3f attitude);
};

#endif