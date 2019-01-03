#ifndef POSE_EDKF
#define POSE_EDKF

#include "riptide_gnc/linear_motion_ekf_suite.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 1, Dynamic> RowXi;
typedef Matrix<float, 2, Dynamic> Matrix2Xf;

// Linear Motion Extended Dynamic Kalman Filter
// This class is designed to estimate a vehicle's body-frame linear velocity and acceleration from a 
// series of sensors running at different rates (which is most probable in practice). Based on the 
// indicated vehicle's on-board sensors, this class will determine all possible combinations of sensory
// data and will automatically determine which EKF to run based on the new sensor data provided.
// NOTE: This class assumes at least one sensor input is provided
// NOTE: For practical reasons, at most TWO of the same type of sensor may be provided
class LinearMotionEDKF
{
  private:
    vector<LinearMotionEKFSuite> LMEKFSuite;
    Matrix3Xi velData;   // Available body-frame velocity data
    Matrix3Xi accelData; // Available body-frame acceleration data
    int velSensors, accelSensors;
    vector<RowXi > sensorCombos;

  public:
    LinearMotionEDKF(Matrix3Xi velIn, Matrix3Xi accelIn, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix2Xf Q);
    void FindDataCombos(int velSensors, int accelSensors);
    void InitLMEDKF(VectorXf Xo);
    void UpdateLMEDKF(float time_step, Matrix3Xf Zvel, Matrix3Xf Zaccel);
};

#endif