#ifndef POSE_EDKF
#define POSE_EDKF

#include "riptide_gnc/kalman_filter.h"
#include "riptide_gnc/auv_math_lib.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 1, Dynamic> RowXi;
typedef Matrix<float, 3, 2> Matrix32f;
typedef Matrix<float, 9, 9> Matrix9Xf;

// Translational Extended Kalman Filter
// This class is designed to estimate a vehicle's body-frame linear velocity and acceleration from a
// series of sensors running at different rates (which is most probable in practice). Based on the
// indicated vehicle's on-board sensors, this class will determine all possible combinations of sensory
// data and will automatically determine which EKF to run based on the new sensor data provided.
// NOTE: This class assumes at least one sensor input is provided
// NOTE: For practical reasons, at most TWO of the same type of sensor may be provided
class TransEKF
{
private:
  vector<KalmanFilter> EKF;
  int n, numEKF;
  vector<Matrix3Xi> sensorMask;
  vector<Matrix3Xf> Rmat;
  RowXi maskCols;
  Matrix9Xf Q;
  Matrix32f dragCoeffs;

  bool sensorsExist;
  bool sensors[3], sensorsRed[3]; // Red = redundant
  bool indexRed[3];
  int axialData[3], axialDataRed[3], numMsmts[2];

  vector<RowXi> sensorCombosBF, sensorCombosW;

public:
  TransEKF(Matrix3Xi posMaskw, Matrix3Xi velMaskbf, Matrix3Xi accelMaskbf,
           Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix9Xf Qin);
  void FindDataCombos(vector<RowXi> &list, int numSensors1, int numSensors2);
  void InitLMEDKF(VectorXf Xo);
  MatrixX3f UpdateEKF(float time_step, Vector3f input_states, Matrix3Xf Zpos, Matrix3Xf Zvel, Matrix3Xf Zaccel);
};

#endif