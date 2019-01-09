#ifndef POSE_EDKF
#define POSE_EDKF

#include "riptide_gnc/linear_motion_ekf_suite.h"
#include "riptide_gnc/pose_ekf.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 1, Dynamic> RowXi;
typedef Matrix<float, 2, 3> Matrix23f;

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
  vector<LinearMotionEKFSuite> PoseEKFSuite;

  Matrix3Xi velMaskbf;   // Available body-frame velocity data
  Matrix3Xi accelMaskbf; // Available body-frame acceleration data
  Matrix3Xi posMaskw;    // Available world-frame position data
  Matrix3Xi vaMaskw;     // Available world-frame velocity and acceleraton data

  int velSensors, accelSensors;
  vector<RowXi> sensorCombos;
  int dragOrder;
  Vector3f linearizedDamping;

public:
  const int DRAG_ORDER_1; // 1st order polynomial model for drag/viscous damping
  const int DRAG_ORDER_2; // 2nd order polynomial for drag/viscous damping

  LinearMotionEDKF(int drag_order, Vector3f damping, Matrix3Xi posIn, Matrix3Xi velIn, Matrix3Xi accelIn, Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix3Xf Q);
  void FindDataCombos(vector<RowXi> &list, int numSensors1, int numSensors2);
  void InitLMEDKF(VectorXf Xo);
  MatrixX3f UpdateLMEDKF(RowXi dataMask, float time_step, Vector3f input_states, Matrix3Xf Zpos, Matrix3Xf Zvel, Matrix3Xf Zaccel);
  Matrix2f VelAccelSTM(float dampingSlope, float dt);
  Matrix3f GetRotationRPY2Body(float roll, float pitch, float yaw); // World to body rotation matrix
};

#endif