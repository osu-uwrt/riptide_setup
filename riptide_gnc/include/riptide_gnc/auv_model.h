#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "riptide_gnc/auv_math_lib.h"

using namespace Eigen;
using namespace std;
using namespace AUVMathLib;

typedef Matrix<float, 12, 12> Matrix12f;
typedef Matrix<float, 6, 2> Matrix62f;
typedef Matrix<float, 9, 9> Matrix9f;
typedef Matrix<float, 6, Dynamic> Matrix6xf;
typedef Matrix<float, 5, Dynamic> Matrix5xf;

typedef Matrix<float, 12, 1> Vector12f;
typedef Matrix<float, 9, 1> Vector9f;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 5, 1> Vector5f;

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
private:
  float mass, vol, rho, Fg, Fb;
  int numThrusters;
  Matrix3f inertia;
  Matrix32f drag;
  Matrix5xf thrusters;
  Matrix6xf thrustCoeffs;
  Vector3f CoB; // Center of buoyancy position relative to CoM

public:
  // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const static float PI = 3.141592653;
  const static float GRAVITY = 9.80665;    // [m/s^2]
  const static float WATER_DENSITY = 1000; // [kg/m^3]

  AUVModel(float m, float V, float fluid_rho, const ref<const Matrix3f> &J, const Ref<const Vector3f> &cob,
           const Ref<const Matrix62f> &dragCoeffs, const Ref<const Matrix5xf> &auv_thrusters);

  void SetThrustCoeffs();
  Vector6f GetTotalThrustLoad(const Ref<const VectorXf> &thrusts);
  Vector6f GetWeightLoad(float phi, float theta);

  Vector9f GetTransEKFTwoStageAPriori(const Ref<const Vector9f> &xPrev, const Ref<const Vector3f> &attitude,
                                      const Ref<const Vector3f> &pqr, const Ref<const VectorXf> &thrusts,
                                      float dt, int maxIter);
  Matrix9f GetTransEKFTwoStageJacobianA(const Ref<const Vector9f> &apriori, const Ref<const Vector3f> &attitude,
                                       const Ref<const Vector3f> &pqr, float dt);

  Matrix12f GetLQRJacobianA(const Ref<const Matrix12f> &states);
};

#endif