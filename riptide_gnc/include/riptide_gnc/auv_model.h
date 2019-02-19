#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;
using namespace std;

typedef Matrix<float, 6, 2> Matrix62f;
typedef Matrix<float, 9, 1> Vector9f;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 5, 1> Vector5f;

// AUV Model
// Contains information about an AUV's attributes: inertia, volume, drag, and thruster properties
class AUVModel
{
  private:
    float mass, vol, rho;
    Vector3f inertia;
    Matrix32f drag;
    vector<Vector5f> thrusters;
    Vector3f CoB; // Center of buoyancy position relative to CoM
  public:
    // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const static float GRAVITY = 9.81;     // [m/s^2]
    const static float DEFAULT_RHO = 1000; // [kg/m^3]
    AUVModel(float m, Vector3f J, float V, float fluid_rho, const Ref<const Vector3f> &cob,
             const Ref<const Matrix62f> &dragCoeffs, vector<Vector5f> &auv_thrusters);

    void DecomposeActuation(Ref<Vector6f> totalFM, const Ref<const VectorXf> &actuation);
    void DecomposeActuator(Ref<Vector6f> thrustFM, int thruster, float actuation);
    void DecomposeWeightForces(Ref<Vector6f> weightFM, float phi, float theta);

    void GetEulerYPR(Ref<Matrix3f> R, float yaw, float pitch, float roll);
    MatrixXf Sgn(const Ref<const MatrixXf> mat);
};

#endif