#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;
using namespace std;

typedef Matrix<float, 3, 2> Matrix32f;
typedef Matrix<float, 6, 1> Vector6f;

// AUV Model
// Contains information about an AUV's attributes: inertia, volume, drag, and thruster properties
class AUVModel
{
private:
    float mass, volume;
    Matrix32f dragCoeffs;
    vector<float[5]> thrusters;
public:
    const static float GRAVITY = 9.81; // [m/s^2]
    const static float RHO = 1000; // [kg/m^3]
    AUVModel(float m, float V, Matrix32f drag, vector<float[5]> auv_thrusters);
    Vector6f DecomposeActuation(VectorXf actuation);
    VectorXf DecomposeActuator(int thruster, float actuation);
};

#endif