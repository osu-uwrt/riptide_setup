#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, Vector3f J, float V, float fluid_rho, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix32f> &dragCoeffs, vector<Vector5f> &auv_thrusters);
{
    mass = m;        // [kg]
    inertia = J;     // Assume only Jxx, Jyy, and Jzz
    vol = V;         // [m^3]
    rho = fluid_rho; // [kg/m^3]
    CoB = cob;
    dragCoeffs = drag;
    thrusters = auv_thrusters;
}

// Get vector of ALL actuation forces/moments as expressed in the body frame
Vector6f AUVModel::DecomposeActuation(const Ref<const VectorXf> &actuation)
{
    Vector6f output;
    output.setZero();

    if (actuation.rows() == thrusters.size())
        for (int i = 0; i < thrusters.size(); i++)
            output = output + AUVModel::DecomposeActuator(i, actuation(i));

    return output;
}

// Get actuator forces/moments as expressed in the body frame
// Parameters:
//      thruster = index of given thruster
//      force = force from given thruster [N]
VectorXf AUVModel::DecomposeActuator(int thruster, float actuation)
{
    VectorXf output;
    output.resize(6, 1);
    output.setZero();

    if (thruster < thrusters.size()) // Verify thruster index is valid
    {
        Vector3f r, f, tao;
        r.setZero();                          // Radius vector
        f.setZero();                          // Force vector
        tao.setZero();                        // Moment vector
        float psi = thrusters[thruster](3);   // Yaw
        float theta = thrusters[thruster](4); // Pitch

        r = thrusters[thruster].head<3>();
        f(0) = force * cos(psi) * cos(theta);
        f(1) = force * sin(psi) * cos(theta);
        f(2) = -force * sin(theta);
        tao = r.cross(f);
        output << f, t;
    }

    return output;
}

// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the body frame
// Parameters:
//      phi = roll [rad]
//      theta = pitch [rad]
VectorXf AUVModel::DecomposeWeightForces(float phi, float theta)
{
    VectorXf output;
    output.resize(6, 1);
    output.setZero();

    Vector3f w, fb, tao;
    w.setZero();   // Weight vector
    fb.setZero();  // Buoyant force vector
    tau.setZero(); // Moment vector
    float W = mass * GRAVITY;
    float B = fluidRho * vol * GRAVITY;

    w(0) = -(W - B) * sin(theta);
    w(1) = (W - B) * sin(phi) * cos(theta);
    w(2) = (W - B) * cos(phi) * cos(theta);
    fb(0) = B * sin(theta);
    fb(1) = -B * sin(phi) * cos(theta);
    fb(2) = -B * cos(phi) * cos(theta);
    tau = CoB.cross(fb);
    output << w, tau;

    return output;
}

// Get rotation matrix from world-frame to body-frame using Euler Angles (yaw, pitch, roll)
// Ex. Vb = R * Vw, Vb = vector in body-frame coordinates, Vw = vector in world-frame coordinates
Matrix3f AUVModel::GetEulerYPR(float yaw, float pitch, float roll)
{
    Matrix3f R = Matrix3f::Zero();
    float s_phi = sin(roll);
    float c_phi = cos(roll);
    float s_theta = sin(pitch);
    float c_theta = cos(pitch);
    float s_psi = sin(yaw);
    float c_psi = cos(yaw);

    R(0, 0) = c_theta * c_psi;
    R(0, 1) = c_theta * s_psi;
    R(0, 2) = -s_theta;
    R(1, 0) = s_phi * s_theta * c_psi - c_phi * s_psi;
    R(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    R(1, 2) = s_phi * c_theta;
    R(2, 0) = c_phi * s_theta * c_psi + s_phi * s_psi;
    R(2, 1) = c_phi * s_theta * s_psi - s_phi * c_psi;
    R(2, 2) = c_phi * c_theta;

    return R;
}

// Compute a priori states for Translation EKF
// 9 States: inertial pos, inertial vel, inertial accel (the last two are expressed in the body-frame)
VectorXf AUVModel::TransEKF9APriori(const Ref<const Vector3f> &posPrev, const Ref<const Vector3f> &velPrev,
                                    const Ref<const Vector3f> &attitude, const Ref<const Vector3f> &omega)
{
    VectorXf apriori;
    apriori.resize(9, 1);
    const float roll = attitude(0);
    const float pitch = attitude(1);
    const float yaw = attitude(2);
    const float uPrev = velPrev(0);
    const float vPrev = velPrev(1);
    const float wPrev = velPrev(2);
    const float p = omega(0);
    const float q = omega(1);
    const float r = omega(2);
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
Vector3f AUVModel::BodyFrameAccel(Vector3f attitude, Vector3f velBF, Vector3f angVelBF)
{
}