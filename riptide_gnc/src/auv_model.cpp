#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, Vector3f inertias, float V, float, fluid_rho, Vector3f cob,
                  Matrix32f drag, vector<Vector5f> auv_thrusters);
{
    mass = m;             // [kg]
    inertia = inertias;   // Assume only Jxx, Jyy, and Jzz
    volume = V;           // [m^3]
    fluidRho = fluid_rho; // [kg/m^3]
    rcob = cob;
    dragCoeffs = drag;
    thrusters = auv_thrusters;
}

// Get vector of ALL actuation forces/moments as expressed in the body frame
Vector6f AUVModel::DecomposeActuation(VectorXf actuation)
{
    Vector6f output;
    output.setZero();

    if (actuation.rows() == thrusters.size())
        for (int i = 0; i < thrusters.size(); i++)
            output(i) = output(i) + AUVModel::DecomposeActuator(i, actuation(i));

    return output;
}

// Get actuator forces/moments as expressed in the body frame
// Parameters:
//      thruster = index of given thruster
//      force = force from given thruster [N]
VectorXf AUVModel::DecomposeActuator(int thruster, float force)
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
    tao.setZero(); // Moment vector
    float W = mass * GRAVITY;
    float B = -fluidRho * volume * GRAVITY;

    w(0) = -(W + B) * sin(theta);
    w(1) = (W + B) * sin(phi) * cos(theta);
    w(2) = (W + B) * cos(phi) * cos(theta);
    fb(0) = -B * sin(theta);
    fb(1) = B * sin(phi) * cos(theta);
    fb(2) = B * cos(phi) * cos(theta);
    tao = rcob.cross(fb);
    output << w, tao;

    return output;
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
Vector3f AUVModel::BodyFrameAccel(Vector3f attitude, Vector3f velBF, Vector3f angVelBF)
{

}