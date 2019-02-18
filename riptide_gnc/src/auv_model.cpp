#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, Vector3f J, float V, float fluid_rho, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix32f> &dragCoeffs, vector<Vector5f> &auv_thrusters);
{
    mass = m;        // [kg]
    inertia = J;     // Assume only Jxx, Jyy, and Jzz
    vol = V;         // [m^3]
    rho = fluid_rho; // [kg/m^3]
    CoB = cob;
    drag = dragCoeffs;
    thrusters = auv_thrusters;
}

// Get vector of ALL actuation forces/moments as expressed in the body frame
// Parameters:
//      totalFM = Vector6f for (3x) overall forces and (3x) overall moments actuators exert on vehicle
//      actuation = VectorXf of force exerted on vehicle by each thruster
void AUVModel::DecomposeActuation(Ref<Vector6f> totalFM, const Ref<const VectorXf> &actuation)
{
    totalFM.setZero();

    if (actuation.rows() == thrusters.size())
        for (int i = 0; i < thrusters.size(); i++)
        {
            Vector6f temp;
            AUVModel::DecomposeActuator(temp, i, actuation(i));
            totalFM = allFM + temp;
        }
}

// Get forces/moments that a SINGLE actuator exerts on the vehicle, expressed in the body frame
// Parameters:
//      thrustFM = Vector6f for (3x) forces and (3x) moments thruster exerts on vehicle
//      thruster = index of given thruster
//      force = force from given thruster [N]
void AUVModel::DecomposeActuator(Ref<Vector6f> thrustFM, int thruster, float force)
{
    thrustFM.setZero(); 

    if (thruster < thrusters.size()) // Verify thruster index is valid
    {
        Vector3f r, f, tau;
        r.setZero();                          // Radius vector
        f.setZero();                          // Force vector
        tau.setZero();                        // Moment vector
        float psi = thrusters[thruster](3);   // Yaw
        float theta = thrusters[thruster](4); // Pitch

        r = thrusters[thruster].head<3>();
        f(0) = force * cos(psi) * cos(theta);
        f(1) = force * sin(psi) * cos(theta);
        f(2) = -force * sin(theta);
        tau = r.cross(f);
        thrustFM << f, tau;
    }
}

// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the body frame
// Parameters:
//      weightFM = Vector6f of forces and moments due to vehicle's weight and buoyancy
//      phi = roll [rad]
//      theta = pitch [rad]
void AUVModel::DecomposeWeightForces(Ref<Vector6f> weightFMfloat phi, float theta)
{
    weightFM.setZero();

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
    weightFM << w, tau;
}

// Get rotation matrix from world-frame to body-frame using Euler Angles (yaw, pitch, roll)
// Ex. Vb = R * Vw, Vb = vector in body-frame coordinates, Vw = vector in world-frame coordinates
void AUVModel::GetEulerYPR(Ref<Matrix3f> R, float yaw, float pitch, float roll)
{
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
}

// Compute a priori states for Translation EKF
// 9 States: inertial pos, inertial vel, inertial accel (the last two are expressed in the body-frame)
void AUVModel::TransEKF9APriori(Ref<Vector9f> apriori, const Ref<const Vector3f> &posPrev, const Ref<const Vector3f> &velPrev,
                                    const Ref<const Vector3f> &attitude, const Ref<const Vector3f> &omega)
{
    float roll = attitude(0);
    float pitch = attitude(1);
    float yaw = attitude(2);
    float uPrev = velPrev(0);
    float vPrev = velPrev(1);
    float wPrev = velPrev(2);
    float p = omega(0);
    float q = omega(1);
    float r = omega(2);
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
Vector3f AUVModel::BodyFrameAccel(Vector3f attitude, Vector3f velBF, Vector3f angVelBF)
{
}