#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, Vector3f J, float V, float fluid_rho, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix62f> &dragCoeffs, vector<Vector5f> &auv_thrusters);
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
Vector6f AUVModel::DecomposeActuation(const Ref<const VectorXf> &actuation)
{
    Vector6f totalFM;
    totalFM.setZero();

    if (actuation.rows() == thrusters.size())
    {
        for (int i = 0; i < thrusters.size(); i++)
        {
            totalFM = totalFM allFM + AUVModel::DecomposeActuator(i, actuation(i));
        }
    }
    return totalFM;
}

// Get forces/moments that a SINGLE actuator exerts on the vehicle, expressed in the body frame
// Parameters:
//      thrustFM = Vector6f for (3x) forces and (3x) moments thruster exerts on vehicle
//      thruster = index of given thruster
//      force = force from given thruster [N]
Vector6f AUVModel::DecomposeActuator(int thruster, float force)
{
    Vector6f thrustFM;
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
    return thrustFM;
}

// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the body frame
// Parameters:
//      weightFM = Vector6f of forces and moments due to vehicle's weight and buoyancy
//      phi = roll [rad]
//      theta = pitch [rad]
Vector6f AUVModel::DecomposeWeightForces(const Ref<const Vector3f> &attitude)
{
    Vector6f weightFM;
    weightFM.setZero();

    Vector3f w, fb, tau;
    w.setZero();   // Weight vector
    fb.setZero();  // Buoyant force vector
    tau.setZero(); // Moment vector
    float W = mass * GRAVITY;
    float B = fluidRho * vol * GRAVITY;
    float phi = attitude(0);
    float theta = attitude(1);

    w(0) = -(W - B) * sin(theta);
    w(1) = (W - B) * sin(phi) * cos(theta);
    w(2) = (W - B) * cos(phi) * cos(theta);
    fb(0) = B * sin(theta);
    fb(1) = -B * sin(phi) * cos(theta);
    fb(2) = -B * cos(phi) * cos(theta);
    tau = CoB.cross(fb);
    weightFM << w, tau;

    return weightFM;
}

// Get rotation matrix from world-frame to body-frame using Euler Angles (yaw, pitch, roll)
// Ex. Vb = R * Vw, Vb = vector in body-frame coordinates, Vw = vector in world-frame coordinates
Matrix3f AUVModel::GetEulerYPR(const Ref<const Vector3f> &attitude)
{
    Matrix3f R = Matrix3f::Zero();

    float s_phi = sin(attitude(0));
    float c_phi = cos(attitude(0));
    float s_theta = sin(attitude(1));
    float c_theta = cos(attitude(1));
    float s_psi = sin(attitude(2));
    float c_psi = cos(attitude(2));

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

// Compute a priori states for Two-Stage Translation EKF
// This is called two-stage b/c the acceleration values come from an IMU's AEKF
// 9 States: inertial pos, inertial vel, inertial accel (the last two are expressed in the body-frame)
Vector9f AUVModel::TransEKFTwoStageAPriori(const Ref<const Vector9f> &xPrev, const Ref<const Vector3f> &attitude,
                                           const Ref<const Vector3f> &pqr, const Ref<const VectorXf> &actuation,
                                           float dt, int max_iter)
{
    Vector9f apriori;
    apriori.setZero();

    Vector3f uvwPrev, uvw, uvwDot, transportThm;
    uvwPrev = xPrev.segment<3>(3);
    uvw = xPrev.segment<3>(3);

    Vector3f weightFM = AUVModel::DecomposeWeightForces(attitude);
    Vector3f totalFM = AUVModel::DecomposeActuation(actuation);
    Vector3f d1 = drag.block<3, 1>(0, 0); // Linear drag coefficients
    Vector3f d2 = drag.block<3, 1>(0, 1); // Quadratic drag coefficients

    // Recurse over accel equations a few times. Hopefully this will create a better prediction
    for (int i = 0; i < max_iter; i++)
    {
        transportThm = pqr.cross(uvw);
        Vector3f Fd = (d1.array() * uvw.array()) + (AUVModel::Sgn(uvw).array() * 0.5 * rho * d2.array * uvw.array * uvw.array);
        uvwDot = (totalFM + weightFM - Fd) / m - transportThm; // Compute accel a priori
        uvw = uvwPrev + apriori.segment<3>(6) * dt;            // Computer vel a priori
    }

    // Express a priori of vehicle's position expressed in the inertial frame
    Matrix3f R = AUVModel::GetEulerYPR(attitude);
    Vector3f velIFrame = R * uvw;
    Vector3f accelIFrame = R * uvwDot;
    Vector3f xyzIFrame = xPrev.segment<3>(0) + velIFrame * dt + 0.5 * accelIFrame * (dt ^ 2);

    // Append all vectors to apriori
    apriori << xyzIFrame, uvw, uvwDot;
    return apriori;
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
MatrixXf AUVModel::Sgn(const Ref<const MatrixXf> mat)
{
    MatrixXf sgnMat = mat;

    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            if (mat(i, j) > 0)
                sgnMat(i, j) = 1;
            else if (mat(i, j) < 0)
                sgnMat(i, j) = -1;
            else
                sgnMat(i, j) = 0;
        }
    }
    return sgnmat;
}