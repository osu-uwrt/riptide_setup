#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, float V, float fluid_rho, const Ref<const Matrix3f> &Inertia, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix62f> &drag, vector<Vector5f> &auv_thrusters);
{
    mass = m;          // [kg]
    inertia = Inertia; // 3x3 inertia matrix
    vol = V;           // [m^3]
    rho = fluid_rho;   // [kg/m^3]
    CoB = cob;
    dragCoeffs = drag;
    thrusters = auv_thrusters;
    numThrusters = thrusters.cols();
    Fg = mass * GRAVITY;      // Force due to gravity [N]
    Fb = rho * vol * GRAVITY; // Buoyant Force [N]

    // Get primary and products of inertia elements
    Ixx = inertia(0, 0);
    Iyy = inertia(1, 1);
    Izz = inertia(2, 2);
    Ixy = -inertia(0, 1);
    Ixz = -inertia(0, 2);
    Iyz = -inertia(1, 2);

    AUVModel::SetThrustCoeffs();
}

// Set the thruster coefficients. Each column corresponds to a single thruster.
// Rows 1,2,3: force contribution in the X, Y, and Z axes, respectively
// Rows 4,5,6: effective moment arms about the X, Y, and Z axes, respectively
void AUVModel::SetThrustCoeffs()
{
    thrustCoeffs.resize(6, numThrusters);
    thrustCoeffs.setZero();

    for (int i = 0; i < numThrusters; i++)
    {
        float psi = thrusters(3, i) * PI / 180;
        float theta = thrusters(4, i) * PI / 180;
        thrustCoeffs(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
        thrustCoeffs(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
        thrustCoeffs(2, i) = -sin(theta);           // -sin(theta)

        // Cross-product
        thrustCoeffs.block<3, 1>(3, i) = thrusters.block<3, 1>(0, i).cross(thrustCoeffs.block<3, 1>(0, i));
    }
}

// Get total thruster forces/moments as expressed in the B-frame
// Parameters:
//      thrusts = VectorXf of force exerted on vehicle by each thruster
Vector6f AUVModel::GetTotalThrustLoad(const Ref<const VectorXf> &thrusts)
{
    Vector6f thrustLoad;
    thrustLoad.setZero();

    if (thrusts.rows() == numThrusters)
        for (int i = 0; i < numThrusters; i++)
            thrustLoad = thrustLoad + thrusts(i) * thrustCoeffs.col(i);

    return thrustLoad;
}

// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the B-frame
// Parameters:
//      attitude = Eigen::Vector3f of yaw, pitch, and roll (in this order)
Vector6f AUVModel::GetWeightLoad(const Ref<const Vector3f> &attitude)
{
    Vector6f weightLoad;
    weightLoads.setZero();

    Vector3f coeffs; // Store coefficients here
    coeffs.setZero();
    float phi = attitude(2);
    float theta = attitude(1);

    coeffs(0) = -sin(theta);
    coeffs(1) = sin(phi) * cos(theta);
    coeffs(2) = cos(phi) * cos(theta);

    weightLoad.head<3>() = (Fg - Fb) * coeffs;        // Forces, expressed in  B-frame
    weightLoad.tail<3>() = CoB.cross((-Fb * coeffs)); // Moments, expressed in B-frame

    return weightLoad;
}

// Compute a priori states for Two-Stage Translation EKF
// This is called two-stage b/c the acceleration values come from an IMU's AEKF
// 9 States: inertial pos (I-frame), inertial vel (B-frame), inertial accel (B-frame)
Vector9f AUVModel::GetTransEKFTwoStageAPriori(const Ref<const Vector9f> &xPrev, const Ref<const Vector3f> &attitude,
                                              const Ref<const Vector3f> &pqr, const Ref<const VectorXf> &thrusts,
                                              float dt, int maxIter)
{
    Vector9f apriori;
    apriori.setZero();

    Vector3f uvwPrev, uvw, uvwDot, transportThm, uvwSq, Fd;
    uvwPrev = xPrev.segment<3>(3);
    uvw = uvwPrev; // xPrev.segment<3>(3);

    Vector3f weightLoad = AUVModel::GetWeightLoad(attitude);
    Vector3f thrustLoad = AUVModel::GetTotalThrustLoad(thrusts);
    Vector3f dragT1 = drag.block<3, 1>(0, 0); // Linear drag coefficients for trans. motion
    Vector3f dragT2 = drag.block<3, 1>(0, 1); // Quadratic drag coefficients for trans. motion

    // Recurse over accel equations a few times. Hopefully recursing will create a better prediction
    // Goal: Use dynamics to better predict the acceleration
    for (int i = 0; i < maxIter; i++)
    {
        transportThm = pqr.cross(uvw);
        uvwSq = uvw.array() * uvw.array();
        Fd = (dragT1.array() * uvw.array()) + (SgnMat(uvw).array() * 0.5 * rho * dragT2.array * uvwSq);
        uvwDot = (thrustLoad + weightLoad - Fd) / m - transportThm; // Compute uvwDot a priori
        uvw = uvwPrev + apriori.segment<3>(6) * dt;                 // Compute uvw a priori
    }

    // Express a priori of vehicle's position expressed in the inertial frame
    Matrix3f R = GetEulerRotMat(attitude).transpose();
    Vector3f velI = R * uvwPrev;
    Vector3f accelI = R * uvwDot;
    Vector3f xyzI = xPrev.head<3>() + (velI * dt) + (0.5 * accelI * pow(dt, 2));

    // Append all vectors to apriori
    apriori << xyzI, uvw, uvwDot;
    return apriori;
}

// Compute Jacobian for Two-Stage Translation EKF
// Parameters:
//      apriori = states computed by GetTransEKFTwoStageApriori
//      attitude = attitude of yaw, pitch, and roll (in this order)
//      pqr = inertial angular velocity, expressed in B-frame
//      dt = time step;
Matrix9f AUVModel::GetTransEKFTwoStageJacobianA(const Ref<const Vector9f> &apriori, const Ref<const Vector3f> &attitude,
                                                const Ref<const Vector3f> &pqr, float dt;)
{
    Matrix9f jacobi = Matrix9f::Zero();
    Matrix3f R = GetEulerRotMat(attitude).transpose();
    Vector3f dragT1 = drag.block<3, 1>(0, 0); // Linear drag coefficients for trans. motion
    Vector3f dragT2 = drag.block<3, 1>(0, 1); // Quadratic drag coefficients for trans. motion

    jacobi.block<6, 6>(0, 0) = MatrixXf::Identity(6, 6);
    jacobi.block<3, 3>(0, 3) = dt * R;
    jacobi.block<3, 3>(0, 6) = pow(dt, 2) * R;
    jacobi.block<3, 3>(3, 6) = dt * Matrix3f::Identity();

    Vector3f uvwAbs = apriori.segment<3, 1>(3).array().abs(); // Absolute value of uvw
    Vector3f dragPartials = -(dragT1.array() + 2 * dragT2.array() * uvwAbs.array()) / mass;
    Matrix3f dragDiag = dragPartials.asDiagonal();
    Matrix3f lastBlock = dragDiag - SkewSym(pqr);

    jacobi.block<3, 3>(6, 3) = lastBlock;

    return jacobi;
}

// Compute the Jacobian of the A-matrix for LQR
// DO NOT CHANGE !!!!!!
Matrix12f AUVModel::GetLQRJacobianA(const Ref<const Vector12f> &states)
{
    // Variables for Auto Diff.
    size_t n = 12, a = 1;
    vector<CppAD::AD<double>> X(n), Xdot(n), A(a), B(a), C(a);
    vector<double> jacobi(n * n); // Create nxn Jacobian matrix

    // MUST set X to contain INDEPENDENT variables
    // Calling this function will BEGIN the recording sequence
    CppAD::Independent(X);

    // 1. Set time derivatives of: xI, yI, zI,
    Xdot[_xI] = (CppAD::cos(X[_psi]) * CppAD::cos(X[_theta])) * X[_U] +
                (-CppAD::sin(X[_psi]) * CppAD::cos(X[_phi]) + CppAD::cos(X[_psi]) * CppAD::sin(X[_theta]) * CppAD::sin(X[_phi])) * X[_V] +
                (CppAD::sin(X[_psi]) * CppAD::sin(X[_phi]) + CppAD::cos(X[_psi]) * CppAD::sin(X[_theta]) * CppAD::cos(X[_phi])) * X[_W];

    Xdot[_yI] = (CppAD::sin(X[_psi]) * CppAD::cos(X[_theta])) * X[_U] +
                (CppAD::cos(X[_psi]) * CppAD::cos(X[_phi]) + CppAD::sin(X[_psi]) * CppAD::sin(X[_theta]) * CppAD::sin(X[_phi])) * X[_V] +
                (-CppAD::cos(X[_psi]) * CppAD::sin(X[_phi]) + CppAD::sin(X[_psi]) * CppAD::sin(X[_theta]) * CppAD::cos(X[_phi])) * X[_W];

    Xdot[_zI] = -CppAD::sin(X[_theta]) * X[_U] + (CppAD::cos(X[_theta]) * CppAD::sin(X[_phi])) * X[_V] + (CppAD::cos(X[_theta]) + CppAD::cos(X[_phi])) * X[_W];

    // 2. Set time-derivatives of: psi, theta, and phi
    Xdot[_psi] = (X[_Q] * CppAD::sin(X[_phi]) + X[_R] * CppAD::cos(X[_phi])) / cos(X[_theta]);

    Xdot[_theta] = X[_Q] * CppAD::cos(X[_phi]) - X[_R] * CppAD::sin(X[_phi]);

    Xdot[_phi] = X[_P] + (X[_Q] * CppAD::sin(X[_phi]) + X[_R] * CppAD::cos(X[_phi])) * CppAD::tan(X[_theta]);

    // 3. Set time-derivaivesof : U, V, W
    Xdot[_U] = (-drag(0, 0) * X[_U] - 0.5 * Sgn(states(_U)) * rho * drag(0, 1) * X[_U] * X[_U]) / mass -
               ((Fg - Fb) / mass) * CppAD::sin(X[_theta]) + X[_R] * X[_V] - X[_Q] * X[_W];

    Xdot[_V] = (-drag(1, 0) * X[_V] - 0.5 * Sgn(states(_V)) * rho * drag(1, 1) * X[_V] * X[_V]) / mass +
               ((Fg - Fb) / mass) * CppAD::cos(X[_theta]) * CppAD::sin(X[_phi]) + X[_P] * X[_W] - X[_R] * X[_U];

    Xdot[_W] = (-drag(2, 0) * X[_W] - 0.5 * Sgn(states(_W)) * rho * drag(2, 1) * X[_W] * X[_W]) / mass +
               ((Fg - Fb) / mass) * CppAD::cos(X[_theta]) * CppAD::cos(X[_phi]) + X[_Q] * X[_U] - X[_P] * X[_V];

    // 4. Set time-derivatives of: P, Q, R
}
