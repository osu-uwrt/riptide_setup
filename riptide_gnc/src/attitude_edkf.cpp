#include "riptide_gnc/attitude_edkf.h"

AttitudeEDKF::AttitudeEDKF(bool cascade, Vector3f inertia, Vector3f damping,
                           Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2)
{
    cascadeInit = cascade;
    if (cascadeInit) // Have access to both attitude and angular velocity
    {
        H2.resize(6, 6);
        H2.setZero();
    }
    else
    { // Only have access to angular velocity
        H2.resize(3, 6);
        H2.setZero();
    }

    J = inertia;
    b = damping;
    pd.setZero();

    F1.setZero();
    F2.setZero();
    H1.setZero();

    AngMotKF = new KalmanFilter(F1, H1, Q1, R1);
    AttKF = new KalmanFilter(F2, H2, Q2, R2);

    // Initialize indeces (will be handy for calculating partial derivatives)
    x = 0;
    y = 1;
    z = 2;
    s = 3;
}

// Update Attitude EDKF
// time_step = dt
// input_states = pre-calculated p_dot, q_dot, r_dot
// Z = IMU measurement (p, q, r) OR (phi, theta, psi, p, q, r)
void AttitudeEDKF::UpdateAttEDKF(float time_step, Vector3f input_states, VectorXf Z)
{
    dt = time_step;
    int size = Z.rows();
    if ((size != s) || (size != 2*s))
        throw std::runtime_error("Dimension mismatch: Input measurement must be of row_size %i or %i", s, 2*s);
    AttitudeEDKF::CalcPartialDerivatives();

    // Update Angular Motion States
    Vector3f Z1; // Input to Angular Motion EDKF is (p, q, r) from IMU
    for (int i = 0; i < s; i++)
        Z1(i) = Z(size - s + i);
    AttitudeEDKF::TimePredictAngMotState(input_states);
    AttitudeEDKF::CalcAngMotJacobians();
    AngMotKF->UpdateKFOverride(X1hatPre, Z1, F1, H1);
    X1hat = AngMotKF->Xhat;

    // Update Attitude States
    VectorXf Z2;
    Z2.resize(size);
    for (int i = 0; i < s; i++)
        Z2(i) = X1hat(i);

    if (cascadeInit) // For cascading, treat X1hat(p, q, r) from EDKF as measurement input
    {
        for (int i = 0; i < s; i++)
            Z2(i + s) = X1hat(i);
    }
    AttitudeEDKF::TimePredictAttState();
    AttitudeEDKF::CalcAttJacobians();
    AttKF->UpdateKFOverride(X2hatPre, Z2, F2, H2);
    X2hat = AttKF->Xhat;
    AttitudeEDKF::KeepAnglesInRange();
}

// Time-predict Step for Angular Motion States
// Constant Acceleration Model: Xk = Xk + Xk_dot*dt
void AttitudeEDKF::TimePredictAngMotState(Vector3f input_states)
{
    for (int i = 0; i < 3; i++)
    {
        X1hatPre(i) = X1hat(i) + X1hat(i + 3) * dt; // Update: p, q, r (using previous state's values)
        X1hatPre(i + 3) = input_states(i);          // Update: p_dot, q_dot, r_dot (from input_states)
    }
}

// Time-predict Step for Attitude States
// Constant Acceleration Model: Xk = Xk + Xk_dot*dt + (1/2)*xk_ddot*dt^2
void AttitudeEDKF::TimePredictAttState()
{
    phi_ddot = p_dot + tan(theta) * (phi_dot * c2 + c3) + (theta_dot * (sec(theta) ^ 2) * c1);
    theta_ddot = (-phi_dot * c1) + c4;
    psi_ddot = (theta_dot * sec(theta) * tan(theta) * c1) + sec(theta) * (phi_dot * c2 + c3);

    X2hatPre(0) = phi + phi_dot * dt + 0.5 * phi_ddot * dt ^ 2;       // Update phi
    X2hatPre(1) = theta + theta_dot * dt + 0.5 * theta_ddot * dt ^ 2; // Update theta
    X2hatPre(2) = psi + psi_dot * dt + 0.5 * psi_ddot * dt ^ 2;       // Update psi
    X2hatPre(3) = phi_dot + phi_ddot * dt;                            // Update phi_dot
    X2hatPre(4) = theta_dot + theta_ddot * dt;                        // Update theta_dot
    X2hatPre(5) = psi_dot + psi_ddot * dt;                            // Update psi_dot
}

void AttitudeEDKF::KeepAnglesInRange()
{
    // Roll (phi) restricted to +/- PI (180 deg.)
    if (X2hat(0) > M_PI)
        X2hat(0) -= M_PI;
    if (X2hat(0) < M_PI)
        X2hat(0) += M_PI;

    // Pitch (theta) restricted to +/- PI/2 (90 deg.)
    
    // Yaw (psi) restricted to +/- PI (180 deg.)
    if (X2hat(2) > M_PI)
        X2hat(2) -= M_PI;
    if (X2hat(2) < M_PI)
        X2hat(2) += M_PI;
}

// Calculate Partial Derivatives, which are needed to calculate the Jacobians
void AttitudeEDKF::CalcPartialDerivatives()
{
    // Helper Variables
    p = X1hat(0);
    q = X1hat(1);
    r = X1hat(2);
    p_dot = X1hat(3);
    q_dot = X1hat(4);
    r_dot = X1hat(5);

    phi = X2hat(0);
    theta = X2hat(1);
    psi = X2hat(2);
    phi_dot = X2hat(3);
    theta_dot = X2hat(4);
    psi_dot = X2hat(5);

    // Common Expressions
    c1 = sin(phi) * q + cos(phi) * r;
    c2 = cos(phi) * q - sin(phi) * r; // d(c1)/d(phi)
    c3 = sin(phi) * q_dot + cos(phi) * r_dot;
    c4 = cos(phi) * q_dot - sin(phi) * r_dot; // d(c3)/d(phi)

    // Partial Derivatives
    // partial p_dot / (partial p, q, r)
    pd(0) = -b(x) / J(x);              // -bx/Jxx
    pd(1) = -r * (J(z) - J(y)) / J(x); // -r(Jzz-Jyy)/Jxx
    pd(2) = -q * (J(z) - J(y)) / J(x); // -q(Jzz-Jyy)/Jxx

    // partial q_dot / (partial p, q, r)
    pd(3) = -r * (J(x) - J(z)) / J(y); // -r(Jxx-Jzz)/Jyy
    pd(4) = -b(y) / J(y);              // -by/Jyy
    pd(5) = -p * (J(x) - J(z)) / J(y); // -p(Jxx-Jzz)/Jyy

    // partial r_dot / (partial p, q, r)
    pd(6) = -q * (J(y) - J(x)) / J(z); // -q(Jyy-Jxx)/Jzz
    pd(7) = -p * (J(y) - J(x)) / J(z); // -p(Jyy-Jxx)/Jzz
    pd(8) = -b(z) / J(z);              // -bz/Jzz

    // partial phi_dot / (partial phi, theta, psi)
    pd(9) = tan(theta) * c2;
    pd(10) = (sec(theta) ^ 2) * c1;
    pd(11) = 0;

    // partial phi_ddot / (partial phi, phi_dot, theta, theta_dot, psi, psi_dot)
    pd(12) = (-phi_dot * tan(theta) * c1) + (theta_dot * (sec(theta) ^ 2) * c2) + tan(theta) * c4;
    pd(13) = tan(theta) * c2;
    pd(14) = (sec(theta) ^ 2) * (phi_dot * c2 + (2 * theta_dot * tan(theta) * c1) + c3);
    pd(15) = (sec(theta) ^ 2) * c1;
    pd(16) = 0;
    pd(17) = 0;

    // partial theta_dot / (partial phi, theta, psi)
    pd(18) = -c1;
    pd(19) = 0;
    pd(20) = 0;

    // partial theta_ddot / (partial phi, phi_dot, theta, theta_dot, psi, psi_dot)
    pd(21) = -(phi_dot * c2 + c3);
    pd(22) = -c1;
    pd(23) = 0;
    pd(24) = 0;
    pd(25) = 0;
    pd(26) = 0;

    // partial psi_dot / (partial phi, theta, psi)
    pd(27) = sec(theta) * c2;
    pd(28) = sec(theta) * tan(theta) * c1;
    pd(29) = 0;

    // partial psi_ddot / (partial phi, phi_dot, theta, theta_dot, psi, psi_dot)
    pd(30) = (theta_dot * sec(theta) * tan(theta) * c2) + sec(theta) * (-phi_dot * c1 + c2);
    pd(31) = sec(theta) * c2;
    pd(32) = (theta_dot * c1 * (sec(theta) * (tan(theta) ^ 2) + (sec(theta) ^ 3))) +
             sec(theta) * tan(theta) * (phi_dot * c2 + c3);
    pd(33) = sec(theta) * tan(theta) * c1;
    pd(34) = 0;
    pd(35) = 0;
}

// Calculate Jacobian F1 = partial f1 / partial x1
// Calculate Jacobian H1 = partial h1 / partial x1
void AttitudeEDKF::CalcAngMotJacobians()
{
    int a = 0; // Index for partial derivatives
    F1.setZero();
    H1.setZero();

    for (int i = 0; i < s; i++)
    {
        for (int j = 0; j < s; j++)
        {
            a = i * s + j;
            F1(i, j) = pd(a) * dt;   // Update top-left 3x3
            F1(i + s, j) = F1(i, j); // Update lower-left 3x3

            if (i == j) // Account for a few diagonal terms within the above 3x3's
            {
                F1(i, j) += 1;
                F1(i, j + s) = dt;
                H1(i, j) = 1;
            }
        }
    }
}

// Calculate Jacobian F2 = partial f2 / partial x2
// Calculate Jacobian H2 = partial h2 / partial x2
// Account for cascading (if initialized)
void AttitudeEDKF::CalcAttJacobians()
{
    int offset = 9;          // Offset
    int d = 0, e = 0, f = 0; // Indeces for partial derivatives
    F2.setZero();
    H2.setZero();

    for (int i = 0; i < s; i++)
    {
        for (int j = 0; j < s; j++)
        {
            d = offset * (i + 1) + j;
            e = d + s + j;
            f = e + 1;

            F2(i, j) = pd(d) * dt + 0.5 * pd(e) * dt ^ 2; // Update upper-left 3x3
            F2(i + s, j) = pd(d) + pd(e) * dt;            // Update lower-left 3x3
            F2(i, j + s) = 0.5 * pd(f) * dt ^ 2;          // Update upper-right 3x3
            F2(i + s, j + s) = pd(f) * dt;                // Update lower-right 3x3

            if (i == j) // Account for a few diagonal terms within the above 3x3's
            {
                F2(i, j) += 1;
                F2(i, j + s) += dt;
                F2(i + s, j + s) += 1;
                H2(i, j) = 1;
            }
        }
    }

    if (cascadeInit)
    {
        H2(3, 3) = 1;
        H2(3, 5) = -sin(theta);
        H2(4, 4) = cos(phi);
        H2(4, 5) = sin(phi) * cos(theta);
        H2(5, 4) = -sin(phi);
        H2(5, 5) = cos(phi) * cos(theta);
    }
}
