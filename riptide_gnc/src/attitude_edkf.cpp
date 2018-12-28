#include "riptide_gnc/attitude_edkf.h"

AttitudeEDKF::AttitudeEDKF(float max_pitch, Vector3f inertia, Vector3f damping,
                           Matrix6f Q1, MatrixXf R1, Matrix6f Q2, MatrixXf R2)
{
    max_theta = max_pitch;
    J = inertia;
    b = damping;
    pd.setZero();
    init = false;

    F1.setZero();
    F2.setZero();
    H1.setZero();
    H2.setZero();

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
// Z = IMU measurement (phi, theta, psi, p, q, r)
void AttitudeEDKF::UpdateAttEDKF(float time_step, Vector3f input_states, Vector6f Z)
{
    dt = time_step;

    if (abs(Z(1)) < max_theta) // Pitch angle within specified operation range
    {
        if (!init)
        {
            init = true;
            AttitudeEDKF::InitAttEDKF(input_states, Z);
        }
        AttitudeEDKF::CalcPartialDerivatives();
        AttitudeEDKF::UpdateAngMotStates(input_states, Z);
        AttitudeEDKF::UpdateAttStates(Z);
    }
    else // Pitch angle too steep - just run the initialize function
    {
        AttitudeEDKF::InitAttEDKF(input_states, Z);
    }
}

// Initialize Attitude EDKF
// input_states = pre-calculated p_dot, q_dot, r_dot
// Z = IMU measurement (phi, theta, psi, p, q, r)
void InitAttEDKF(Vector3f input_states, Vector6f Z)
{
    // Initialize X1hat (p, q, r) to IMU (p, q, r) and pre-calculated input_states
    for (int i = 0; i < s; i++)
        X1hat(i) = Z(i + s);
    for (int i = 0; i < s; i++)
        X1hat(i + s) = input_states(i);

    // Initialize X2hat (phi, theta, psi) equal to IMU (phi, theta, psi)
    for (int i = 0; i < s; i++)
        X2hat(i) = Z(i);

    // Initialize X2hat (phi_dot, theta_dot, psi_dot) to actual equations
    p = X1hat(0), q = X1hat(1), r = X1hat(2);
    phi = X2hat(0), theta = X2hat(1), psi = X2hat(2);
    X2hat(3) = p + tan(theta) * (sin(phi) * q + cos(phi) * r);
    X2hat(4) = cos(phi) * q - sin(phi) * r;
    X2hat(5) = sec(theta) * (sin(phi) * q + cos(phi) * r);
}

// Update Angular Motion States
void AttitudeEDKF::UpdateAngMotStates(Vector3f input_states, Vector6f Z)
{
    Vector3f Z1; // Input to Angular Motion EDKF is (p, q, r) from IMU
    for (int i = 0; i < s; i++)
        Z1(i) = Z(i + s);
    AttitudeEDKF::TimePredictAngMotState(input_states);
    AttitudeEDKF::CalcAngMotJacobians();
    AngMotKF->UpdateKFOverride(X1hatPre, Z1, F1, H1);
    X1hat = AngMotKF->Xhat;
}

// Update Attitude EDKF
void AttitudeEDKF::UpdateAttStates(Vector6f Z)
{
    Vector6f Z2 = Z;
    for (int i = 0; i < s; i++) // Set measured (p, q, r) to values from X1hat
        Z2(i + s) = X1hat(i);
    AttitudeEDKF::TimePredictAttState();
    AttitudeEDKF::CalcAttJacobians();

    // Make sure the difference b/w X2hatPre (phi, psi) and Z (phi, psi) is within PI
    Z2(0) = AttitudeEDKF::KeepMsmtWithinPI(X2hatPre(0), Z2(0));
    Z2(2) = AttitudeEDKF::KeepMsmtWithinPI(X2hatPre(2), Z2(2));

    AttKF->UpdateKFOverride(X2hatPre, Z2, F2, H2);
    X2hat = AttKF->Xhat;
    X2hat(0) = AttitudeEDKF::KeepAngleWithinPI(X2hatPre(0));
    X2hat(2) = AttitudeEDKF::KeepAngleWithinPI(X2hatPre(2));
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

    // Fill in the rest of H2
    H2(3, 3) = 1;
    H2(3, 5) = -sin(theta);
    H2(4, 4) = cos(phi);
    H2(4, 5) = sin(phi) * cos(theta);
    H2(5, 4) = -sin(phi);
    H2(5, 5) = cos(phi) * cos(theta);
}

// Keep angle within +/- PI (applies to roll and yaw)
float AttitudeEDKF::KeepAngleWithinPI(float angle)
{
    if (angle > M_PI)
        return (angle - 2 * M_PI);
    if (angle < -M_PI)
        return (angle + 2 * M_PI);
}

// Keep measurement within PI of prediction (applies to roll and yaw)
float AttitudeEDKF::KeepMsmtWithinPI(float predict, float msmt)
{
    if (msmt - predict > M_PI)
        return (msmt - 2 * M_PI);
    if (msmt - predict < -M_PI)
        return (msmt + 2 * M_PI);
}