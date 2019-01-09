#include "riptide_gnc/linear_motion_edkf.h"

// NOTE: vel and accel sensors are BODY-FRAME, where pos sensors are WORLD-FRAME
// drag_order = order of the polynomial used for drag (linear or quadratic)
// damping = the linearized damping slope for each body-frame axis
//      Ex: Linear: accel_drag = -b*v/mass --> damping slope = -b/mass
//      Ex: Quadratic: accel_drag = (0.5/mass)*rho*C*A*v^2 --> damping solpe = rho*C*A/mass
// posIn, velIn, accelIn = matrices indicating which sensors are provided to this EKF
//      Row 0 = X; Row 1 = Y; Row 2 = Z
//      Each column represents data from a different sensor
//      1 = msmt provided, 0 = msmt not provided
// Rpos, Rvel, and Raccel = the main-diagonal elements for the sensor noise covariance matrices
//      Row 0 = X; Row 1 = Y; Row 2 = Z
// Q = horizontal concatenation of process coviariance matrices for each axis (Qx, Qy, and Qz)
//      The first row is for position, the second two rows are for vel and accel
//      For simplicity, the bottom two rows will be used for the body-frame EKFs, but all three rows
//      will be used for world-frame estimation (pos, vel, and accel)
LinearMotionEDKF::LinearMotionEDKF(int drag_order, Vector3f damping, Matrix3Xi posIn, Matrix3Xi velIn, Matrix3Xi accelIn,
                                   Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix3Xf Q)
{
    dragOrder = drag_order;
    linearizedDamping = damping;

    /******************* Body-Frame Velocity and Acceleration Initialization ********************/
    velMaskbf.resize(velIn.rows(), velIn.cols());
    accelMaskbf.resize(accelIn.rows(), accelIn.cols());
    velMaskbf = velIn;     // Available body-frame velocity data
    accelMaskbf = accelIn; // Available body-frame acceleration data
    int colsVel = velMaskbf.cols();
    int colsAccel = accelMaskbf.cols();

    // Count number of each type of sensor
    velSensors = 0, accelSensors = 0;
    for (int i = 0; i < colsVel; i++)
        if (velMaskbf.col(i).sum() > 0)
            velSensors++;
    for (int i = 0; i < colsAccel; i++)
        if (accelMaskbf.col(i).sum() > 0)
            accelelSensors++;

    // Find all possible sensor combinations
    int numSensors = velSensors + accelSensors;
    int noVel = (int)(velSensors == 0);
    int noAccel = (int)(accelSensors == 0);
    LinearMotionEDKF::FindDataCombos(&sensorCombos, velSensors, accelSensors);

    // Create states vector
    Vector3i states;
    states(0) = 0; // Not tracking pos
    states(1) = 1; // Tracking vel
    states(2) = 1; // Tracking accel

    RowXi combo;
    combo.resize(1, numSensors + noVel + noAccel);
    Matrix3Xi posInDummy = MatrixXi::Zero(3, 1);
    Matrix3Xf RposDummy = MatrixXf::Zero(3, 1);

    int colsV = velSensors + noVel;
    int colsA = accelSensors + noAccel;
    RowXi velCols(1, colsV);
    RowXi accelCols(1, colsA);

    // Add KFs to LMEKFSuite
    for (int i = 0; i < sensorCombos.size(); i++)
    {
        combo = sensorCombos[i];
        velCols = combo.leftCols(colsV);    // Vel on the left
        accelCols = combo.rightCols(colsA); // Accel on the right

        Matrix3Xi velInNew(3, velCols.sum());
        Matrix3Xf Rvelnew(3, velCols.sum());
        Matrix3Xi accelInNew(3, accelCols.sum());
        Matrix3Xf Raccelnew(3, accelCols.sum());

        // Get required dataIn and R columns
        int col = 0;
        for (int j = 0; j < colsV; j++)
        {
            if (velCols(j))
            {
                velInNew.col(col) = combo.col(j);
                Rvelnew.col(col++) = Rvel.col(j);
            }
        }
        col = 0;
        for (int j = 0; j < colsA; j++)
        {
            if (accelCols(j))
            {
                accelInNew.col(col) = combo.col(colsV + j);
                Raccelnew.col(col++) = Raccel.col(j);
            }
        }

        // Add LMEKF to LMEKFSuite
        LMEKFSuite.push_back(new LinearMotionEKFSuite(states, posInDummy, velInNew, accelInNew,
                                                      RposDummy, Rvelnew, Raccelnew, Q.block(1,0,2,2)));
    }

    /************* World-Frame Position, Velocity, and Acceleration Initialization ******************/
    posMaskw.resize(posIn.rows(), posIn.cols());
    posMaskw = posIn;
    vaMaskw.resize(3,2);
}

// Find all combinations b/w two types of sensors provided
// list = address of std::vector
// numSensors1 = number of sensor1's provided
// numSensors2 = number of sensor2's provided
void LinearMotionEDKF::FindDataCombos(vector<RowXi> &list, int numSensors1, int numSensors2)
{
    // Determine number of sensors and columns needed
    // Add a ones column at the front if numSensors1 = 0, and/or at end if numSensors = 0
    int numSensors = numSensors1 + numSensors2;
    int numCols = numSensors + (int)(numSensors1 == 0) + (int)(numSensors2 == 0);
    int noSensor1 = (int)(numSensors1 == 0); // Sensor 1 not provided

    RowXi combo;
    combo.resize(1, numCols);
    combo.setZero();
    if (numSensors1 == 0)
        combo(0) = 1; // Add a 1 at beginning if numSensors1 = 0
    if (numSensors2 == 0)
        combo(numCols - 1) = 1; // Add a 1 at end if numSensors2 = 0

    // Now create all combos (looking only at the columns that started with 0)
    // numSensors1 on the left, numSensors2 on the right
    // Ex: numSensors1 = numSensors2 = 1 (there are only 3 possible combinations)
    // combinations =
    //      00 (no data, this is not considered an option)
    //      10 (only sensor1 data)
    //      01 (only sensor2 data)
    //      11 (both sensor1 and sensor2 data)
    //      First entry is then removed b/c it's all 0's
    for (int i = 0; i < numSensors; i++)
    {
        if (i == 0)
            list.push_back(combo); // Start with initialized combo row
        int num2add = list.size();
        int index = 0;
        while (index < num2add)
        {
            combo = sensorCombos[index++]; // Copy the (2^i)th earlier entries
            combo.col(i + noSensor1) = 1;      // And change (i+noSensor1)th column to 1
            list.push_back(combo);
        }
    }
    list.erase(sensorCombos.begin()); // Erase first element, since it's all 0's
}

void LinearMotionEDKF::InitLMEDKF(VectorXf Xo)
{
}

// dataMask = row vector indicating which measurements are new
//      Follows same format as when creating all sensor combinations in which the first and/or last
//      column is a 1 depending if there were no vel or accel sensors initialized
// time_step = time step [s] since last call of this update function
// Xpredict = predicted state values
//      Col 0 = X-axis, Col 1 = Y-axis, Col 2 = Z-axis
// Zvel = measurements from vel sensors (must have same number of columns as velIn)
// Zaccel = measurements from accel sensors (must have same number of columns as accelIn)
MatrixX3f LinearMotionEDKF::UpdateLMEDKF(RowXi dataMask, float time_step, Vector3 input_states,
                                    Matrix3Xf Zvel, Matrix3Xf Zaccel)
{
    // Determine which KF to use based on provided dataMask
    int KFindex = 0;
    for (int i = 0; i < sensorCombos.size(); i++)
    {
        if (mask == sensorCombos[i])
        {
            KFindex = i;
            break;
        }
    }

    // Create dummy Zpos matrix
    Matrix3Xf Zpos = MatrixXf::Zero(3, 1);

    // Create A matrices
    // This uses the actual state transition matrix formula
    MatrixXf Anew(2, 6);
    for (int axis = 0; axis < 3; axis++)
        Anew.block(0, axis*2, 2, 2) = LinearMotionEDKF::VelAccelSTM(linearizedDamping(axis), time_step);
    
    // Get new body-frame vel and accel
    MatrixX3f X = LMEKFSuite[KFindex]->UpdateEKFSuite(Xpredict, Zpos, Zvel, Zaccel, Anew);
    return X;
}

// Compute State Transition Matrix (this was computed from e^At)
// b = damping slope
// dt = time step [s]
Matrix2f LinearMotionEDKF::VelAccelSTM(float b, float dt)
{
    Matrix2f A = Matrix2f::Zero();
    A(0, 0) = cos(b * dt);
    A(0, 1) = (1.0 / b) * sin(b * dt);
    A(1, 0) = -b * sin(b * dt);
    A(1, 1) = cos(b * dt);
    return A;
}

// Get rotation matrix from world-frame to body-frame using Euler Angles (roll, pitch, yaw)
Matrix3f LinearMotionEKF::GetRotationRPY2Body(float roll, float pitch, float yaw)
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