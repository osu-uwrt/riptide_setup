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
LinearMotionEDKF::LinearMotionEDKF(int drag_order, Vector3f damping, Matrix3Xi posMaskW, Matrix3Xi velMaskBF, Matrix3Xi accelMaskBF,
                                   Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix3Xf Q)
{
    dragOrder = drag_order;
    linearizedDamping = damping;

    /*posMaskW.resize(posIn.rows(), posIn.cols());
    velMaskBF.resize(velIn.rows(), velIn.cols());
    accelMaskBF.resize(accelIn.rows(), accelIn.cols());
    posMaskW = posIn;      // Available world-frame position data
    velMaskBF = velIn;     // Available body-frame velocity data
    accelMaskBF = accelIn; // Available body-frame acceleration data*/
    // Get number of columns of each sensor type
    int colsPos = posMaskW.cols();
    int colsVel = velMaskBF.cols();
    int colsAccel = accelMaskBF.cols();

    // Count number of each type of sensor
    posSensors = 0, velSensors = 0, accelSensors = 0;
    for (int i = 0; i < colsPos; i++)
        if (posMaskW.col(i).sum() > 0)
            posSensors++;
    for (int i = 0; i < colsVel; i++)
        if (velMaskBF.col(i).sum() > 0)
            velSensors++;
    for (int i = 0; i < colsAccel; i++)
        if (accelMaskBF.col(i).sum() > 0)
            accelelSensors++;

    /******************* Body-Frame Velocity and Acceleration Initialization ********************/
    // Find all possible body-frame sensor combinations
    LinearMotionEDKF::FindDataCombos(&sensorCombosBF, velSensors, accelSensors);

    // Create states vector
    Vector3i states;
    states(0) = 0; // Not tracking pos
    states(1) = 1; // Tracking vel
    states(2) = 1; // Tracking accel

    RowXi combo;
    combo.resize(1, colsVel + colsAccel);
    Matrix3Xi posMaskBlank = MatrixXi::Zero(3, 1);
    Matrix3Xf RposBlank = MatrixXf::Zero(3, 1);

    RowXi velCols(1, colsVel);
    RowXi accelCols(1, colsAccel);

    // Add KFs to LMEKFSuite
    for (int i = 0; i < sensorCombosBF.size(); i++)
    {
        combo = sensorCombosBF[i];
        velCols = combo.leftCols(colsVel);      // Vel on the left
        accelCols = combo.rightCols(colsAccel); // Accel on the right

        Matrix3Xi velMaskNew(3, abs(velCols.sum());
        Matrix3Xf Rvelnew(3, abs(velCols.sum());
        Matrix3Xi accelMaskNew(3, abs(accelCols.sum());
        Matrix3Xf Raccelnew(3, abs(accelCols.sum());
        velMaskNew.setZero();
        RvelNew.setZero();
        accelMaskNew.setZero();
        RaccelNew.setZero();

        // Get required dataIn and R columns
        int col = 0;
        for (int j = 0; j < colsVel; j++)
        {
            if (velCols(j) != 0)
            {
                velMaskNew.col(col) = velMaskBF.col(j);
                RvelNew.col(col++) = Rvel.col(j);
            }
        }
        col = 0;
        for (int j = 0; j < colsAccel; j++)
        {
            if (accelCols(j) != 0)
            {
                accelMaskNew.col(col) = accelMaskBF.col(j);
                RaccelNew.col(col++) = Raccel.col(j);
            }
        }

        // Add LMEKF to LMEKFSuite
        LMEKFSuite.push_back(new LinearMotionEKFSuite(states, posMaskBlank, velMaskNew, accelMaskNew,
                                                      RposBlank, RvelNew, RaccelNew, Q.block(1, 0, 2, 2)));
    }

    /************* World-Frame Position, Velocity, and Acceleration Initialization ******************/
    // Velocity and acceleration are now bundled together since they both come from the LMEKFSuite
    vaMaskW.resize(3, 2);
    vaMaskW.col(0) << 1, 1, 1; // Have X, Y, and Z vel/accel
    vaMaskW.col(1) << 1, 1, 0; // Have X and Y vel/accel (in case the Z axis was not updated)

    // Find all possible world-frame sensor combinations
    LinearMotionEDKF::FindDataCombos(&sensorCombosW, posSensors, 2);

    // Update states vector
    states(0) = 1; // Tracking pos
    states(1) = 1; // Tracking vel
    states(2) = 1; // Tracking accel

    combo.resize(1, colsPos + 2); // No need to declare a new one
    RowXi posCols(1, colsPos);
    RowXi vaCols(1, 2);

    // Create new R matrices for vel and accel for cascaded EKF ?????????????????????????????
    Matrix3Xf RvelW(3,2), RaccelW(3,2);
    RvelW.setZero();
    RaccelW.setZero();

    // Add KFs to PoseEKFSuite
    for (int i = 0; i < sensorCombosW.size(); i++)
    {
        combo = sensorCombosW[i];
        posCols = combo.leftCols(colsPos); // Pos on the left
        vaCols = combo.rightCols(2);       // Vel/accel on the right

        Matrix3Xi posMaskNew(3, abs(posCols.sum());
        Matrix3Xf RposNew(3, abs(posCols.sum());
        Matrix3Xi vaMaskNew(3, abs(vaCols.sum());
        Matrix3Xf RvelNew(3, abs(vaCols.sum()); // ?????????????????
        Matrix3Xf RaccelNew(3, abs(vaCols.sum()); // ???????????????
        posMaskNew.setZero();
        RposNew.setZero();
        vaMaskNew.setZero();
        RvelNew.setZero();
        RaccelNew.setZero();

        /*// Get required dataIn and R columns
        int col = 0;
        for (int j = 0; j < colsVel; j++)
        {
            if (velCols(j) != 0)
            {
                velMaskNew.col(col) = velMaskBF.col(j);
                Rvelnew.col(col++) = Rvel.col(j);
            }
        }
        col = 0;
        for (int j = 0; j < colsAccel; j++)
        {
            if (accelCols(j) != 0)
            {
                accelMaskNew.col(col) = accelMaskBF.col(j);
                Raccelnew.col(col++) = Raccel.col(j);
            }
        }

        // Add LMEKF to LMEKFSuite
        PoseEKFSuite.push_back(new LinearMotionEKFSuite(states, posMaskBlank, velMaskNew, accelMaskNew,
                                                      RposBlank, Rvelnew, Raccelnew, Q.block(1, 0, 2, 2)));*/
    }
}

// Find all combinations b/w two types of sensors provided
// list = address of std::vector to append a new combinations
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
        combo(0) = -1; // Add a -1 at beginning if numSensors1 = 0
    if (numSensors2 == 0)
        combo(numCols - 1) = -1; // Add a 1 at end if numSensors2 = 0

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
            combo = list[index++];        // Copy the (2^i)th earlier entries
            combo.col(i + noSensor1) = 1; // And change (i+noSensor1)th column to 1
            list.push_back(combo);
        }
    }
    list.erase(list.begin()); // Erase first element, since it's all 0's
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
        Anew.block(0, axis * 2, 2, 2) = LinearMotionEDKF::VelAccelSTM(linearizedDamping(axis), time_step);

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