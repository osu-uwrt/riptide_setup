#include "riptide_gnc/linear_motion_edkf.h"

LinearMotionEDKF::LinearMotionEDKF(Matrix3Xi velIn, Matrix3Xi accelIn,
                                   Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix2Xf Q)
{
    velData.resize(velIn.rows(), velIn.cols());
    accelData.resize(accelIn.rows(), accelIn.cols());
    velData = velIn;     // Available velocity data
    accelData = accelIn; // Available acceleration data
    int colsVel = velData.cols();
    int colsAccel = accelData.cols();

    // Count number of each type of sensor
    velSensors = 0, accelSensors = 0;
    for (int i = 0; i < colsVel; i++)
        if (velData.col(i).sum() > 0)
            velSensors++;
    for (int i = 0; i < colsAccel; i++)
        if (accelData.col(i).sum() > 0)
            accelelSensors++;

    // Find all possible sensor combinations
    int numSensors = velSensors + accelSensors;
    int noVel = (int)(velSensors == 0);
    int noAccel = (int)(accelSensors == 0);
    LinearMotionEDKF::FindDataCombos(velSensors, accelSensors);

    // Create states vector
    Vector3i states;
    states(0) = 0; // No pos
    states(1) = 1; // Yes vel
    states(2) = 1; // Yes accel

    RowXi combo;
    combo.resize(1, numSensors + noVel + noAccel);
    Matrix3Xi posIn = MatrixXi::Zero(3, 1);
    Matrix3Xf Rpos = MatrixXf::Zero(3, 1);

    int colsV = velSensors + noVel;
    int colsA = accelSensors + noAccel;
    RowXi velCols(1, colsV);
    RowXi accelCols(1, colsA);

    // Add KFs to KFSuite
    for (int i = 0; i < sensorCombos.size(); i++)
    {
        combo = sensorCombos[i];
        velCols = combo.leftCols(colsV); // Vel on the left
        accelCols = combo.rightCols(colsA); // Accel on the right

        Matrix3Xi velInNew(3, velCols.sum());
        Matrix3Xf Rvelnew(3, velCols.sum());
        Matrix3Xi accelInNew(3, accelCols.sum());
        Matrix3Xf Raccelnew(3, accelCols.sum());
        
        // Get required dataIn and R columns
        int col = 0;
        for (int j = 0; j < velCols.cols(); j++)
        {
            if (velCols(j))
            {
                velInNew.col(col) = velData.col(col);
                Rvelnew.col(col) = Rvel.col(col++);
            }
        }
        col = 0;
        for (int j = 0; j < accelCols.cols(); j++)
        {
            if (accelCols(j))
            {
                accelInNew.col(col) = accelData.col(col);
                Raccelnew.col(col) = Raccel.col(col++);
            }
        }

        // Add LMEKF to LMEKFSuite
        LMEKFSuite.push_back(new LinearMotionEKFSuite(states, posIn, velInNew, accelInNew,
                                                Rpos, Rvelnew, Raccelnew, Q));

        
    }
}

void LinearMotionEDKF::FindDataCombos(int velSensors, int accelSensors)
{
    // Determine number of sensors and columns needed
    // Add a ones column at the front if velSensors = 0, and/or at end if accelSensors = 0
    int numSensors = velSensors + accelSensors;
    int numCols = numSensors + (int)(velSensors == 0) + (int)(accelSensors == 0);
    int noVel = (int)(velSensors == 0);

    RowXi combo;
    combo.resize(1, numCols);
    combo.setZero();
    if (velSensors == 0)
        combo(0) = 1; // Add a 1 at beginning if velSensors = 0
    if (accelSensors == 0)
        combo(numCols - 1) = 1; // Add a 1 at end if accelSensors = 0

    // Now create all combos (looking only at the columns that started with 0)
    // Vel on the left, Accel on the right
    // Ex: velSensors = accelSensors = 1 (there are only 3 possible combinations)
    // sensorCombos =
    //      00 (no data, this is not considered an option)
    //      10 (only vel data)
    //      01 (only accel data)
    //      11 (both vel and accel data)
    //      First entry is then removed b/c it's all 0's
    for (int i = 0; i < numSensors; i++)
    {
        if (i == 0)
            sensorCombos.push_back(combo); // Start with initialized combo row
        int num2add = sensorCombos.size();
        int index = 0;
        while (index < num2add)
        {
            combo = sensorCombos[index++]; // Copy the (2^i)th earlier entries
            combo.col(i + noVel) = 1;      // And change (i+noVel)th column to 1
            sensorCombos.push_back(combo);
        }
    }
    sensorCombos.erase(sensorCombos.begin()); // Erase first element, since it's all 0's
}

void LinearMotionEDKF::InitLMEDKF(VectorXf Xo)
{
}

void LinearMotionEDKF::UpdateLMEDKF(float time_step, Matrix3Xf Zvel, Matrix3Xf Zaccel)
{
}