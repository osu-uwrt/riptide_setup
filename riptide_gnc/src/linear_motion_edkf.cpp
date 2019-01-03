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
    combo.resize(1, numSensors);
    Matrix3Xf posIn = MatrixXf::Zero(3, 1);

    int colsV = velSensors + noVel;
    int colsA = accelSensors + noAccel;

    // Add KFs to KFSuite
    for (int i = 0; i < sensorCombos.size(); i++)
    {
        combo = sensorCombos[i];
        int velCols = combo.leftCols(colsV);
        int accelCols = combo.rightCols(colsA);

        Matrix3Xf Rvelnew(3, velCols.sum());
        Matrix3Xf Raccelnew(3, accelCols.sum());
        int col = 0;

        // Place required R vectors into a vector
        for (int j = 0; j < velCols.cols(); j++)
            if (velCols(j))
                Rvel_vector.push_back(Rvel.col(j));
        for (int j = 0; j < accelelCols.cols(); j++)
            if (accelCols(j))
                Raccel_vector.push_back(Raccel.col(j));
    }
}

void LinearMotionEDKF::FindDataCombos(int velSensors, int accelSensors)
{
    // Determine number of sensors and columns needed
    // Add a zero column at the front if velSensors = 0, and/or at end if accelSensors = 0
    int numSensors = velSensors + accelSensors;
    int numCols = numSensors + (int)(velSensors == 0) + (int)(accelSensors == 0);
    int noVel = (int)(velSensors == 0);

    RowXi combo;
    combo.resize(1, numCols);
    combo.setZero();
    if (velSensors == 0)
        combo(0) = 1;
    if (accelSensors == 0)
        combo(numCols - 1) = 1;

    for (int i = 0; i < numSensors; i++)
    {
        if (i == 0)
            sensorCombos.push_back(combo); // Start with an entry of all 0's
        int num2add = sensorCombos.size();
        int index = 0;
        while (index < num2add)
        {
            combo = sensorCombos[index++]; // Copy the (2^i)th earlier entries
            combo.row(i + noVel) = 1;      // And change ith column to 1
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