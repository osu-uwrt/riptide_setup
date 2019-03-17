#include "riptide_gnc/trans_ekf.h"

// NOTE: vel and accel sensors are BODY-FRAME, where pos sensors are WORLD-FRAME
// posIn, velIn, accelIn = matrices indicating which sensors are provided to this EKF
//      Row 0 = X; Row 1 = Y; Row 2 = Z
//      Each column represents data from a different sensor
//      1 = msmt provided, 0 = msmt not provided
// Rp, Rv, and Ra = the main-diagonal elements for the sensor noise covariance matrices
//      Row 0 = X; Row 1 = Y; Row 2 = Z
// Q = horizontal concatenation of process coviariance matrices for each axis (Qx, Qy, and Qz)
//      The first row is for position, the second two rows are for vel and accel
//      For simplicity, the bottom two rows will be used for the body-frame EKFs, but all three rows
//      will be used for world-frame estimation (pos, vel, and accel)
TransEKF::TransEKF(Matrix3Xi posMaskw, Matrix3Xi velMaskbf, Matrix3Xi accelMaskbf,
                   Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, Matrix9Xf Qin)
{
    n = 9;

    // Add masks to sensorMask
    sensorMask.push_back(posMaskw);
    sensorMask.push_back(velMaskbf);
    sensorMask.push_back(accelMaskbf);

    // Add R matrices to Rmat
    Rmat.push_back(Rpos);
    Rmat.push_back(Rvel);
    Rmat.push_back(Raccel);

    // Get Q
    Q = Qin;

    // Initialize sensors and sensorsRed arrays, get number of cols in each mask
    for (int i = 0; i < 3; i++)
    {
        sensors[i] = false;                 // Primary sensors
        sensorsRed[i] = false;              // Redundant sensors
        maskCols[i] = sensorMask[i].cols(); // Number of cols in each mask
        indexRed[i] = maskCols[i];          // Index for redundant data, initialize to maskCols[i]
        axialData[i] = 0;
        axialDataRed[i] = 0;
    }

    // Determine which sensors exist
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < maskCols[i]; j++)
        {
            if (sensorMask[i].col(j).sum() > 0)
            {
                sensors[i] = true;
                sensorsExist = true;
            }
            else if ((sensorMask[i].col(j).sum() == 0) && (j != maskCols[i] - 1))
            {
                // Redundancy separator (column of zeroes) cannot occur in the last column of mask
                sensorsRed[i] = true; // Have redundant sensors
                indexRed[i] = j + 1;  // We want the index where the redundant info BEGINS
            }
        }
    }

    // See which axes the primary and redundant sensor groups provide data for
    int numMsmts[0] = 0, numMsmts[1] = 0;
    for (int i = 0; i < 3; i++)
    {
        if (sensors[i])
        {
            int c = indexRed[i]; // Will equal number of columns if no redundant sensors exist
            axialData[i] = sensorMask[i].block<3, c>(0, 0).sum();
            numMsmts[0] += axialData[i];

            if (sensorsRed[i])
            {
                int c2 = maskCols[i] - c;
                axialDataRed[i] = sensorMask[i].block<3, c2>(0, c).sum();
                numMsmts[1] += axialDataRed[i];
            }
        }
    }
    
    // Create EKFs
    numEKF = (int)(sensorsExist) + (int)(sensorsRed[0] || (sensorsRed[1] || sensorsRed[2]);
    for (int i = 0; i < numEKF; i++)
    {
        Matrix9f A;
        A.setIdentity();

        int m = numMsmts[i];
        MatrixXf H;
        H.resize(m, n);
        H.setZero();

        MatrixXf R;
        R.resize(m, m);
        R.setIdentity();

        EKF.push_back(new KalmanFilter(A, H, R, Q));
    }
}

// Find all combinations b/w two types of sensors provided
// list = address of std::vector to append a new combinations
// numSensors1 = number of sensor1's provided
// numSensors2 = number of sensor2's provided
void TransEKF::FindDataCombos(vector<RowXi> &list, int numSensors1, int numSensors2)
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

void TransEKF::InitLMEDKF(VectorXf Xo)
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
MatrixX3f TransEKF::Update(RowXi dataMask, float time_step, Vector3 input_states,
                                 Matrix3Xf Zvel, Matrix3Xf Zaccel)
{

}