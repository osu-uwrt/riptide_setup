#include "riptide_gnc/linear_motion_ekf_suite.h"

// dataAvail = matrix indicating which measurements are provided to this EKF (1 indicates provided, 0 otherwise)
//             Row 0 = X; Row 1 = Y; Row 2 = Z
//             Col 0 = abs. pos; Col 1 = body-frame vel; Col 2 = body-frame accel
// Rpos, Rvel, and Raccel contain the main-diagonal elements for the sensor noise covariance matrices
// Qx, Qy, and Qz are the process coviariance matrices for each axis
LinearMotionEKFSuite::LinearMotionEKFSuite(Vector3i kf_states, Matrix3Xi posIn, Matrix3Xi velIn, Matrix3Xi accelIn,
                                           Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, MatrixXf Q)
{
    states = kf_states;
    n = states.sum();

    posData.resize(posIn.rows(), posIn.cols());
    velData.resize(velIn.rows, velIn.cols());
    accelData.resize(accelIn.rows, accelIn.cols());
    posData = posIn;     // Available position data
    velData = velIn;     // Available velocity data
    accelData = accelIn; // Available acceleration data
    colsPos = posData.cols();
    colsVel = velData.cols();
    colsAccel = accelData.cols();

    // Check for basic matrix size requirements
    if (colsPos != Rpos.cols())
        throw std::runtime_error("Dimension mismatch: row size of posIn and Rpos do not match");
    if (colsVel != Rvel.cols())
        throw std::runtime_error("Dimension mismatch: row size of velIn and Rvel do not match");
    if (colsAccel != Raccel.cols())
        throw std::runtime_error("Dimension mismatch: row size o accelIn and Raccel do not match");
    if (numStates != Q.rows())
        throw std::runtime_error("Dimension mismatch: column size of Q does not match desired number of states");

    for (int axis = 0; axis < 3; axis++) // One axis at a time
    {
        int colP = 0, colV = 0, colA = 0;
        int maxCols = max(colsPos, max(colsVel, colsAccel));
        numKFPerAxis[axis] = 0;
        for (int k = 0; k < mmaxCols; i++) // Match columns one-by-one
        {
            int numMsmts = posIn(axis, colP) + velIn(axis, colV) + accelIn(axis, colA); // At most 3
            int Rindex = 0;

            // Create A and H matrices
            MatrixXf A(n, n);
            MatrixXf H(numMsmts, n);
            A.setZero();
            H.setZero();

            // Create R matrix
            MatrixXf R(numMsmts, numMsmts);
            R.setZero();
            if (posIn(axis, colP))
                R(Rindex, Rindex++) = Rpos(axis, colP);
            if (velIn(axis, colV))
                R(Rindex, Rindex++) = Rvel(axis, colV);
            if (accelIn(axis, colA))
                R(Rindex, Rindex++) = Raccel(axis, colA);

            // Add to KFSuite
            KFSuite.push_back(new KalmanFilter(A, H, Q.block(n, n, 0, axis * n), R));
            numKFPerAxis[axis] += 1;

            // Update current column of posIn, velIn, and accelIn
            colP = (colP + 1) % colsPos;
            colV = (colV + 1) % colsVel;
            colA = (colA + 1) % colsAccel;
        }
    }
}

// Update Linear Motion EKF Suite
// Xpredict = concatenation of time-predictions for each axis
//            Row 0 = abs. pos, Row 1 = abs. vel, Row 2 = abs. accel
//            Col 0 = X-axis KF, Col 1 = Y-axis KF, Col 2 = Z-axis KF
// attitude = roll, pitch, yaw [rad]
// Z = matrix of measurements (same order as for dataAvail in constructor)
// Anew = concatenation of each KF's A matrix (Ax, Ay, Az)
MatrixX3f LinearMotionEKFSuite::UpdateEKFSuite(MatrixX3f Xpredict, Matrix3Xf Z, MatrixXf Anew)
{
    MatrixX3f X(states.sum(), 3);

    for (int axis = 0; axis < 3; axis++) // One axis at a time
    {
        int colP = 0, colV = 0, colA = 0;
        int maxCols = max(colsPos, max(colsVel, colsAccel));

        for (int k = 0; k < mmaxCols; i++) // Match columns one-by-one (same format as above)
        {
            int numMsmts = posData(axis, colP) + velData(axis, colV) + accelData(axis, colA);
            Vector3i msmts = Vector3i::Zero();
            msmts(0) = posData(axis, colP);
            msmts(1) = velData(axis, colV);
            msmts(2) = accelData(axis, colA);
            int Zindex = 0;

            // Create Z vector for current KF
            VectorXf Znew(numMsmts);

            Znew.setZero();

            if (posData(axis, colP))
                Znew(Zindex++) = Z(axis, colP);
            if (velData(axis, colV))
                Znew(Zindex++) = Z(axis, colsPos + colV);
            if (accelData(axis, colA))
                Znew(Zindex++) = Z(axis, colsPos + colsVel + colA);

            // Create H matrix for current KF
            MatrixXf Hnew(numMsmts, states.sum());
            Hnew.setZero();
            int Hrow = 0, Hcol = 0, temp = 0;
            for (int i = 0; i < 3; i++)
            {
                if (state(i) && msmts(i))
                {
                    Hcol += temp;
                    Hnew(Hrow++, Hcol) = 1;
                    temp = 1;
                }
                else if (state(i))
                    temp++;
            }

            // Update current column of posData, velData, and accelData
            colP = (colP + 1) % colsPos;
            colV = (colV + 1) % colsVel;
            colA = (colA + 1) % colsAccel;
        }
    }
}