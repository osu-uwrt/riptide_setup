#include "riptide_gnc/linear_motion_ekf_suite.h"

// kf_states = column vector indicating which variables are states (pos, vel, and/or accel)
//      Row 0 = pos; Row 1 = vel; Row 2 = accel
//      1 = state included, 0 = state not included
// posIn, velIn, accelIn = matrices indicating which measurements are provided to this EKF
//      Row 0 = X; Row 1 = Y; Row 2 = Z
//      Each column represents data from a different sensor
//      1 = msmt provided, 0 = msmt not provided
// Rpos, Rvel, and Raccel = the main-diagonal elements for the sensor noise covariance matrices
//      Row 0 = X; Row 1 = Y; Row 2 = Z
// Q = horizontal concatenation of process coviariance matrices for each axis (Qx, Qy, and Qz)
LinearMotionEKFSuite::LinearMotionEKFSuite(Vector3i kf_states, Matrix3Xi posIn, Matrix3Xi velIn, Matrix3Xi accelIn,
                                           Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, MatrixXf Q)
{
    states = kf_states;
    n = states.sum();

    posData.resize(posIn.rows(), posIn.cols());
    velData.resize(velIn.rows(), velIn.cols());
    accelData.resize(accelIn.rows(), accelIn.cols());
    posData = posIn;     // Available position data
    velData = velIn;     // Available velocity data
    accelData = accelIn; // Available acceleration data
    colsPos = posData.cols();
    colsVel = velData.cols();
    colsAccel = accelData.cols();
    int maxCols = max(colsPos, max(colsVel, colsAccel));

    // Check for basic matrix size requirements
    /*if (colsPos != Rpos.cols())
        throw std::runtime_error("Dimension mismatch: row size of posIn and Rpos do not match");
    if (colsVel != Rvel.cols())
        throw std::runtime_error("Dimension mismatch: row size of velIn and Rvel do not match");
    if (colsAccel != Raccel.cols())
        throw std::runtime_error("Dimension mismatch: row size o accelIn and Raccel do not match");
    if (numStates != Q.rows())
        throw std::runtime_error("Dimension mismatch: column size of Q does not match desired number of states");
    if (Q.cols() != numStates * 3)
        throw std::runtime_error("Dimension mismatch: Q must have row size equal to (number of states * 3)");*/

    for (int axis = 0; axis < 3; axis++) // One axis at a time (X, Y, then Z)
    {
        int colP = 0, colV = 0, colA = 0;

        activeKFPerAxis[axis] = 0;
        maxMsmtsPerAxis[axis] = 0;
        vector<int> msmtsPerKF;
        vector<bool> dontSkip;

        for (int i = 0; i < maxCols; i++) // Match columns one-by-one
        {
            int numMsmts = posData(axis, colP) + velData(axis, colV) + accelData(axis, colA); // At most 3

            if (numMsmts > 0) // Add KF
            {
                // Create A and H matrices
                MatrixXf A(n, n);
                MatrixXf H(numMsmts, n);
                A.setZero();
                H.setZero();

                // Create R matrix
                MatrixXf R(numMsmts, numMsmts);
                R.setZero();
                int Rindex = 0;
                if (posData(axis, colP))
                    R(Rindex, Rindex++) = Rpos(axis, colP);
                if (velData(axis, colV))
                    R(Rindex, Rindex++) = Rvel(axis, colV);
                if (accelData(axis, colA))
                    R(Rindex, Rindex++) = Raccel(axis, colA);

                // Add to KFSuite
                KFSuite.push_back(new KalmanFilter(A, H, Q.block(n, n, 0, axis * n), R));
                activeKFPerAxis[axis] += 1;     // 1 more potential active KF
                msmtsPerKF.push_back(numMsmts); // Add num. msmts

                // Update max number of measurements per axis
                if (numMsmts > maxMsmtsPerAxis[axis])
                    maxMsmtsPerAxis[axis] = numMsmts;
            }
            else
            {
                msmtsPerKF.push_back(0);
            }

            // Update current column of posData, velData, and accelData
            // If thre is only one column, this will always return that column
            colP = (colP + 1) % colsPos;
            colV = (colV + 1) % colsVel;
            colA = (colA + 1) % colsAccel;

            dontSkip.push_back(false); // For now, every KF will be used (will be updated later)
        }

        // Puch back msmtsPerKF to msmtsPerAxis
        msmtsPerAxis.push_back(msmtsPerKF);
        msmtsPerKF.clear();

        // Push back dontSkip to SkipKF
        skipKF.push_back(dontSkip);
        dontSkip.clear();
    }

    // Search through msmtsPerAxis and indicate if any KFs should be skipped
    // Only KFs using the max number of provided msmts should be used, all others should be skipped
    for (int axis = 0; axis < 3; axis++)
    {
        for (int i = 0; i < maxCols; i++)
        {
            if (msmtsPerAxis[axis][i] < maxMsmtsPerAxis[axis])
            {
                skipKF[axis][i] = true;     // Skip KF b/c another KF has more measurement inputs
                activeKFPerAxis[axis] -= 1; // One less active KF per axis
            }
        }
    }
}

// Update Linear Motion EKF Suite
// Xpredict = concatenation of time-predictions for each axis (up to 3 rows, if necessary)
//            Row 0 = state1 prediction, Row 1 = state2 prediction, Row 2 = state3 prediction
//            Col 0 = X-axis, Col 1 = Y-axis, Col 2 = Z-axis
// Z = horizontal concatenation of posIn, velIn, and accelIn where each "1" is replaced with the actual msmt)
// Anew = horizontal concatenation of each KF's A matrix (Ax, Ay, Az)
MatrixX3f LinearMotionEKFSuite::UpdateEKFSuite(MatrixX3f Xpredict, Matrix3Xf Z, MatrixXf Anew)
{
    /*if (Z.cols() != (colsPos + colsVel + colsAccel))
        throw std::runtime_error("Dimension mismatch: row size of Z does not match expected row size");*/

    MatrixX3f X(n, 3);
    X.setZero();
    int KFindex = -1;

    for (int axis = 0; axis < 3; axis++) // One axis at a time
    {
        int colP = 0, colV = 0, colA = 0;
        int maxCols = max(colsPos, max(colsVel, colsAccel));

        VectorXf state(n);
        MatrixXf Pinverse(n, n);
        vector<VectorXf > states; // Vector for all state estimates for this axis
        vector<MatrixXf > Pinverses; // Vector of all process covariance inverses
        MatrixXf overallInverse(n, n); // May or may not be used
        state.setZero();
        Pinverse.setZero();
        overallInverse.setZero();

        for (int i = 0; i < maxCols; i++) // Match columns one-by-one (same format as above)
        {
            // Keep track of KF index
            int numMsmts = posData(axis, colP) + velData(axis, colV) + accelData(axis, colA);
            if (numMsmts > 0)
                KFindex++;

            if (!skipKF[axis][i]) // This KF is required
            {
                // Create vector indicating available measurements
                Vector3i msmts = Vector3i::Zero();
                msmts(0) = posData(axis, colP);
                msmts(1) = velData(axis, colV);
                msmts(2) = accelData(axis, colA);

                // Create Z vector for current KF
                VectorXf Znew(numMsmts);
                Znew.setZero();
                int Zindex = 0;
                if (posData(axis, colP))
                    Znew(Zindex++) = Z(axis, colP);
                if (velData(axis, colV))
                    Znew(Zindex++) = Z(axis, colsPos + colV);
                if (accelData(axis, colA))
                    Znew(Zindex++) = Z(axis, colsPos + colsVel + colA);

                // Create H matrix for current KF
                MatrixXf Hnew(numMsmts, n);
                Hnew.setZero();
                int Hrow = 0, Hcol = 0;
                for (int j = 0; j < 3; j++)
                {
                    if (state(j) && msmts(j)) // This state is measured
                        Hnew(Hrow++, Hcol++) = 1;
                    else if (state(j)) // This state is not measured
                        Hcol++;
                }

                state = KFSuite[KFindex]->UpdateKFOverride(Xpredict.col(axis), Znew,
                                                   Anew.block(n, n, 0, axis * n), Hnew);
                Pinverse = KFSuite[KFindex]->GetProcessCovariance().inverse();

                states.push_back(state);
                Pinverses.push_back(Pinverse);
                overallInverse = overallInverse + Pinverse; // Sum all inverses
            }

            // Update current column of posData, velData, and accelData
            colP = (colP + 1) % colsPos;
            colV = (colV + 1) % colsVel;
            colA = (colA + 1) % colsAccel;
        }

        // Output final state estimate for this axis
        if (activeKFPerAxis[axis] > 1) // Have multiple sensors recording the same data
        {
            // Final state estimate is a weighting of the process covariance inverses
            overallInverse = overallInverse.inverse(); // Inverse of the summation of inverses
            for (int j = 0; j < activeKFPerAxis[axis]; j++)
            {
                X.col(axis) = X.col(axis) + (Pinverses[j] * states[j]); // Sum each inverse times its state
            }
            X.col(axis) = overallInverse * X.col(axis); // Then pre-multiply by overallInverse
        }
        else 
            X.col(axis) = state;
    }

    return X;
}