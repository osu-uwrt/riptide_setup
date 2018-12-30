#include "riptide_gnc/pose_ekf_suite.h"

// dataAvail = matrix indicating which measurements are provided to this EKF (1 indicates provided, 0 otherwise)
//             Row 0 = X; Row 1 = Y; Row 2 = Z
//             Col 0 = absolute pos; Col 1 = body-frame vel; Col 2 = body-frame accel
// Rpos, Rvel, and Raccel contain the main-diagonal elements for the sensor noise covariance matrices
// Qx, Qy, and Qz are the process-noise coviariance matrices for each axis
PoseEKFSuite::PoseEKFSuite(Matrix3i dataAvail, Vector3f Rpos, Vector3f Rvel, Vector3f Raccel,
                           Matrix3f Qx, Matrix3f Qy, Matrix3f Qz);
{
    dataMask = dataAvail;
    Vector3i posData = dataMask.col(0);
    MatrixXi bfData(3,2);
    bfData = dataMask.rightCols(2)
    int relMsmts = relData.sum(); // Number of body-frame measurements
    
    // Create vector of process noise covariance matrices (will make it easier to create KFs)
    vector<Matrix3f> Qmatrices;
    Qmatrices.push_back(Qx);
    Qmatrices.push_back(Qy);
    Qmatrices.push_back(Qz);

    // Deterine which KFs need to be created (X, Y, and/or Z)
    for (int i = 0; i < 3; i++) // Check for absolute position measurements
        needKF(i) = (bool)dataMask(i,0);
    if (bfData.sum() > 0) // Check for relative measurements
        for (int i = 0; i < 3; i++)
            needKF(i) = true;
    
    // Create required KFs and initiailize w/A, H, Q, and R matrices
    // The actual A and H matrices will be calculated by the Jacobian
    Matrix3f A = Matrix3f::Zero(); // Create A matrix
    MatrixXf Rbf(Rvel.rows() + Raccel.rows(),1);
    Rbf << Rvel, Raccel; // Concatenate all body-frame R-vectors
    for (int i = 0; i < 3; i++)
    {
        int numMsmts = 0, Rindex = 0;

        if (needKF(i))
        {
            numMsmts = posData(i) + relMsmts;
            MatrixXf H(numMsmts, 3); // Create H matrix
            H.setZero();

            MatrixXf R(numMsmts, numMsmts); // Create R matrix
            if (posData(i))
                R(Rindex,Rindex++) = Rpos(i);
            for (int k = 0; k < bfData.cols(); k++) 
                    for (int j = 0; j < bfData.rows(); j++)
                        if (bfData(j,k)) // All body-frame measurements are needed
                            R(Rindex, Rindex++) = Rbf(k*relData.rows() + j);

            // Add new KF to KFSuite
            KFSuite.push_back(new KalmanFilter(A, H, Qmatrices(i), R));
        }
    }
}

void PoseEKFSuite::UpdatePoseEKF(Vector3f Xpredict, Matrix3f data, Matrix3f Anew, Vector3f attitude)
{

}

void PoseEKFSuite::UpdateEKFX()
{

}

void PoseEKFSuite::UpdateEKFY()
{

}

void PoseEKFSuite::UpdateEKFZ()
{

}