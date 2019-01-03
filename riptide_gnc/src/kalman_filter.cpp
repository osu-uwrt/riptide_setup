#include "riptide_gnc/kalman_filter.h"

KalmanFilter::KalmanFilter(MatrixXf Ao, MatrixXf Ho, MatrixXf Qo, MatrixXf Ro)
{
    // Verify matrix dimensions
    int Arows = Ao.rows(), Acols = Ao.cols(), Hrows = Ho.rows(), Hcols = Ho.cols();
    int Qrows = Qo.rows(), Qcols = Qo.cols(), Rrows = Ro.rows(), Rcols = Ro.cols();
    /*if (Arows != Acols)
        throw std::runtime_error("Dimension mismatch: Ao of size(%i, %i) is not a square matrix", Arows, Acols);
    if (Qrows != Qcols)
        throw std::runtime_error("Dimension mismatch: Qo of size(%i, %i) is not a square matrix", Qrows, Qcols);
    if (Rrows != Rcols)
        throw std::runtime_error("Dimension mismatch: Ro of size(%i, %i) is not a square matrix", Rrows, Rcols);
    if (Arows != Hcols)
        throw std::runtime_error("Dimension mismatch: Ao of row_size(%i) must match Ho of col_size(%i)", Arows, Hcols);
    if (Hrows != Rrows)
        throw std::runtime_error("Dimension mismatch: Ho of row_size(%i) must match Ro of row_size(%i)", Hrows, Rrows);*/

    // Resize matrices and initialize
    m = Hrows;
    n = Arows;
    A.resize(n, n);
    H.resize(m, n);
    Q.resize(n, n);
    R.resize(m, m);
    P.resize(n, n);
    A = Ao;
    H = Ho;
    Q = Qo;
    P = Qo; // Initialize error covariance to process covariance
    R = Ro;

    K.resize(n, m);
    K.setZero();
    I.resize(n, n);
    I.setIdentity();
    init = false;
}

void KalmanFilter::InitKF(VectorXf Xo)
{
    int in_rows = Xo.rows();
    /*if (in_rows != n)
        throw std::runtime_error("Dimension mismatch: Input initial state row_size(%i) does not match expcted row_size(%i)", in_rows, n);*/
    
    Xhat = Xo;
    init = true;
}

// Update Kalman Filter, assuming linear system
VectorXf KalmanFilter::UpdateKF(VectorXf Z)
{
    // Verify input vector dimensions
    int Zrows = Z.rows();
    /*if (Zrows != m)
        throw std::runtime_error("Dimension mismatch: Input measurement row_size(%i) does not match expected row_size(%i)", Zrows, m);*/
    if (!init)
    {
        init = true;
        Xhat.setZero();
    }

    Xhat = A * Xhat;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    Xhat = Xhat + K * (Z - H * Xhat);
    P = (I - K * H) * P;
    return Xhat;
}

// Update Kalman Filter with overridden state vector and system matrices
// To be called by an Extended Kalman Filter (EKF)
// A and H matrices are Jacobians calculated by the EKF
VectorXf KalmanFilter::UpdateKFOverride(VectorXf Xpredict, VectorXf Z, MatrixXf Anew, MatrixXf Hnew)
{
    // Verify input matrix dimensions
    int Xrows = Xpredict.rows(), Zrows = Z.rows();
    int Arows = Anew.rows(), Acols = Anew.cols(), Hrows = Hnew.rows(), Hcols = Hnew.cols();
    /*if (Xrows != n)
        throw std::runtime_error("Dimension mismatch: Input state row_size(%i) does not match expected row_size(%i)", Xrows, n);
    if (Zrows != m)
        throw std::runtime_error("Dimension mismatch: Input measurement row_size(%i) does not match expected row_size(%i)", Zrows, m);
    if ((Arows != n) && (Acols != n)
        throw std::runtime_error("Dimension mismatch: Input A matrix of size(%i, %i) does not match expected size(%i, %i)", Arows, Acols, n, n);
    if ((Hrows != m) && (Hcols != n))
        throw std::runtime_error("Dimension mismatch: Input H matrix of size(%i, %i) does not match expected size(%i, %i)", Hrows, Hcols, m, n);*/

    A = Anew;
    H = Hnew;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    Xhat = Xpredict + K * (Z - H * Xpredict);
    P = (I - K * H) * P;
    return Xhat;
}

MatrixXf KalmanFilter::GetProcessCovariance()
{
    return P;
}