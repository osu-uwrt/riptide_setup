#include "riptide_gnc/kalman_filter"

KalmanFilter::KalmanFilter(MatrixXf Ao, MatrixXf Ho, MatrixXf Qo, MatrixXf Ro)
{
    A = Ao;
    H = Ho;
    Q = Qo;
    P = Qo; // Initialize error covariance to process covariance
    R = Ro;
    m = H.rows();
    n = A.rows();
    K.resize(n,m);
    K.setZero();
    I.resize(n,n);
    I.setIdentity();
    init = false;
}

KalmanFilter::Init(VectorXf Xo)
{
    Xhat = Xo;
    init = true;
}

// Update Kalman Filter, assuming linear system
KalmanFilter::UpdateKF(VectorXf Z)
{
    if(!init)
        throw std::runtime_error("Kalman Filter not initialized");
    
    Xhat = A*Xhat;
    P = A*P*A.transpose() + Q;
    K = P*H.transpose() * ((H*P*H.transpose() + R).inverse());
    Xhat = Xhat + K*(Z - H*Xhat);
    P = (I-K*H)*P;
}

// Update Kalman Filter with overridden state vector and system matrices
// To be called by an Extended Kalman Filter (EKF)
// A and H matrices are Jacobians calculated by the EKF
KalmanFilter::UpdateKFOverride(VectorXf Xpredict, Vector Xf Z, MatrixXf Anew, Matrix Hnew)
{
    A = Anew;
    H = Hnew;
    
    P = A*P*A.transpose() + Q;
    K = P*H.transpose() * ((H*P*H.transpose() + R).inverse());
    Xhat = Xpredict + K*(Z - H*Xpredict);
    P = (I-K*H)*P;
}