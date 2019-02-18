#include "riptide_gnc/kalman_filter.h"

KalmanFilter::KalmanFilter(const Ref<const MatrixXf> &Ao, const Ref<const MatrixXf> &Ho,
                           const Ref<const MatrixXf> &Qo, const Ref<const MatrixXf> &Ro)
{
    // Verify Parameter Dimensions
    int Arows = Ao.rows(), Acols = Ao.cols(), Hrows = Ho.rows(), Hcols = Ho.cols();
    int Qrows = Qo.rows(), Qcols = Qo.cols(), Rrows = Ro.rows(), Rcols = Ro.cols();
    if (Arows != Acols)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' of size(" << Arows << "," << Acols << ") is not a square matrix" << endl;
        throw std::runtime_error(ss.str());
    }
    if (Qrows != Qcols)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Qo' of size(" << Qrows << "," Qcols << ") is not a square matrix" << endl;
        throw std::runtime_error(ss.str());
    }
    if (Rrows != Rcols)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ro' of size(" << Rrows << "," Rcols << ") is not a square matrix" << endl;
        throw std::runtime_error(ss.str());
    }
    if (Arows != Hcols)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' row_size(" << Arows << ") must match param 'Ho' col_size(" << Hcols << ")" << endl;
        throw std::runtime_error(ss.str());
    }
    if (Hrows != Rrows)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ho' row_size(" << Hrows << ") must match param 'Ro' row_size(" << Rrows << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    // Resize matrices and initialize
    n = Ao.rows();
    m = Ho.rows();
    /*A.resize(n, n);
    H.resize(m, n);
    Q.resize(n, n);
    R.resize(m, m);
    P.resize(n, n);*/
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

void KalmanFilter::InitKF(const Ref<const VectorXf> &Xo)
{
    // Verify Parameter Dimensions
    int Xorows = Xo.rows();
    if (Xorows != n)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::InitKF(...): Param 'Xo' row_size(" << Xorows << ") does not match expected row_size(" << n << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    Xhat = Xo;
    init = true;
}

// Update Kalman Filter, assuming linear system
void KalmanFilter::UpdateKF(const Ref<const VectorXf> &Z)
{
    // Verify Parameter Dimensions
    int Zrows = Z.rows();
    if (Zrows != m)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::UpdateKF(...): Param 'Z' row_size(" << Zrows << ") does not match expected row_size(" << m << ")" << endl;
        throw std::runtime_error(ss.str());
    }

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
}

// Update Kalman Filter with overridden state vector and system matrices
// To be called by an Extended Kalman Filter (EKF)
// A and H matrices are Jacobians calculated by the EKF
void KalmanFilter::UpdateEKF(const Ref<const MatrixXf> &Anew, const Ref<const MatrixXf> &Hnew,
                             const Ref<const VectorXf> &Xpredict, const Ref<const VectorXf> &Z)
{
    // Verify Parameter Dimensions
    int Xrows = Xpredict.rows(), Zrows = Z.rows();
    int Arows = Anew.rows(), Acols = Anew.cols(), Hrows = Hnew.rows(), Hcols = Hnew.cols();

    if (Xrows != n)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::UpdateEKF(...): Param 'Xpredict' row_size(" << Xrows << ") does not match expected row_size(" << n ")" << endl;
        throw std::runtime_error(ss.str());
    }
    if (Zrows != m)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::UpdateEKF(...): Param 'Z' row_size(" << Zrows << ") does not match expected row_size(" << m << ")" << endl;
        throw std::runtime_error(ss.str());
    }
    if ((Arows != n) && (Acols != n)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::UpdateEKF(...): Param 'Anew' of size(" << Arows << "," << Acols << ") does not match expected size(" << n << "," << n << ")" << endl;
        throw std::runtime_error(ss.str());
    }
    if ((Hrows != m) && (Hcols != n))
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::UpdateEKF(...): Param 'Hnew' of size(" << Hrows << "," << Hcols << ") does not match expected size(" << m << "," << n << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    A = Anew;
    H = Hnew;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    Xhat = Xpredict + K * (Z - H * Xpredict);
    P = (I - K * H) * P;
}

void KalmanFilter::GetXhat(Ref<VectorXf> x)
{
    x = Xhat;
}

void KalmanFilter::GetErrorCovariance(Ref<MatrixXf> mat)
{
    mat = P;
}