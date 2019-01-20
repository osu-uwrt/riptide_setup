#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;

// Basic Kalman Filter
// If not initialized manually, then it will auto-initialize (set Xhat prediction to zero-vector).
class KalmanFilter
{
private:
    float m, n; // m = # measurements, n = # states
    VectorXf Xhat; // State Vector
    MatrixXf A; // State-transition Matrix
    MatrixXf H; // Measurement Matrix
    MatrixXf K; // Kalman Gain
    MatrixXf P; // Error Covariance Matrix
    MatrixXf Q; // Process Noise Covariance Matrix
    MatrixXf R; // Measurement Noise Covariance Matrix
    MatrixXf I; // Identity Matrix
    bool init;
public:
    KalmanFilter(MatrixXf Ao, MatrixXf Ho, MatrixXf Qo, MatrixXf Ro);
    void InitKF(VectorXf Xo);
    VectorXf UpdateKF(VectorXf Z);
    VectorXf UpdateEKF(MatrixXf Anew, MatrixXf Hnew, VectorXf Xpredict, VectorXf Z);
    MatrixXf GetErrorCovariance();
};

#endif