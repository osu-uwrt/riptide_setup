#ifndef AUV_MATH_LIB
#define AUV_MATH_LIB

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;

// Useful math tools
namespace AUVMathLib
{
Matrix3f GetAxisRotation(int axis, float angle);

Matrix3f GetEulerRotMat(const Ref<const Vector3f> &attitude);

MatrixXf SgnMat(const Ref<const MatrixXf>& mat);

Matrix3f SkewSym(const Ref<const Vector3f> &v);
} // namespace AUVMathLib

#endif