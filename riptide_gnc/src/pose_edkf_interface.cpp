#include "riptide_gnc/pose_edkf_interface.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "pose_edkf");
 PoseEDKFInterface poseEDKF;
 ros::spin();
}

PoseEDKFInterface::PoseEDKFInterface() : nh("pose_edkf")
{
    int n = 2;
    int m = 3;
    vector<MatrixXi> mat;
    MatrixXi m1(n,m);
    m1.setZero();
    MatrixXi m2(m,n);
    m2.setOnes();

    mat.push_back(m1);
    mat.push_back(m2);
    cout << "m1: \n" << mat[0] << endl;
    cout << "m2: \n" << mat[1] << endl;

    Matrix3i m3 = Matrix3i::Ones();
    m1.row(0) << 1, 1, 1;
    cout << "m3: \n" << m3 << endl;
    cout << "Replaced row 1 with m3 in m1: \n" << m1 << endl;

    Matrix3Xf posIn = MatrixXf::Zero(3, 1);
    cout << "posIn \n" << posIn << endl;
    
}