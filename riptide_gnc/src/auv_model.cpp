#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, float V, Matrix32f drag, vector<float[5]> auv_thrusters);
{
    mass = m;
    volume = V;
    dragCoeffs = drag;
    thrusters = auv_thrusters;
}

VectorXf AUVModel::DecomposeActuation(VectorXf actuation)
{
    VectorXf output;
    output.resize(6, 1);
    output.setZero();

    if (actuation.rows() == thrusters.size())
    {
        for (int i = 0; i < thrusters.size(); i++)
        {
            output
        }
    }
    return output;
}

VectorXf AUVModel::DecomposeActuator(int thruster, float force)
{
    VectorXf output;
    output.resize(6, 1);
    output.setZero();

    if (thruster < thrusters.size())
    {
        float rx = thrusters[thruster][0];
        float ry = thrusters[thruster][1];
        float rz = thrusters[thruster][2];
        float psi = thrusters[thruster][3];
        float theta = thrusters[thruster][4];
        
        output(0) = force * cos(psi) * cos(theta);
        output(1) = force * sin(psi) * cos(theta);
        output(2) = -force * sin(theta);
    }
    return decomposition;
}