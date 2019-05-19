#ifndef TRAPEZOIDAL_MOTION
#define TRAPEZOIDAL_MOTION

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

class TrapezoidalMotion
{
private:
    float start, destination, cruiseSpeed, accel;
    int accelSeq;
    float t1, t2, tMid, tFinal;
    bool accelerate;
public:
    TrapezoidalMotion(float startPos, float travelSpeed);
    TrapezoidalMotion(float startPos, float travelSpeed, bool accel);
    void SetAccel(float accel, int seq);
    float GetTotalTime();
    void SetDestination(float dest);
    void ComputeTravelInfo();
    Vector2f ComputeState(float time);
}

#endif