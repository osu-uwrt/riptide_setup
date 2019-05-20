#ifndef SIMPLE_MOTION_PLANNER
#define SIMPLE_MOTION_PLANNER

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

// This class performs motion planning along a single axis with which is constrains velocity to
// a trapezoidal profile
class SimpleMotionPlanner
{
private:
    float start, destination, distance, cruiseSpeed, acceleration;
    float cruiseDuration, vel0, accel1, accel2;
    int accelSeq;
    float t1, t2, tMid, tEnd; // Key times
    bool accelerate;
public:
    const int SEQ_START = 0;
    const int SEQ_END = 1;
    const int SEQ_BOTH = 2;
    const float DEFAULT_SPEED = 1.0;
    
    SimpleMotionPlanner(float startPos, float nominalSpeed);
    void SetAcceleration(float accel, int seq);
    void SetDestination(float dest);
    void InitMotionPlanner();
    float GetTravelTime();
    Vector2f ComputeState(float t);
}

#endif