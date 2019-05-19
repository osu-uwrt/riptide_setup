#include "riptide_gnc/trapezoidal_motion.h"

TrapezoidalMotion::TrapezoidalMotion(float startPos, float travelSpeed)
{
    start = startPos;
    cruiseSpeed = abs(travelSpeed);
    accelerate = false;

    destination = 0;
    accel = 0;
    accelSeq = SEQ_BOTH;
    t1 = 0, t2 = 0, tMid = 0, tFinal = 0;
}

TrapezoidalMotion::TrapezoidalMotion(float startPos, float travelSpeed, bool accelOption)
{
    start = startPos;
    cruiseSpeed = abs(travelSpeed);
    accelerate = accelOption;
    
    destination = 0;
    accel = 0;
    accelSeq = SEQ_BOTH;
    t1 = 0, t2 = 0, tMid = 0, tFinal = 0;
}

void TrapezoidalMotion::SetAccel(float acceleration, int seq)
{
    if (accelerate)
    {
        if (acceleration != 0)
            accel = abs(acceleration);
        else
            accelerate = false; // If input accel is zero, then do not accelerate

        if (seq >= SEQ_START && seq <= SEQ_BOTH) // Check accel seq is valid
            accelSeq = seq;
        else
            accelSeq = SEQ_BOTH; // Invalid, default to SEQ_BOTH
    }
}

void TrapezoidalMotion::SetDestination(float dest)
{
    destination = dest;
    TrapezoidalMotion::ComputeTravelInfo();
}

void TrapezoidalMotion::ComputeTravelInfo()
{
    float distance = abs(destination - start);

    if (!accelerate)
    {
        tFinal = distance / cruiseSpeed;
    }
    else
    {
        float tAccel = cruiseSpeed / accel;
        float distAccel = pow(cruiseSpeed, 2) / accel;

        if (accelSeq == SEQ_START)
        {

        }
        else if (accelSeq == SEQ_END)
        {

        }
        else
        {
            
        }
    }
}

float TrapezoidalMotion::GetTotalTime()
{
}

Vector2f TrapezoidalMotion::ComputeState(float time)
{
}