#include "riptide_gnc/simple_motion_planner.h"

/**
 * @param startPos Starting position
 * @param nominalSpeed Desired cruise speed
 */
SimpleMotionPlanner::SimpleMotionPlanner(float startPos, float nominalSpeed)
{
    start = startPos;

    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed = DEFAULT_SPEED;
    else
        cruiseSpeed = abs(nominalSpeed);

    accelerate = false; // Default to constant speed (accel = 0)
    destination = 0;
    acceleration = 0;
    distance = 0;

    accelSeq = SEQ_BOTH; // Default to both
    t1 = 0, t2 = 0, tMid = 0, tFinal = 0;
    cruiseDuration = 0;
    vel0 = 0, accel1 = 0, accel2 = 0;
}

/**
 * @param acceleration Desired acceleration
 * @param seq Acceleration sequence (SEQ_START, SEQ_END, or SEQ_BOTH)
 */
void SimpleMotionPlanner::SetAcceleration(float accel, int seq)
{
    if (accel != 0)
    {
        accelerate = true;
        acceleration = abs(accel);

        // Check accel seq is valid
        // If not, then it remains as SEQ_BOTH
        if (seq >= SEQ_START && seq <= SEQ_BOTH)
            accelSeq = seq;
    }
    else
        accelerate = false; // Keep at false
}

/**
 * @param dest Destination
 */
void SimpleMotionPlanner::SetDestination(float dest)
{
    destination = dest;
    SimpleMotionPlanner::InitMotionPlanner();
}

// Initialize Motion Planner - calculate travel parameters
void SimpleMotionPlanner::InitMotionPlanner()
{
    distance = abs(destination - start);

    // Exit if distance is 0
    if (distance == 0)
        return;

    if (!accelerate) // Constant speed
    {
        tEnd = distance / cruiseSpeed;
        cruiseDuration = tEnd;
        t2 = tEnd;
    }
    else // Will be accelerating for certain portions of travel
    {
        float accelDuration = cruiseSpeed / acceleration;           // Assuming accelerating from rest
        float accelDist = 0.5 * pow(cruiseSpeed, 2) / acceleration; // Or 0.5*acceleration*t^2

        if (accelSeq == SEQ_START)
        {
            if (accelDist <= distance) // Possible Scenario: Will be traveling at cruiseSpeed at destination
            {
                cruiseDuration = (distance - accelDist) / cruiseSpeed;
                t1 = accelDuration;
                tEnd = t1 + cruiseDuration;
                t2 = tEnd;
                accel1 = acceleration;
            }
            else // Impossible Scenario: Will be traveling slower than cruiseSpeed at destination
            {
                stringstream ss;
                ss << "SimpleMotionPlanner: Accelerating too slowly in SEQ_START! Will be traveling faster than cruiseSpeed at destination. Adjust parameters." << endl;
                throw std::runtime_error(ss.str());
            }
        }
        else if (accelSeq == SEQ_END)
        {
            if (accelDist < distance) // Possible Scenario: Will be able to accelerate from cruiseSpeed to rest
            {
                cruiseDuration = (distance - accelDist) / cruiseSpeed;
                t1 = 0;
                t2 = cruiseDuration;
                tEnd = t2 + accelDuration;
                vel0 = cruiseSpeed;
                accel2 = -acceleration;
            }
            else // Impossible Scenario: Will have non-zero speed when you reach the destination
            {
                stringstream ss;
                ss << "SimpleMotionPlanner: Accelerating too slowly in SEQ_END! Will have non-zero speed at destination. Adjust parameters." << endl;
                throw std::runtime_error(ss.str());
            }
        }
        else if (accelSeq == SEQ_BOTH)
        {
            if (2 * accelDist <= distance) // Will reach cruise speed for some time >= 0
            {
                cruiseDuration = (distance - 2 * accelDist) / cruiseSpeed;
                t1 = accelDuration;
                t2 = accelDuration + cruiseDuration;
                tEnd = t2 + accelDuration;
                accel1 = acceleration;
                acccel2 = -acceleration;
            }
            else // Will not reach cruiseSpeed during travel
            {
                tMid = sqrt(distance / acceleration);
                t1 = tMid;
                t2 = tMid;
                tEnd = tMid * 2;
            }
        }
    }
}

float SimpleMotionPlanner::GetTravelTime()
{
    return tEnd;
}

/**
 * @param t Current time for state to be computed (state = [pos, vel])
 **/
Vector2f SimpleMotionPlanner::ComputeState(float t)
{
    Vector2f state;
    state.setZero();

    if (!accelerate) // Constant Speed
    {
        if (t >= 0 && t <= tEnd) // Valid time instance
        {
            state(0) = start + cruiseSpeed * t;
            state(1) = cruiseSpeed;
        }
        else if (t > tEnd) // Only know about THIS trajectory
        {
            state(0) = destination;
            state(1) = cruiseSpeed;
        }
        else if (t < 0)
        {
            state(0) = start;
            state(1) = cruiseSpeed;
        }
    }
    else if (accelerate)
    {
        if (t >= 0 && t <= tEnd) // Valid time instance
        {
            float time1 = 0, time2 = 0, time3 = 0;

            // t in [t, t1] range - accelarate from rest to cruiseSpeed
            time1 = (t <= t1) ? t : t1;
            state(0) = start + vel0 * time1 + 0.5 * accel1 * pow(time1, 2);
            state(1) = accel1 * time1;

            // t in (t1, t2] range - move at cruiseSpeed
            time2 = (t > t1 && t <= t2) ? t - t1 : 0;
            time2 = (t > t2) ? t2 - t1 : time2;
            state(0) = state(0) + cruiseSpeed * time2;

            // t in (t2, tEnd] range - accelerate from cruiseSPeed to rest
        }
    }
}