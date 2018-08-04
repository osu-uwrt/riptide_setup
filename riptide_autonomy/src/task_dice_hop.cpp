#include "riptide_autonomy/task_dice_hop.h"

DiceHop::DiceHop(BeAutonomous *master)
{
  this->master = master;
  duration = 0;
}

void DiceHop::Start()
{
  // MUST add 1 second to prevent issues due to function call timing
  duration = master->tslam->tslam_duration + 1;
  timer = master->nh.createTimer(ros::Duration(duration), &DiceHop::DiceHopTimer, this, true);
  ROS_INFO("DiceHop: Timer initiated until Tslam ends in %f seconds", duration);
}

void DiceHop::DiceHopTimer(const ros::TimerEvent &event)
{
    timer.stop();
    master->LaunchTSlam();
    ROS_INFO("DiceHop: Launching TSlam for next task");
}

// Shutdown all active subscribers
void DiceHop::Abort()
{
  ROS_INFO("DiceHop: Aborting");
  duration = 0;
}
