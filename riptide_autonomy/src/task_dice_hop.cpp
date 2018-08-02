#include "riptide_autonomy/task_dice_hop.h"

DiceHop::DiceHop(BeAutonomous *master)
{
  this->master = master;
}

void DiceHop::Start()
{
  timer = master->nh.createTimer(ros::Duration(master->tslam->tslam_duration), &DiceHop::DiceHopTimer, this, true);
  ROS_INFO("DiceHop: Timer initiated until Tslam ends");
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
}
