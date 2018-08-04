#include "riptide_autonomy/task_cash_in.h"

CashIn::CashIn(BeAutonomous *master)
{
  this->master = master;
  duration = 0;
}

void CashIn::Start()
{
  // MUST add 1 second to prevent issues due to function call timing
  duration = master->tslam->tslam_duration + 1;
  timer = master->nh.createTimer(ros::Duration(duration), &CashIn::CashInTimer, this, true);
  ROS_INFO("CashIn: Timer initiated until Tslam ends in %f seconds", duration);
  ROS_INFO("CashIn: Current x accel: %f", master->linear_accel.x);
}

void CashIn::CashInTimer(const ros::TimerEvent &event)
{
    timer.stop();
    master->EndMission();
    ROS_INFO("CashIn: Ending mission. Will hopefully surface within square.");
}

// Shutdown all active subscribers
void CashIn::Abort()
{
  ROS_INFO("CashIn: Aborting");
  duration = 0;
}
