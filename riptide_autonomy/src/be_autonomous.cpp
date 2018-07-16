#include "riptide_autonomy/be_autonomous.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "be_autonomous");
  BeAutonomous ba;
  ba.Loop();
}

BeAutonomous::BeAutonomous() : nh("be_autonomous") { // NOTE: there is no namespace declared in nh()
  switch_sub = nh.subscribe<riptide_msgs::SwitchState>("/state/switches", 1, &BeAutonomous::SwitchCB, this);
	task_info_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);
  state_mission_pub = nh.advertise<riptide_msgs::MissionState>("/state/mission", 1);

  execute_id = rc::EXECUTE_STANDBY;
  mission_loaded = false;
  BeAutonomous::LoadParam<double>("loader_watchdog", loader_watchdog);

  // Load Task Info
  BeAutonomous::LoadParam<string>("task_file", task_file);
  task_id = rc::TASK_ROULETTE;
  tasks = YAML::LoadFile(task_file);

  // Verify number of objects and thresholds match
  num_tasks = (int)tasks["tasks"].size();
  for(int i=0; i<num_tasks; i++) {
    int num_objects = (int)tasks["tasks"][i]["objects"].size();
    int num_thresholds = (int)tasks["tasks"][i]["thresholds"].size();
    task_name = tasks["tasks"][i]["name"].as<string>();
    if(num_objects != num_thresholds) {
      ROS_INFO("Task ID %i (%s): %i objects and %i thresholds. Quantities must match", i, task_name.c_str(), num_objects, num_thresholds);
      ROS_INFO("Shutting Down");
      ros::shutdown();
    }
  }
  BeAutonomous::UpdateTaskInfo();
  //TSlam ts(&nh);
  //tslam_ptr = &ts;
}

// Load parameter from namespace
template <typename T>
void BeAutonomous::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Be Autonomous Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void BeAutonomous::Execute() {
  if(execute_id == rc::EXECUTE_STANDBY) {
    if(mission_loaded) {
      execute_id = rc::EXECUTE_TSLAM;
    }
  }
  else if(execute_id == rc::EXECUTE_TSLAM) {

  }
  else if(execute_id == rc::EXECUTE_MISSION) {

  }
}

void BeAutonomous::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)
}

void BeAutonomous::SystemCheck() {

}

void BeAutonomous::SwitchCB(const riptide_msgs::SwitchState::ConstPtr& switch_msg) {
  /* 5 activation switches
      sw1 - quad A
      sw2 - quad B
      sw3 - quad C
      sw4 - quad D
      sw5 - test / casino color (In -> Black, Out -> Red)
  */
  int quad_sum = switch_msg->sw1 + switch_msg->sw2 + switch_msg->sw3 + switch_msg->sw4;
  int activation_sum = quad_sum + switch_msg->sw5;

  if(switch_msg->kill == 0 && (quad_sum > 1 || activation_sum == 0)) {
    mission_loaded = false; // Cancel if more than two quads activated, or if no switch is in
  }
  else if(!mission_loaded && switch_msg->kill == 0 && activation_sum > 0) { // Ready to load new mission
    if(quad_sum > 1) {
      // No kill switch and one quadrant selected, ready to prime
      if(switch_msg->sw1 && !switch_msg->sw5){
        load_id = rc::MISSION_A_BLACK; //Quad A Black
      }
      else if(switch_msg->sw1 && switch_msg->sw5){
        load_id = rc::MISSION_A_RED; //Quad A Red
      }
      else if(switch_msg->sw2 && !switch_msg->sw5){
        load_id = rc::MISSION_B_BLACK; //Quad B Black
      }
      else if(switch_msg->sw2 && switch_msg->sw5){
        load_id = rc::MISSION_B_RED; //Quad B Red
      }
      else if(switch_msg->sw3 && !switch_msg->sw5){
        load_id = rc::MISSION_C_BLACK; //Quad C Black
      }
      else if(switch_msg->sw3 && switch_msg->sw5){
        load_id = rc::MISSION_C_RED; //Quad C Red
      }
      else if(switch_msg->sw4 && !switch_msg->sw5){
        load_id = rc::MISSION_D_BLACK; //Quad D Black
      }
      else if(switch_msg->sw4 && switch_msg->sw5){
        load_id = rc::MISSION_D_RED; //Quad D Red
      }
    }
    else if(quad_sum == 0 && switch_msg->sw5) {
      load_id = rc::MISSION_TEST; // Do system check
    }

    if(load_id != last_load_id) {
      load_timer = ros::Time::now().toSec();
    }
    else {
      load_timer = ros::Time::now().toSec() - load_timer;
      if(load_timer > loader_watchdog) {
        mission_loaded = true;
      }
    }
  }
}

void BeAutonomous::Loop()
{
  ros::Rate rate(200);
  while(ros::ok())
  {
    ros::spinOnce();
    BeAutonomous::Execute();
    rate.sleep();
  }
}
