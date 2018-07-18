#include "riptide_autonomy/be_autonomous.h"

#define A2V_SLOPE 0.2546
#define A2V_INT .1090

int main(int argc, char** argv) {
  ros::init(argc, argv, "be_autonomous");
  BeAutonomous ba;
  ba.Loop();
}

BeAutonomous::BeAutonomous() : nh("be_autonomous") { // NOTE: there is no namespace declared in nh()
  switch_sub = nh.subscribe<riptide_msgs::SwitchState>("/state/switches", 1, &BeAutonomous::SwitchCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &BeAutonomous::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &BeAutonomous::DepthCB, this);

  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  linear_accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
  attitude_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/attitude", 1);
  alignment_pub = nh.advertise<riptide_msgs::AlignmentCommand>("/command/alignment", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
  task_info_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);
  state_mission_pub = nh.advertise<riptide_msgs::MissionState>("/state/mission", 1);

  execute_id = rc::EXECUTE_STANDBY;
  mission_loaded = false;
  BeAutonomous::LoadParam<int>("competition_id", competition_id);
  BeAutonomous::LoadParam<double>("loader_watchdog", loader_watchdog);

  // Activate all non-alignment controllers
  riptide_msgs::ResetControls reset_msg;
  reset_msg.reset_surge = false;
  reset_msg.reset_sway = false;
  reset_msg.reset_heave = false;
  reset_msg.reset_roll = false;
  reset_msg.reset_pitch = false;
  reset_msg.reset_yaw = false;
  reset_msg.reset_depth = false;
  reset_msg.reset_pwm = false;
  reset_pub.publish(reset_msg);

  // Load Task Info
  task_file = rc::FILE_TASKS;
  task_id = rc::TASK_ROULETTE;
  last_task_id = -1;
  search_depth = 1;
  //task_map_file = rc::FILE_MAP_SEMIS;
  if(competition_id == rc::COMPETITION_SEMIS)
    task_map_file = rc::FILE_MAP_SEMIS;
  else
    task_map_file = rc::FILE_MAP_FINALS;

  tasks = YAML::LoadFile(task_file);
  task_map = YAML::LoadFile(task_map_file);

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

  // Verify number of tasks in task.yaml and task_map.yaml agree
  BeAutonomous::UpdateTaskInfo();
  BeAutonomous::ReadMap();

  // Initialize class objects and pointers
  tslam = new TSlam(this); // "new" creates a pointer to the object
  time_elapsed = 0;

  roulette = new Roulette(this);
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
    if(mission_loaded && load_id == rc::MISSION_TEST) {
      execute_id = rc::EXECUTE_STANDBY;
    }
    else if(mission_loaded && load_id < rc::MISSION_TEST) {
      execute_id = rc::EXECUTE_TSLAM;
    }
  }
  else if(execute_id == rc::EXECUTE_TSLAM) {
    tslam->Execute();
    execute_id = rc::EXECUTE_MISSION;
  }
  else if(execute_id == rc::EXECUTE_MISSION) {
    cur_time = ros::Time::now();
    if(tslam->enroute && cur_time.toSec() > eta_start.toSec()) { // Keep track of eta set by tslam
      if((cur_time.toSec() - eta_start.toSec()) >= eta) {
        tslam->Abort();
      }
    }
    if(task_id == rc::TASK_ROULETTE && !roulette->active)
      roulette->Execute();
  }
}

void BeAutonomous::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  search_depth = tasks["tasks"][task_id]["search_depth"].as<double>();

  riptide_msgs::TaskInfo task_msg;
  task_msg.task_id = task_id;
  task_msg.task_name = task_name;
  task_msg.alignment_plane = alignment_plane;
  task_info_pub.publish(task_msg);
}

void BeAutonomous::ReadMap() {
  int quad = floor(load_id/2.0);
  if(quad < 4) {
    if(last_task_id == -1) {
      current_x = task_map["task_map"][quad]["dock_x"].as<double>();
      current_x = task_map["task_map"][quad]["dock_y"].as<double>();
    }
    else {
      current_x = task_map["task_map"][quad]["map"][last_task_id]["end_x"].as<double>();
      current_y = task_map["task_map"][quad]["map"][last_task_id]["end_y"].as<double>();
    }

    start_x = task_map["task_map"][quad]["map"][task_id]["start_x"].as<double>();
    start_y = task_map["task_map"][quad]["map"][task_id]["start_y"].as<double>();
  }
  else {
    current_x = 420;
    current_y = 420;
    start_x = 420;
    start_y = 420;
  }
}

void BeAutonomous::CalcETA(double Ax, double dist) {
  if(Ax >= 0.6 && Ax <= 1.25) { // Eqn only valid for Ax=[0.6, 1.25] m/s^2
    double vel = A2V_SLOPE*Ax + A2V_INT;
    eta = dist/vel;
  }
  else eta = -1;
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
    tslam->Abort();
    roulette->Abort();
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

void BeAutonomous::ImuCB(const riptide_msgs::Imu::ConstPtr & imu_msg) {
  euler_rpy = imu_msg->euler_rpy;
  linear_accel = imu_msg->linear_accel;
}

void BeAutonomous::DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg) {
  depth = depth_msg->depth;
}

void BeAutonomous::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  frame_width = cv_ptr->image.size().width;
  frame_height = cv_ptr->image.size().height;
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
