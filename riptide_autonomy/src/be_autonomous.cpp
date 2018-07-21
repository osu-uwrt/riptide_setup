#include "riptide_autonomy/be_autonomous.h"

// Slope and y-int for x-accel to x-velocity
#define A2V_SLOPE 0.2546
#define A2V_INT .1090

int main(int argc, char** argv) {
  ros::init(argc, argv, "be_autonomous");
  BeAutonomous ba;
  ros::spin();
}

BeAutonomous::BeAutonomous() : nh("be_autonomous") { // NOTE: there is no namespace declared in nh()
  switch_sub = nh.subscribe<riptide_msgs::SwitchState>("/state/switches", 1, &BeAutonomous::SwitchCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &BeAutonomous::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &BeAutonomous::DepthCB, this);

  linear_accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
  attitude_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/attitude", 1);
  alignment_pub = nh.advertise<riptide_msgs::AlignmentCommand>("/command/alignment", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
  pneumatics_pub = nh.advertise<riptide_msgs::Pneumatics>("/command/pneumatics", 1);

  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  thrust_pub = nh.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);
  task_info_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);
  state_mission_pub = nh.advertise<riptide_msgs::MissionState>("/state/mission", 1);

  BeAutonomous::LoadParam<int>("competition_id", competition_id);
  BeAutonomous::LoadParam<double>("loader_timer", loader_timer);
  BeAutonomous::LoadParam<double>("start_timer", start_timer);

  // Load Task Execution Parameters
  nh.param("Task_Execution/task_order", task_order, vector<int>(0));
  BeAutonomous::LoadParam<bool>("Task_Execution/run_single_task", run_single_task);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_x", relative_current_x);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_y", relative_current_y);

  // Load Task Runtime Parameters
  BeAutonomous::LoadParam<double>("Controller_Thresholds/depth_thresh", depth_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/roll_thresh", roll_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/pitch_thresh", pitch_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/yaw_thresh", yaw_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/error_duration_thresh", error_duration_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/bbox_duration_thresh", bbox_duration_thresh);

  mission_loaded = false;
  mission_running = false;
  thruster = 0;
  pre_start_duration = 0;
  clock_is_ticking = false;

  start_x = 0;
  start_y = 0;
  current_x = 0;
  current_y = 0;

  // Load Task Info
  task_file = rc::FILE_TASKS;
  task_id = -1;
  last_task_id = -1;
  color = rc::COLOR_BLACK;
  quadrant = rc::QUAD_A;
  ROS_INFO("comp id: %i", competition_id);
  if(competition_id == rc::COMPETITION_SEMIS)
    task_map_file = rc::FILE_MAP_SEMIS;
  else
    task_map_file = rc::FILE_MAP_FINALS;

  ROS_INFO("Loading tasks: %s", task_file.c_str());
  tasks = YAML::LoadFile(task_file);
  ROS_INFO("Loading task map: %s", task_map_file.c_str());
  task_map = YAML::LoadFile(task_map_file);
  ROS_INFO("YAML loaded");
  task_order_index = -1;
  tasks_enqueued = task_order.size();
  search_depth = 0;
  search_accel = 0;

  // Verify number of objects and thresholds match
  total_tasks = (int)tasks["tasks"].size();
  for(int i=0; i<total_tasks; i++) {
    int num_objects = (int)tasks["tasks"][i]["objects"].size();
    int num_thresholds = (int)tasks["tasks"][i]["thresholds"].size();
    task_name = tasks["tasks"][i]["name"].as<string>();
    if(num_objects != num_thresholds) {
      ROS_INFO("Task ID %i (%s): %i objects and %i thresholds. Quantities must match", i, task_name.c_str(), num_objects, num_thresholds);
      ROS_INFO("Shutting Down");
      ros::shutdown();
    }
  }
  ROS_INFO("Tasks verified");


  // Vehicle State
  euler_rpy.x = 0;
  euler_rpy.y = 0;
  euler_rpy.z = 0;
  linear_accel.x = 0;
  linear_accel.y = 0;
  linear_accel.z = 0;
  depth = 0;
  frame_width = 644;
  frame_height = 482;

  // Initialize class objects and pointers
  // The "new" keyword creates a pointer to the object
  tslam = new TSlam(this);
  roulette = new Roulette(this);
  ROS_INFO("Created task objects");
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

void BeAutonomous::StartTask() {
  mission_running = true;
  task_order_index++;
  if(task_order_index < task_order.size()) {
    task_id = task_order.at(task_order_index);
    ROS_INFO("New task ID: %i", task_id);
    BeAutonomous::UpdateTaskInfo();
    BeAutonomous::ReadMap();
    tslam->Start();
    if(task_id == rc::TASK_ROULETTE) {
      ROS_INFO("Starting roulette task");
      roulette->Start();
      ROS_INFO("Roulette task started");
    }
  }
  else {
    BeAutonomous::EndMission();
  }
}

void BeAutonomous::EndMission() {
  mission_running = false;

  if(tslam->enroute)
    tslam->Abort();
  if(task_id == rc::TASK_ROULETTE)
    roulette->Abort();

  task_order_index = -1;
  task_id = -1;
  last_task_id = -1;

  BeAutonomous::SendResetMsgs();
}

void BeAutonomous::SendInitMsgs() {
  reset_msg.reset_surge = false;
  reset_msg.reset_sway = false;
  reset_msg.reset_heave = false;
  reset_msg.reset_roll = false;
  reset_msg.reset_pitch = false;
  reset_msg.reset_yaw = false;
  reset_msg.reset_depth = false;
  reset_msg.reset_pwm = false;
  reset_pub.publish(reset_msg);
}

void BeAutonomous::SendResetMsgs() {
  attitude_cmd.roll_active = false;
  attitude_cmd.pitch_active = false;
  attitude_cmd.yaw_active = false;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = 0;
  attitude_pub.publish(attitude_cmd);

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = "Casino_Gate_Black";
  align_cmd.alignment_plane = rc::PLANE_YZ;
  align_cmd.bbox_dim = frame_width/2;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  alignment_pub.publish(align_cmd);

  reset_msg.reset_surge = true;
  reset_msg.reset_sway = true;
  reset_msg.reset_heave = true;
  reset_msg.reset_roll = true;
  reset_msg.reset_pitch = true;
  reset_msg.reset_yaw = true;
  reset_msg.reset_depth = true;
  reset_msg.reset_pwm = true;
  reset_pub.publish(reset_msg);
}

void BeAutonomous::SystemCheckTimer(const ros::TimerEvent& event) {


  ROS_INFO("SystemCheckTimer, thruster number: %i", thruster);
  thrust_msg.header.stamp = ros::Time::now();
  thrust_msg.force.surge_port_lo = 0;
  thrust_msg.force.surge_stbd_lo = 0;
  thrust_msg.force.sway_fwd = 0;
  thrust_msg.force.sway_aft = 0;
  thrust_msg.force.heave_port_fwd = 0;
  thrust_msg.force.heave_stbd_fwd = 0;
  thrust_msg.force.heave_port_aft = 0;
  thrust_msg.force.heave_stbd_aft = 0;

  if(thruster < 8) { // Set thrusters to 1550 one at a time

    if(thruster == 0)
      thrust_msg.force.heave_port_fwd = 1;
    else if(thruster == 1)
      thrust_msg.force.heave_stbd_fwd = 1;
    else if(thruster == 2)
      thrust_msg.force.heave_port_aft = 1;
    else if(thruster == 3)
      thrust_msg.force.heave_stbd_aft = 1;
    else if(thruster == 4)
      thrust_msg.force.surge_port_lo = 1;
    else if(thruster == 5)
      thrust_msg.force.surge_stbd_lo = 1;
    else if(thruster == 6)
      thrust_msg.force.sway_fwd = 1;
    else if(thruster == 7)
      thrust_msg.force.sway_aft = 1;


    thrust_pub.publish(thrust_msg);
    thruster++;
    timer = nh.createTimer(ros::Duration(2), &BeAutonomous::SystemCheckTimer, this, true);
  }
  else {
    SendInitMsgs();
    thrust_pub.publish(thrust_msg); // Publish all 1500s
    thruster = 0;
  }
}

void BeAutonomous::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();
  num_objects = (int)tasks["tasks"][task_id]["objects"].size();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  object_names.clear();
  for(int i=0; i < num_objects; i++) {
    object_names.push_back(tasks["tasks"][task_id]["objects"][i].as<string>());
  }

  search_depth = tasks["tasks"][task_id]["search_depth"].as<double>();
  search_accel = tasks["tasks"][task_id]["search_accel"].as<double>();
  align_thresh = tasks["tasks"][task_id]["align_thresh"].as<int>();
  bbox_thresh = tasks["tasks"][task_id]["bbox_thresh"].as<int>();
  detection_duration_thresh = tasks["tasks"][task_id]["detection_duration_thresh"].as<double>();
  detections_req = tasks["tasks"][task_id]["detections_req"].as<int>();

  riptide_msgs::TaskInfo task_msg;
  task_msg.task_id = task_id;
  task_msg.task_name = task_name;
  task_msg.alignment_plane = alignment_plane;
  task_info_pub.publish(task_msg);
}

void BeAutonomous::ReadMap() {
  quadrant = floor(load_id / 2.0);
  ROS_INFO("Quadrant: %i", quadrant);
  if(quadrant < 4) {
    start_x = task_map["task_map"][quadrant]["map"][task_id]["start_x"].as<double>();
    start_y = task_map["task_map"][quadrant]["map"][task_id]["start_y"].as<double>();

    if(last_task_id == -1 && !single_test) {
      current_x = task_map["task_map"][quadrant]["dock_x"].as<double>();
      current_y = task_map["task_map"][quadrant]["dock_y"].as<double>();
    }
    else if(single_test && task_order.size() == 1) {
      current_x = start_x + relative_current_x;
      current_y = start_y + relative_current_y;
    }
    else {
      current_x = task_map["task_map"][quadrant]["map"][last_task_id]["end_x"].as<double>();
      current_y = task_map["task_map"][quadrant]["map"][last_task_id]["end_y"].as<double>();
    }
  }
  else {
    current_x = 420;
    current_y = 420;
    start_x = 420;
    start_y = 420;
  }

  ROS_INFO("Start X: %f", start_x);
  ROS_INFO("Start Y: %f", start_y);
}

// Calculate eta for TSlam to bring vehicle to next task
void BeAutonomous::CalcETA(double Ax, double dist) {
  if(Ax >= 0.6 && Ax <= 1.25) { // Eqn only valid for Ax=[0.6, 1.25] m/s^2
    double vel = A2V_SLOPE*Ax + A2V_INT;
    eta = dist/vel;
  }
  else eta = 0;
}

void BeAutonomous::EndTSlamTimer(const ros::TimerEvent& event) {
  if(tslam->enroute)
    BeAutonomous::EndTSlam();
}

void BeAutonomous::EndTSlam() {
  tslam->Abort();
  eta = 0;
}

void BeAutonomous::ResetSwitchPanel() {
  pre_start_duration = 0;
  clock_is_ticking = false;
  load_duration = 0;
  last_load_id = -1;
  mission_loaded = false;
  color = 0;
  thruster = 0;
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
    BeAutonomous::ResetSwitchPanel();
    BeAutonomous::EndMission();
    ROS_INFO("Mission ended. Waiting for new load ID");
  }
  else if(!switch_msg->kill && last_kill_switch_value) { // Put kill witch in then out
    BeAutonomous::ResetSwitchPanel();
    BeAutonomous::EndMission();
    ROS_INFO("Mission ended. Waiting for new load ID");
  }
  else if(!mission_loaded && switch_msg->kill == 0 && activation_sum > 0) { // Ready to load new mission
    if(quad_sum == 1) { // No kill switch and one quadrant selected
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
    else if(quad_sum == 0 && switch_msg->sw5) { // Only test switch is engaged
      load_id = rc::MISSION_TEST; // Do system check
    }

    if(load_id != last_load_id) {
      last_load_id = load_id;
      load_time = ros::Time::now();
      ROS_INFO("Load ID != last load ID");
    }
    else {
      load_duration = ros::Time::now().toSec() - load_time.toSec();
      ROS_INFO("Load duration: %f", load_duration);
      if(load_duration > loader_timer) {
        mission_loaded = true;
        ROS_INFO("Mission Loaded: %i", load_id);
        if(load_id < rc::MISSION_TEST) {
          color == load_id % 2;
        }
      }
    }
  }
  else if(switch_msg->kill && mission_loaded) {
    if(load_id == rc::MISSION_TEST) {
      BeAutonomous::ResetSwitchPanel();
      // Use a delay for the first time because of initializing thrusters
      ROS_INFO("Back off bro. Thrusters be turnin' shortly.");
      reset_msg.reset_surge = true;
      reset_msg.reset_sway = true;
      reset_msg.reset_heave = true;
      reset_msg.reset_roll = true;
      reset_msg.reset_pitch = true;
      reset_msg.reset_yaw = true;
      reset_msg.reset_depth = true;
      reset_msg.reset_pwm = false;
      reset_pub.publish(reset_msg);
      timer = nh.createTimer(ros::Duration(5), &BeAutonomous::SystemCheckTimer, this, true);
    }
    else if(load_id < rc::MISSION_TEST) {
      ROS_INFO("Starting the mission. Maelstrom goin' under in %f sec.", start_timer-pre_start_duration);
      if(!clock_is_ticking) {
        pre_start_time = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        pre_start_duration = ros::Time::now().toSec() - pre_start_time.toSec();

      if(pre_start_duration > start_timer) {
        ROS_INFO("About to call Starttask()");
        // Set these back to '0' or 'true' so it doesn't run again
        BeAutonomous::ResetSwitchPanel();
        BeAutonomous::SendInitMsgs();
        BeAutonomous::StartTask();
      }
    }
  }
  last_kill_switch_value = switch_msg->kill;
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
