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

  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  pwm_pub = nh.advertise<riptide_msgs::PwmStamped>("/command/pwm", 1);
  linear_accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
  attitude_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/attitude", 1);
  alignment_pub = nh.advertise<riptide_msgs::AlignmentCommand>("/command/alignment", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
  task_info_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);
  state_mission_pub = nh.advertise<riptide_msgs::MissionState>("/state/mission", 1);

  BeAutonomous::LoadParam<int>("competition_id", competition_id);
  BeAutonomous::LoadParam<double>("loader_timer", loader_timer);

  // Load Task Execution Parameters
  nh.param("Task_Execution/task_order", task_order, vector<int>(0));
  BeAutonomous::LoadParam<bool>("Task_Execution/single_test", single_test);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_x", relative_current_x);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_y", relative_current_y);

  // Load Task Runtime Parameters
  BeAutonomous::LoadParam<double>("Controller_Thresholds/depth_thresh", depth_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/roll_thresh", roll_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/pitch_thresh", pitch_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/yaw_thresh", yaw_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/error_duration_thresh", error_duration_thresh);

  mission_loaded = false;
  vehicle_ready = false;
  thruster = 0;

  // Load Task Info
  task_file = rc::FILE_TASKS;
  task_id = -1;
  last_task_id = -1;
  color = rc::COLOR_BLACK;
  if(competition_id == rc::COMPETITION_SEMIS)
    task_map_file = rc::FILE_MAP_SEMIS;
  else
    task_map_file = rc::FILE_MAP_FINALS;

  tasks = YAML::LoadFile(task_file);
  task_map = YAML::LoadFile(task_map_file);
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

  BeAutonomous::UpdateTaskInfo();
  BeAutonomous::ReadMap();

  // Enable all controllers (won't be activated yet until commanded to do so)
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

  // Vehicle State
  euler_rpy.x = 0;
  euler_rpy.y = 0;
  euler_rpy.z = 0;
  linear_accel.x = 0;
  linear_accel.y = 0;
  linear_accel.z = 0;
  depth = 0;

  // Initialize class objects and pointers
  // The "new" keyword creates a pointer to the object
  tslam = new TSlam(this);
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

void BeAutonomous::StartTaskTimer(const ros::TimerEvent& event) {
  BeAutonomous::StartTask();
}

void BeAutonomous::StartTask() {
  task_order_index++;
  if(task_order_index < task_order.size()) {
    task_id = task_order.at(task_order_index);
    BeAutonomous::UpdateTaskInfo();
    BeAutonomous::ReadMap();
    tslam->Start();
  }
  else {
    BeAutonomous::EndMission();
  }
}

void BeAutonomous::EndMission() {
  task_order_index = -1;
  task_id = -1;
  last_task_id = -1;
  tslam->Abort();
  if(roulette->active)
    roulette->Abort();
}

void BeAutonomous::SystemCheckTimer(const ros::TimerEvent& event) {
  riptide_msgs::PwmStamped pwm_msg;
  pwm_msg.header.stamp = ros::Time::now();
  pwm_msg.pwm.surge_port_lo = 1500;
  pwm_msg.pwm.surge_stbd_lo = 1500;
  pwm_msg.pwm.sway_fwd = 1500;
  pwm_msg.pwm.sway_aft = 1500;
  pwm_msg.pwm.heave_port_fwd = 1500;
  pwm_msg.pwm.heave_stbd_fwd = 1500;
  pwm_msg.pwm.heave_port_aft = 1500;
  pwm_msg.pwm.heave_stbd_aft = 1500;

  if(thruster < 8) { // Set thrusters to 1550 one at a time
    if(thruster == 0)
      pwm_msg.pwm.surge_port_lo = 1550;
    else if(thruster == 1)
      pwm_msg.pwm.surge_stbd_lo = 1550;
    else if(thruster == 2)
      pwm_msg.pwm.sway_fwd = 1550;
    else if(thruster == 3)
      pwm_msg.pwm.sway_aft = 1550;
    else if(thruster == 4)
      pwm_msg.pwm.heave_port_fwd = 1550;
    else if(thruster == 5)
      pwm_msg.pwm.heave_stbd_fwd = 1550;
    else if(thruster == 6)
      pwm_msg.pwm.heave_port_aft = 1550;
    else if(thruster == 7)
      pwm_msg.pwm.heave_stbd_aft = 1550;

    pwm_pub.publish(pwm_msg);
    thruster++;
    timer = nh.createTimer(ros::Duration(0.5), &BeAutonomous::SystemCheckTimer, this);
  }
  else pwm_pub.publish(pwm_msg);
}

void BeAutonomous::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  search_depth = tasks["tasks"][task_id]["search_depth"].as<double>();
  search_accel = tasks["tasks"][task_id]["search_accel"].as<double>();
  align_thresh = tasks["tasks"][task_id]["align_thresh"].as<double>();
  bbox_thresh = tasks["tasks"][task_id]["bbox_thresh"].as<double>();
  detection_duration_thresh = tasks["tasks"][task_id]["detection_duration_thresh"].as<double>();
  detections_req = tasks["tasks"][task_id]["detections_req"].as<double>();

  riptide_msgs::TaskInfo task_msg;
  task_msg.task_id = task_id;
  task_msg.task_name = task_name;
  task_msg.alignment_plane = alignment_plane;
  task_info_pub.publish(task_msg);
}

void BeAutonomous::ReadMap() {
  quadrant = floor(load_id/2.0);
  if(quadrant < 4) {
    start_x = task_map["task_map"][quadrant]["map"][task_id]["start_x"].as<double>();
    start_y = task_map["task_map"][quadrant]["map"][task_id]["start_y"].as<double>();

    if(last_task_id == -1 && !single_test) {
      current_x = task_map["task_map"][quadrant]["dock_x"].as<double>();
      current_x = task_map["task_map"][quadrant]["dock_y"].as<double>();
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
  BeAutonomous::EndTSlam();
}

void BeAutonomous::EndTSlam() {
  tslam->Abort();
  eta = 0;
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
    vehicle_ready = false;
    BeAutonomous::EndMission();
  }
  else if(!mission_loaded && switch_msg->kill == 0 && activation_sum > 0) { // Ready to load new mission
    if(quad_sum > 1) { // No kill switch and one quadrant selected
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
      load_time = ros::Time::now();
    }
    else {
      load_duration = ros::Time::now().toSec() - load_time.toSec();
      if(load_duration > loader_timer) {
        mission_loaded = true;
      }
    }
  }
  else if(switch_msg->kill && mission_loaded && !vehicle_ready) {
    vehicle_ready = true;
    if(load_id == rc::MISSION_TEST) {
      // Use a delay for the first time because of initializing thrusters
      timer = nh.createTimer(ros::Duration(3), &BeAutonomous::SystemCheckTimer, this);
    }
    else if(load_id < rc::MISSION_TEST) {
      timer = nh.createTimer(ros::Duration(5), &BeAutonomous::StartTaskTimer, this);
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
