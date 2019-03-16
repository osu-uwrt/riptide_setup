#include "riptide_autonomy/be_autonomous.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "be_autonomous");
  BeAutonomous ba;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("BE ERROR: %s", e.what());
    ba.light_msg.green1 = false;
    ba.light_msg.red1 = true;
    ba.light_msg.green2 = false;
    ba.light_msg.red2 = true;
    ba.light_msg.green3 = false;
    ba.light_msg.red3 = true;
    ba.light_msg.green4 = false;
    ba.light_msg.red4 = true;
    ba.status_light_pub.publish(ba.light_msg);
    ROS_ERROR("BE: Shutting Down");
  }
}

BeAutonomous::BeAutonomous() : nh("be_autonomous")
{ // NOTE: there is no namespace declared in nh()
  switch_sub = nh.subscribe<riptide_msgs::SwitchState>("/state/switches", 1, &BeAutonomous::SwitchCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &BeAutonomous::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &BeAutonomous::DepthCB, this);
  mission_sub = nh.subscribe<std_msgs::Int8>("/remote/mission", 1, &BeAutonomous::StartMissionCB, this);

  x_accel_pub = nh.advertise<std_msgs::Float64>("/command/accel_x", 1);
  y_accel_pub = nh.advertise<std_msgs::Float64>("/command/accel_y", 1);
  attitude_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/attitude", 1);
  alignment_pub = nh.advertise<riptide_msgs::AlignmentCommand>("/command/alignment", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
  pneumatics_pub = nh.advertise<riptide_msgs::Pneumatics>("/command/pneumatics", 1);

  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  thrust_pub = nh.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);
  task_info_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);
  state_mission_pub = nh.advertise<riptide_msgs::MissionState>("/state/mission", 1);
  status_light_pub = nh.advertise<riptide_msgs::StatusLight>("/status/light", 1);

  BeAutonomous::LoadParam<int>("competition_id", competition_id);
  BeAutonomous::LoadParam<double>("loader_timer", loader_timer);
  BeAutonomous::LoadParam<double>("start_timer", start_timer);

  // Load Task Execution Parameters
  nh.param("Task_Execution/task_order", task_order, vector<int>(0));
  BeAutonomous::LoadParam<bool>("Task_Execution/run_single_task", run_single_task);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_x", relative_current_x);
  BeAutonomous::LoadParam<double>("Task_Execution/relative_current_y", relative_current_y);
  BeAutonomous::LoadParam<double>("Task_Execution/tslam_brake_duration", brake_duration);
  BeAutonomous::LoadParam<double>("Task_Execution/tslam_velocity", tslam_velocity);

  // Load Task Runtime Parameters
  BeAutonomous::LoadParam<double>("Controller_Thresholds/depth_thresh", depth_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/roll_thresh", roll_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/pitch_thresh", pitch_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/yaw_thresh", yaw_thresh);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/error_duration", error_duration);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/bbox_surge_duration", bbox_surge_duration);
  BeAutonomous::LoadParam<double>("Controller_Thresholds/bbox_heave_duration", bbox_heave_duration);

  ROS_INFO("BE: Competition id: %i", competition_id);

  mission_loaded = false;
  mission_running = false;
  thruster = 0;
  pre_start_duration = 0;
  load_duration = 0;
  clock_is_ticking = false;

  // Load Task Info
  task_file = rc::FILE_TASKS;
  task_id = -1;
  last_task_id = -1;
  quadrant = rc::QUAD_A;
  tasks = YAML::LoadFile(task_file);
  task_order_index = -1;
  search_depth = 0;
  search_accel = 0;
  bbox_thresh = 0;

  black_side = rc::LEFT;

  // Verify number of objects and thresholds match
  total_tasks = (int)tasks["tasks"].size();
  for (int i = 0; i < total_tasks; i++)
  {
    int num_objects = (int)tasks["tasks"][i]["objects"].size();
    int num_thresholds = (int)tasks["tasks"][i]["thresholds"].size();
    task_name = tasks["tasks"][i]["name"].as<string>();
    if (num_objects != num_thresholds)
    {
      ROS_INFO("Task ID %i (%s): %i objects and %i thresholds. Quantities must match", i, task_name.c_str(), num_objects, num_thresholds);
      ROS_INFO("Shutting Down");
      ros::shutdown();
    }
  }
  ROS_INFO("BE: Tasks yaml loaded and verified");

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
  cam_center_x = frame_width / 2;
  cam_center_y = frame_height / 2;

  // Initialize class objects and pointers
  // The "new" keyword creates a pointer to the object
  tslam = new TSlam(this);
  casino_gate = new CasinoGate(this);
  path = new PathMarker(this);
  dice_hop = new DiceHop(this);
  dice = new Dice(this);
  slots = new Slots(this);
  roulette = new Roulette(this);
  gold_chip = new GoldChip(this);
  cash_in = new CashIn(this);

  ROS_INFO("BE: Created task objects. Awaiting mission start.");
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
  catch (int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Be Autonomous Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

// TSlam gets launched first, and it will call StartTask when ready
void BeAutonomous::LaunchTSlam()
{
  mission_running = true;
  task_order_index++;
  if (task_order_index < task_order.size())
  {
    task_id = task_order.at(task_order_index);
    ROS_INFO("New task ID: %i", task_id);
    BeAutonomous::UpdateTaskInfo();
    tslam->Start();
  }
  else
  {
    ROS_INFO("BE: No more tasks to load. Collecting all rewards and will skidaddle");
    ROS_INFO("BE: Ending mission");
    BeAutonomous::EndMission();
  }
}

void BeAutonomous::StartTask()
{
  ROS_INFO("BE: Start task id: %i", task_id);
  switch (task_id)
  {
  case rc::TASK_CASINO_GATE:
    ROS_INFO("BE: Starting Casino Gate.");
    casino_gate->Start();
    break;
  case rc::TASK_PATH_MARKER1:
    ROS_INFO("BE: Starting Path Marker 1.");
    path->Start();
    break;
  case rc::TASK_DICE_HOP:
    ROS_INFO("BE: Starting dice hop task.");
    dice_hop->Start();
    break;
  case rc::TASK_DICE:
    ROS_INFO("BE: Starting dice task.");
    dice->Start();
    break;
  case rc::TASK_PATH_MARKER2:
    ROS_INFO("BE: Starting Path Marker 2.");
    path->Start();
    break;
  case rc::TASK_SLOTS:
    ROS_INFO("BE: Starting slots task");
    slots->Start();
    break;
  case rc::TASK_BUY_GOLD_CHIP1:
    ROS_INFO("BE: Starting Buy Gold Chip 1.");
    gold_chip->Start();
    break;
  case rc::TASK_ROULETTE:
    ROS_INFO("BE: Starting Roulette Task.");
    roulette->Start();
    break;
  case rc::TASK_BUY_GOLD_CHIP2:
    ROS_INFO("BE: Starting Buy Gold Chip 2.");
    gold_chip->Start();
    break;
  case rc::TASK_CASH_IN:
    ROS_INFO("BE: Starting CashIn");
    cash_in->Start();
    break;
  default:
    ROS_INFO("BE: Invalid Task ID. Ending mission.");
    BeAutonomous::EndMission();
    break;
  }
}

void BeAutonomous::EndMission()
{
  if (mission_running)
  {
    light_msg.green1 = false;
    light_msg.red1 = false;
    light_msg.green2 = false;
    light_msg.red2 = false;
    light_msg.green3 = false;
    light_msg.red3 = false;
    light_msg.green4 = false;
    light_msg.red4 = false;
    status_light_pub.publish(light_msg);

    ROS_INFO("ENDING MISSION!!!");

    tslam->EndMission();
    tslam->Abort(false); // Don't apply thruster brake
    casino_gate->Abort();
    path->Abort();
    dice->Abort();
    slots->Abort();
    roulette->Abort();

    task_order_index = -1;
    task_id = -1;
    last_task_id = -1;
    load_duration = 0;
  }

  BeAutonomous::SendResetMsgs();
  quadrant = 0;
  mission_running = false;
}

void BeAutonomous::SendInitMsgs()
{
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

void BeAutonomous::SendResetMsgs()
{
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
  align_cmd.bbox_dim = frame_width / 2;
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

void BeAutonomous::SystemCheckTimer(const ros::TimerEvent &event)
{
  string thruster_name;
  thrust_msg.header.stamp = ros::Time::now();
  thrust_msg.force.surge_port_lo = 0;
  thrust_msg.force.surge_stbd_lo = 0;
  thrust_msg.force.sway_fwd = 0;
  thrust_msg.force.sway_aft = 0;
  thrust_msg.force.heave_port_fwd = 0;
  thrust_msg.force.heave_stbd_fwd = 0;
  thrust_msg.force.heave_port_aft = 0;
  thrust_msg.force.heave_stbd_aft = 0;

  if (thruster < 8)
  { // Set thrusters to output 1 N of force

    if (thruster == 0)
    {
      thrust_msg.force.heave_port_fwd = 1;
      thruster_name = "HPF";
    }
    else if (thruster == 1)
    {
      thrust_msg.force.heave_stbd_fwd = 1;
      thruster_name = "HSF";
    }
    else if (thruster == 2)
    {
      thrust_msg.force.heave_port_aft = 1;
      thruster_name = "HPA";
    }
    else if (thruster == 3)
    {
      thrust_msg.force.heave_stbd_aft = 1;
      thruster_name = "HSA";
    }
    else if (thruster == 4)
    {
      thrust_msg.force.surge_port_lo = 1;
      thruster_name = "SPL";
    }
    else if (thruster == 5)
    {
      thrust_msg.force.surge_stbd_lo = 1;
      thruster_name = "SSL";
    }
    else if (thruster == 6)
    {
      thrust_msg.force.sway_fwd = 1;
      thruster_name = "SWFWD";
    }
    else if (thruster == 7)
    {
      thrust_msg.force.sway_aft = 1;
      thruster_name = "SWAFT";
    }

    ROS_INFO("SystemCheckTimer: thruster %s, depth %f m, yaw: %.5f", thruster_name.c_str(), depth, euler_rpy.z);
    thrust_pub.publish(thrust_msg);
    thruster++;
    timer = nh.createTimer(ros::Duration(2), &BeAutonomous::SystemCheckTimer, this, true);
  }
  else
  {
    ROS_INFO("SystemCheckTimer: thrusters tested, depth %f m, yaw: %.5f", depth, euler_rpy.z);
    thrust_pub.publish(thrust_msg); // Publish zero
    reset_msg.reset_surge = true;
    reset_msg.reset_sway = true;
    reset_msg.reset_heave = true;
    reset_msg.reset_roll = true;
    reset_msg.reset_pitch = true;
    reset_msg.reset_yaw = true;
    reset_msg.reset_depth = true;
    reset_msg.reset_pwm = true;
    reset_pub.publish(reset_msg);
    thruster = 0;
    //timer.stop();
  }
}

void BeAutonomous::UpdateTaskInfo()
{
  task_name = tasks["tasks"][task_id]["name"].as<string>();
  num_objects = (int)tasks["tasks"][task_id]["objects"].size();
  ROS_INFO("New Task Name: %s", task_name.c_str());

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if (alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  ROS_INFO("Alignment Plane: %i", alignment_plane);

  object_names.clear();
  for (int i = 0; i < num_objects; i++)
  {
    object_names.push_back(tasks["tasks"][task_id]["objects"][i].as<string>());
  }

  // Task specific parameters (in tasks.yaml)
  search_depth = tasks["tasks"][task_id]["search_depth"].as<double>();
  search_accel = tasks["tasks"][task_id]["search_accel"].as<double>();
  align_thresh = tasks["tasks"][task_id]["align_thresh"].as<int>();
  bbox_thresh = tasks["tasks"][task_id]["bbox_thresh"].as<int>();
  detection_duration = tasks["tasks"][task_id]["detection_duration"].as<double>();
  detections_req = tasks["tasks"][task_id]["detections_req"].as<int>();

  // Publish new task info
  riptide_msgs::TaskInfo task_msg;
  task_msg.task_id = task_id;
  task_msg.task_name = task_name;
  task_msg.alignment_plane = alignment_plane;
  task_info_pub.publish(task_msg);
}

void BeAutonomous::ResetSwitchPanel()
{
  pre_start_duration = 0;
  clock_is_ticking = false;
  load_duration = 0;
  last_load_id = -1;
  mission_loaded = false;
  thruster = 0;
  timer.stop();
}

void BeAutonomous::StartMissionCB(const std_msgs::Int8::ConstPtr &missionMsg)
{
  if (missionMsg->data >= 0)
  {
    if (!mission_running)
    {
      ROS_INFO("Remote start: %i", missionMsg->data);
      load_id = missionMsg->data;
      if (load_id < rc::MISSION_TEST)
        quadrant = load_id;
      BeAutonomous::ResetSwitchPanel();
      BeAutonomous::SendInitMsgs();
      BeAutonomous::LaunchTSlam();
    }
  }
  else
    BeAutonomous::EndMission();
}

void BeAutonomous::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &switch_msg)
{
  /* 5 activation switches
      sw1 - quad A
      sw2 - quad B
      sw3 - quad C
      sw4 - quad D
      sw5 - test / casino color (In -> Black, Out -> Red)
  */
  int quad_sum = switch_msg->sw1 + switch_msg->sw2 + switch_msg->sw3 + switch_msg->sw4;
  int activation_sum = quad_sum + switch_msg->sw5;

  if (switch_msg->kill == 0 && (quad_sum > 1 || activation_sum == 0))
  {
    BeAutonomous::ResetSwitchPanel();
    BeAutonomous::EndMission();
  }
  else if (!switch_msg->kill && last_kill_switch_value)
  { // Put kill switch in then out
    BeAutonomous::ResetSwitchPanel();
    BeAutonomous::EndMission();
  }
  else if (!mission_loaded && switch_msg->kill == 0 && activation_sum > 0)
  { // Ready to load new mission
    if (quad_sum == 1)
    { // No kill switch and one quadrant selected
      if (switch_msg->sw1)
      {
        load_id = rc::MISSION_A_BLACK; //Quad A Black
        light_msg.green1 = true;
        status_light_pub.publish(light_msg);
      }
      else if (switch_msg->sw2)
      {
        load_id = rc::MISSION_B_BLACK; //Quad B Black
        light_msg.green2 = true;
        status_light_pub.publish(light_msg);
      }
      else if (switch_msg->sw3)
      {
        load_id = rc::MISSION_C_BLACK; //Quad C Black
        light_msg.green3 = true;
        status_light_pub.publish(light_msg);
      }
      else if (switch_msg->sw4)
      {
        load_id = rc::MISSION_D_BLACK; //Quad D Black
        light_msg.green4 = true;
        status_light_pub.publish(light_msg);
      }

      if(switch_msg->sw5) // In -> Black on right
        black_side = rc::RIGHT;
      else // In -> Black on left
        black_side = rc::LEFT;
    }
    else if (quad_sum == 0 && switch_msg->sw5)
    {                             // Only test switch is engaged
      load_id = rc::MISSION_TEST; // Do system check
      light_msg.green1 = true;
      light_msg.red1 = true;
      light_msg.green2 = true;
      light_msg.red2 = true;
      light_msg.green3 = true;
      light_msg.red3 = true;
      light_msg.green4 = true;
      light_msg.red4 = true;
      status_light_pub.publish(light_msg);
    }

    if (load_id != last_load_id)
    {
      last_load_id = load_id;
      load_time = ros::Time::now();
      ROS_INFO("Load ID != last load ID");
    }
    else
    {
      load_duration = ros::Time::now().toSec() - load_time.toSec();
      ROS_INFO("Load duration: %f", load_duration);
      if (load_duration > loader_timer)
      {
        mission_loaded = true;
        ROS_INFO("Mission Loaded: %i", load_id);
        if (load_id < rc::MISSION_TEST)
        {
          quadrant = load_id;
        }
      }
    }
  }
  else if (switch_msg->kill && mission_loaded && activation_sum == 0)
  { // Must wait for all activation switches to be pulled
    if (load_id == rc::MISSION_TEST)
    {
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
    else if (load_id < rc::MISSION_TEST)
    {
      ROS_INFO("Starting mission. Maelstrom sinking to bottom of ocean in %f sec.", start_timer - pre_start_duration);
      if (!clock_is_ticking)
      {
        pre_start_time = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        pre_start_duration = ros::Time::now().toSec() - pre_start_time.toSec();

      if (pre_start_duration > start_timer)
      {
        ROS_INFO("About to call StartTask()");
        BeAutonomous::ResetSwitchPanel();
        BeAutonomous::SendInitMsgs();
        BeAutonomous::LaunchTSlam();
      }
    }
  }
  last_kill_switch_value = switch_msg->kill;
}

void BeAutonomous::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  euler_rpy = imu_msg->rpy_deg;
  linear_accel = imu_msg->linear_accel;
}

void BeAutonomous::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  depth = depth_msg->depth;
}

void BeAutonomous::ImageCB(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  frame_width = cv_ptr->image.size().width;
  frame_height = cv_ptr->image.size().height;
  cam_center_x = frame_width / 2;
  cam_center_y = frame_height / 2;
}
