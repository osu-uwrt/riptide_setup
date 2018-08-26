#include "riptide_controllers/thruster_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_controller");
  ThrusterController ThrusterController(argv);
  ThrusterController.Loop();
}

ThrusterController::ThrusterController(char **argv) : nh("thruster_controller") {
  // Load parameters from .yaml files or launch files
  nh.getParam("debug", debug_controller);
  ThrusterController::LoadParam("buoyancy_depth_thresh", buoyancy_depth_thresh); // Depth threshold to include buoyancy

  // Load postions of each thruster relative to CoM
  ThrusterController::LoadParam<double>("HPF/X", pos_heave_port_fwd.x);
  ThrusterController::LoadParam<double>("HPF/Y", pos_heave_port_fwd.y);
  ThrusterController::LoadParam<double>("HPF/Z", pos_heave_port_fwd.z);
  ThrusterController::LoadParam<bool>("HPF/ENABLE", enableHPF);

  ThrusterController::LoadParam<double>("HPA/X", pos_heave_port_aft.x);
  ThrusterController::LoadParam<double>("HPA/Y", pos_heave_port_aft.y);
  ThrusterController::LoadParam<double>("HPA/Z", pos_heave_port_aft.z);
  ThrusterController::LoadParam<bool>("HPA/ENABLE", enableHPA);

  ThrusterController::LoadParam<double>("HSF/X", pos_heave_stbd_fwd.x);
  ThrusterController::LoadParam<double>("HSF/Y", pos_heave_stbd_fwd.y);
  ThrusterController::LoadParam<double>("HSF/Z", pos_heave_stbd_fwd.z);
  ThrusterController::LoadParam<bool>("HSF/ENABLE", enableHSF);

  ThrusterController::LoadParam<double>("HSA/X", pos_heave_stbd_aft.x);
  ThrusterController::LoadParam<double>("HSA/Y", pos_heave_stbd_aft.y);
  ThrusterController::LoadParam<double>("HSA/Z", pos_heave_stbd_aft.z);
  ThrusterController::LoadParam<bool>("HSA/ENABLE", enableHSA);

  ThrusterController::LoadParam<double>("SWF/X", pos_sway_fwd.x);
  ThrusterController::LoadParam<double>("SWF/Y", pos_sway_fwd.y);
  ThrusterController::LoadParam<double>("SWF/Z", pos_sway_fwd.z);
  ThrusterController::LoadParam<bool>("SWF/ENABLE", enableSWF);

  ThrusterController::LoadParam<double>("SWA/X", pos_sway_aft.x);
  ThrusterController::LoadParam<double>("SWA/Y", pos_sway_aft.y);
  ThrusterController::LoadParam<double>("SWA/Z", pos_sway_aft.z);
  ThrusterController::LoadParam<bool>("SWA/ENABLE", enableSWA);

  ThrusterController::LoadParam<double>("SPL/X", pos_surge_port_lo.x);
  ThrusterController::LoadParam<double>("SPL/Y", pos_surge_port_lo.y);
  ThrusterController::LoadParam<double>("SPL/Z", pos_surge_port_lo.z);
  ThrusterController::LoadParam<bool>("SPL/ENABLE", enableSPL);

  ThrusterController::LoadParam<double>("SSL/X", pos_surge_stbd_lo.x);
  ThrusterController::LoadParam<double>("SSL/Y", pos_surge_stbd_lo.y);
  ThrusterController::LoadParam<double>("SSL/Z", pos_surge_stbd_lo.z);
  ThrusterController::LoadParam<bool>("SSL/ENABLE", enableSSL);

  // Load vehicle properties
  ThrusterController::LoadParam<double>("Mass", mass);
  ThrusterController::LoadParam<double>("Volume", volume);
  ThrusterController::LoadParam<double>("Ixx", Ixx);
  ThrusterController::LoadParam<double>("Iyy", Iyy);
  ThrusterController::LoadParam<double>("Izz", Izz);
  ThrusterController::LoadParam<double>("Buoyancy_X_POS", pos_buoyancy.x);
  ThrusterController::LoadParam<double>("Buoyancy_Y_POS", pos_buoyancy.y);
  ThrusterController::LoadParam<double>("Buoyancy_Z_POS", pos_buoyancy.z);

  std::string t = "true", f = "false"; // Use with '<expression>?a:b' --> if expression is 'true' return a, else return b
  ROS_INFO("Thruster Status:");
  ROS_INFO("\tSPL enabled: %s", enableSPL?t.c_str():f.c_str());
  ROS_INFO("\tSSL enabled: %s", enableSSL?t.c_str():f.c_str());
  ROS_INFO("\tSWF enabled: %s", enableSWF?t.c_str():f.c_str());
  ROS_INFO("\tSWA enabled: %s", enableSWA?t.c_str():f.c_str());
  ROS_INFO("\tHPF enabled: %s", enableHPF?t.c_str():f.c_str());
  ROS_INFO("\tHSF enabled: %s", enableHSF?t.c_str():f.c_str());
  ROS_INFO("\tHPA enabled: %s", enableHPA?t.c_str():f.c_str());
  ROS_INFO("\tHSA enabled: %s", enableHSA?t.c_str():f.c_str());

  R_b2w.setIdentity();
  R_w2b.setIdentity();
  euler_deg.setZero();
  euler_rpy.setZero();
  ang_v.setZero();

  isBuoyant = false;
  weight = mass*GRAVITY;
  buoyancy = volume*WATER_DENSITY*GRAVITY;

  state_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &ThrusterController::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &ThrusterController::DepthCB, this);
  cmd_sub = nh.subscribe<geometry_msgs::Accel>("/command/accel", 1, &ThrusterController::AccelCB, this);
  cmd_pub = nh.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);
  residual_pub = nh.advertise<riptide_msgs::ThrusterResiduals>("/status/controls/thruster", 1);

  // Dynamic Reconfigure Variables
  cb = boost::bind(&ThrusterController::DynamicReconfigCallback, this, _1, _2);
  server.setCallback(cb);

  // Debug variables
  if(debug_controller) {
    buoyancy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/debug/pos_buoyancy", 1);

    // Published in a message
    buoyancy_pos.vector.x = 0;
    buoyancy_pos.vector.y = 0;
    buoyancy_pos.vector.z = 0;
  }

  ThrusterController::InitThrustMsg();

  google::InitGoogleLogging(argv[0]);

  // PROBLEM SETUP
  // Add residual blocks (equations)

  // Linear
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<surge, 1, 1, 1>(new surge), NULL,
                           &surge_port_lo, &surge_stbd_lo);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<sway, 1, 1, 1>(new sway), NULL,
                           &sway_fwd, &sway_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<heave, 1, 1, 1, 1, 1>(new heave), NULL,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);

  // Angular
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<roll, 1, 1, 1, 1, 1, 1, 1>(new roll), NULL,
                           &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<pitch, 1, 1, 1, 1, 1, 1, 1>(new pitch), NULL,
                           &surge_port_lo, &surge_stbd_lo,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<yaw, 1, 1, 1, 1, 1>(new yaw), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft);

  /*problem.SetParameterLowerBound(&surge_port_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_port_lo, 0, MAX_THRUST);
  problem.SetParameterLowerBound(&surge_stbd_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_stbd_lo, 0, MAX_THRUST);*/

  if(!enableSPL) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableSPL, 1, 1>(new disableSPL), NULL, &surge_port_lo);
  }
  if(!enableSSL) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableSSL, 1, 1>(new disableSSL), NULL, &surge_stbd_lo);
  }
  if(!enableSWF) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableSWF, 1, 1>(new disableSWF), NULL, &sway_fwd);
  }
  if(!enableSWA) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableSWA, 1, 1>(new disableSWA), NULL, &sway_aft);
  }
  if(!enableHPF) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableHPF, 1, 1>(new disableHPF), NULL, &heave_port_fwd);
  }
  if(!enableHSF) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableHSF, 1, 1>(new disableHSF), NULL, &heave_stbd_fwd);
  }
  if(!enableHPA) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableHPA, 1, 1>(new disableHPA), NULL, &heave_port_aft);
  }
  if(!enableHSA) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<disableHSA, 1, 1>(new disableHSA), NULL, &heave_stbd_aft);
  }

  // Configure solver
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;

  //*** Tuning for Buoyancy ***////////////////////////////////////////////////
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tuneRoll, 1, 1, 1>(new tuneRoll), NULL,
                           &pos_buoyancy_y, &pos_buoyancy_z);
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tunePitch, 1, 1, 1>(new tunePitch), NULL,
                           &pos_buoyancy_x, &pos_buoyancy_z);
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tuneYaw, 1, 1, 1>(new tuneYaw), NULL,
                           &pos_buoyancy_x, &pos_buoyancy_y);

  //buoyancyProblem.SetParameterLowerBound(&pos_buoyancy_x, 0, 0.001);

  // Configure solver
  buoyancyOptions.max_num_iterations = 50;
  buoyancyOptions.linear_solver_type = ceres::DENSE_QR;
  /////////////////////////////////////////////////////////////////////////////

#ifdef progress
  options.minimizer_progress_to_stdout = true;
#endif
}

// Load parameter from namespace
template <typename T>
void ThrusterController::LoadParam(std::string param, T &var)
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
    std::string ns = nh.getNamespace();
    ROS_ERROR("Thruster Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void ThrusterController::InitThrustMsg()
{
  riptide_msgs::ThrustStamped thrust;
  thrust.header.stamp = ros::Time::now();
  thrust.force.surge_port_lo = 0;
  thrust.force.surge_stbd_lo = 0;
  thrust.force.sway_fwd = 0;
  thrust.force.sway_aft = 0;
  thrust.force.heave_port_aft = 0;
  thrust.force.heave_stbd_aft = 0;
  thrust.force.heave_stbd_fwd = 0;
  thrust.force.heave_port_fwd = 0;
  cmd_pub.publish(thrust);
}

// Callback for dynamic reconfigure
void ThrusterController::DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels) {
  if(debug_controller) {
    mass = config.Mass;
    volume = config.Volume;
    pos_buoyancy.x = config.Buoyancy_X_POS;
    pos_buoyancy.y = config.Buoyancy_Y_POS;
    pos_buoyancy.z = config.Buoyancy_Z_POS;

    weight = mass*GRAVITY;
    buoyancy = volume*WATER_DENSITY*GRAVITY;
  }
}

//Get orientation from IMU
void ThrusterController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  //Get euler angles, convert to radians, and make two rotation matrices
  vector3MsgToTF(imu_msg->euler_rpy, euler_deg);
  euler_rpy.setValue(euler_deg.x()*PI/180, euler_deg.y()*PI/180, euler_deg.z()*PI/180);
  R_b2w.setRPY(euler_rpy.x(), euler_rpy.y(), euler_rpy.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector
  R_w2b = R_b2w.transpose(); //World to body rotations --> body_vector = R_w2b * world_vector

  //Get angular velocity and convert to [rad/s]
  vector3MsgToTF(imu_msg->ang_vel, ang_v);
  ang_v.setValue(ang_v.x()*PI/180, ang_v.y()*PI/180, ang_v.y()*PI/180);
}

//Get depth and determine if buoyancy should be included
void ThrusterController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  if(depth_msg->depth > buoyancy_depth_thresh){
    isBuoyant = true;
  }
  else {
    isBuoyant = false;
  }
}

void ThrusterController::AccelCB(const geometry_msgs::Accel::ConstPtr &a)
{
  cmdSurge = a->linear.x;
  cmdSway = a->linear.y;
  cmdHeave = a->linear.z;
  cmdRoll = a->angular.x;
  cmdPitch = a->angular.y;
  cmdYaw = a->angular.z;

  // These forced initial guesses don't make much of a difference.
  // We currently experience a sort of gimbal lock w/ or w/o them.
  surge_port_lo = 0.0;
  surge_stbd_lo = 0.0;
  sway_fwd = 0.0;
  sway_aft = 0.0;
  heave_port_aft = 0.0;
  heave_stbd_aft = 0.0;
  heave_stbd_fwd = 0.0;
  heave_port_fwd = 0.0;

  #ifdef debug
    std::cout << "Initial surge_port_lo = " << surge_port_lo << ", surge_stbd_lo = " << surge_stbd_lo
              << ", sway_fwd = " << sway_fwd << ", sway_aft = " << sway_aft << ", heave_port_aft = " << heave_port_aft
              << ", heave_stbd_aft = " << heave_stbd_aft << ", heave_stbd_fwd = " << heave_stbd_fwd
              << ", heave_port_fwd = " << heave_port_fwd << std::endl;
  #endif

  // Solve all my problems
  ceres::Solve(options, &problem, &summary);

  #ifdef report
    std::cout << summary.FullReport() << std::endl;
  #endif

  #ifdef debug
    std::cout << "Final surge_port_lo = " << surge_port_lo << ", surge_stbd_lo = " << surge_stbd_lo
              << ", sway_fwd = " << sway_fwd << ", sway_aft = " << sway_aft << ", heave_port_aft = " << heave_port_aft
              << ", heave_stbd_aft = " << heave_stbd_aft << ", heave_stbd_fwd = " << heave_stbd_fwd
              << ", heave_port_fwd = " << heave_port_fwd << std::endl;
  #endif

  //Forces are in POS dxn of the vehicle, where thrusts are what the
  //thruster outputs (POS thrust equals NEG vehicle dxn)
  thrust.header.stamp = ros::Time::now();
  thrust.force.surge_port_lo = -surge_port_lo;
  thrust.force.surge_stbd_lo = -surge_stbd_lo;
  thrust.force.sway_fwd = -sway_fwd;
  thrust.force.sway_aft = -sway_aft;
  thrust.force.heave_port_aft = -heave_port_aft;
  thrust.force.heave_stbd_aft = -heave_stbd_aft;
  thrust.force.heave_stbd_fwd = -heave_stbd_fwd;
  thrust.force.heave_port_fwd = -heave_port_fwd;

  cmd_pub.publish(thrust);

  // Calculate residuals from EOMs
  residuals.res_surge = (surge_port_lo + surge_stbd_lo +
                        (R_w2b.getRow(0).z() * (buoyancy - weight) * isBuoyant)) / mass - cmdSurge;

  residuals.res_sway = (sway_fwd + sway_aft +
                (R_w2b.getRow(1).z() * (buoyancy - weight) * isBuoyant)) / mass - cmdSway;

  residuals.res_heave = (heave_port_fwd + heave_stbd_fwd + heave_port_aft + heave_stbd_aft +
                (R_w2b.getRow(2).z() * (buoyancy - weight) * isBuoyant)) / mass - cmdHeave;

  residuals.res_roll = ((R_w2b.getRow(1).z() * buoyancy * (-pos_buoyancy.z) +
                R_w2b.getRow(2).z() * buoyancy * pos_buoyancy.y) * isBuoyant +
                sway_fwd * (-pos_sway_fwd.z) + sway_aft * (-pos_sway_aft.z) +
                heave_port_fwd * pos_heave_port_fwd.y + heave_stbd_fwd * pos_heave_stbd_fwd.y +
                heave_port_aft * pos_heave_port_aft.y + heave_stbd_aft * pos_heave_stbd_aft.y -
                ((ang_v.z() * ang_v.y()) * (Izz - Iyy))) / Ixx - cmdRoll;

  residuals.res_pitch = ((R_w2b.getRow(0).z() * buoyancy * pos_buoyancy.z +
                R_w2b.getRow(2).z() * buoyancy * (-pos_buoyancy.x)) * isBuoyant +
                surge_port_lo * pos_surge_port_lo.z + surge_stbd_lo * pos_surge_stbd_lo.z +
                heave_port_fwd * (-pos_heave_port_fwd.x) + heave_stbd_fwd * (-pos_heave_stbd_fwd.x) +
                heave_port_aft * (-pos_heave_port_aft.x) + heave_stbd_aft * (-pos_heave_stbd_aft.x) -
                ((ang_v.x() * ang_v.z()) * (Ixx - Izz))) / Iyy - cmdPitch;

  residuals.res_yaw = ((R_w2b.getRow(0).z() * buoyancy * (-pos_buoyancy.y) +
                R_w2b.getRow(1).z() * buoyancy * pos_buoyancy.x) * isBuoyant +
                surge_port_lo * (-pos_surge_port_lo.y) + surge_stbd_lo * (-pos_surge_stbd_lo.y) +
                sway_fwd * pos_sway_fwd.x + sway_aft * pos_sway_aft.x -
                ((ang_v.y() * ang_v.x()) * (Iyy - Ixx))) / Izz - cmdYaw;

  residual_pub.publish(residuals);

  // Tune Buoyancy - locate the center of buoyancy
  // The output will only make sense if the depth, roll, and pitch controllers
  // are initialized, and the vehicle is roughly stationary in the water.
  // The output should contain non-zero distances so long as the the vehicle is
  // unable to reach a target orientation along any axis.
  // The depth controller is used only to keep the vehicle fully submerged
  if(debug_controller) {
    // Initialize values
    pos_buoyancy_x = 0.0;
    pos_buoyancy_y = 0.0;
    pos_buoyancy_z = 0.0;

    ceres::Solve(buoyancyOptions, &buoyancyProblem, &buoyancySummary);
    buoyancy_pos.header.stamp = ros::Time::now();
    buoyancy_pos.vector.x = pos_buoyancy_x;
    buoyancy_pos.vector.y = pos_buoyancy_y;
    buoyancy_pos.vector.z = pos_buoyancy_z;

    buoyancy_pub.publish(buoyancy_pos);
  }
}

void ThrusterController::Loop()
{
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    ros::spinOnce();
    rate.sleep();
  }
}
