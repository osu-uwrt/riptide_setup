#include "riptide_controllers/thruster_controller.h"

#undef debug
#undef report
#undef progress

//Rotation Matrices: world relative to body, and body relative to world
tf::Matrix3x3 R_wRelb, R_bRelw;
tf::Vector3 ang_v;

#define PI 3.141592653

// Thrust limits (N):
double MIN_THRUST = -8.0;
double MAX_THRUST = 8.0;

// Vehicle mass (kg):
// TODO: Get this value from model
double MASS = 33.5;

// Vehcile volume (m^3)
// TODO: Get this value from model
// Updated on 2/21/18
double VOLUME = 0.0340;

// Gravity (m/s^2)
double GRAVITY = 9.81;

// Water density (kg/m^3)
double WATER_DENSITY = 1000.0;
double BUOYANCY = VOLUME * WATER_DENSITY * GRAVITY;

// Moments of inertia (kg*m^2)
double Ixx = 0.52607145;
double Iyy = 1.50451601;
double Izz = 1.62450600;

// Acceleration commands (m/s^):
double cmdSurge = 0.0;
double cmdSway = 0.0;
double cmdHeave = 0.0;
double cmdRoll = 0.0;
double cmdPitch = 0.0;
double cmdYaw = 0.0;

//BUOYANCY Boolean  <-
bool isBuoyant = false;

struct vector
{
  double x;
  double y;
  double z;
};

void get_transform(vector *v, tf::StampedTransform *tform)
{
  v->x = tform->getOrigin().x();
  v->y = tform->getOrigin().y();
  v->z = tform->getOrigin().z();
  return;
}

/*** Thruster Positions ***/
// Positions are in meters relative to the center of mass.
vector pos_surge_port_lo;
vector pos_surge_stbd_lo;
vector pos_sway_fwd;
vector pos_sway_aft;
vector pos_heave_port_aft;
vector pos_heave_stbd_aft;
vector pos_heave_stbd_fwd;
vector pos_heave_port_fwd;

/*** EQUATIONS ***/
// These equations solve for linear/angular acceleration in all axes

// Linear Equations
struct surge
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] =
        ((surge_port_lo[0] + surge_stbd_lo[0]) +
          (R_wRelb.getRow(0).z() * (T(BUOYANCY) - T(MASS) * T(GRAVITY))*T(isBuoyant))) /
            T(MASS) -
        T(cmdSurge);
    return true;
  }
};

struct sway
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] =
        ((sway_fwd[0] + sway_aft[0]) +
         (R_wRelb.getRow(1).z() * (T(BUOYANCY) - T(MASS) * T(GRAVITY))*T(isBuoyant))) /
            T(MASS) -
        T(cmdSway);
    return true;
  }
};

struct heave
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {

      residual[0] =
          ((heave_port_fwd[0] + heave_port_aft[0] + heave_stbd_fwd[0] + heave_stbd_aft[0]) +
           (R_wRelb.getRow(2).z() * (T(BUOYANCY) - T(MASS) * T(GRAVITY))*T(isBuoyant))) /
           T(MASS) -
           T(cmdHeave);
    return true;
  }
};

// Angular equations
struct roll
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = (heave_port_fwd[0] * T(pos_heave_port_fwd.y) + heave_stbd_fwd[0] * T(pos_heave_stbd_fwd.y) +
                   heave_port_aft[0] * T(pos_heave_port_aft.y) + heave_stbd_aft[0] * T(pos_heave_stbd_aft.y) -
                   (sway_fwd[0] * T(pos_sway_fwd.z) + sway_aft[0] * T(pos_sway_aft.z)) +
                   T(Iyy) * T(ang_v.y()) * T(ang_v.z()) - T(Izz) * T(ang_v.y()) * T(ang_v.z())) /
                      T(Ixx) -
                  T(cmdRoll);
    return true;
  }
};

struct pitch
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = (surge_port_lo[0] * T(pos_surge_port_lo.z) + surge_stbd_lo[0] * T(pos_surge_stbd_lo.z) +
                   heave_port_fwd[0] * T(-pos_heave_port_fwd.x) + heave_stbd_fwd[0] * T(-pos_heave_stbd_fwd.x) +
                   heave_port_aft[0] * T(-pos_heave_port_aft.x) + heave_stbd_aft[0] * T(-pos_heave_stbd_aft.x) +
                   T(Izz) * T(ang_v.x()) * T(ang_v.z()) - T(Ixx) * T(ang_v.x()) * T(ang_v.z())) /
                      T(Iyy) -
                  T(cmdPitch);
    return true;
  }
};

struct yaw
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = (surge_port_lo[0] * T(-pos_surge_port_lo.y) + surge_stbd_lo[0] * T(-pos_surge_stbd_lo.y) +
                   sway_fwd[0] * T(pos_sway_fwd.x) + sway_aft[0] * T(pos_sway_aft.x) +
                   T(Ixx) * T(ang_v.x()) * T(ang_v.y()) - T(Iyy) * T(ang_v.x()) * T(ang_v.y())) /
                      T(Izz) -
                  T(cmdYaw);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_controller");
  tf::TransformListener tf_listener;
  ThrusterController ThrusterController(argv, &tf_listener);
  ThrusterController.loop();
}

ThrusterController::ThrusterController(char **argv, tf::TransformListener *listener_adr)
{
  R_bRelw.setIdentity();
  R_wRelb.setIdentity();
  ang_v.setZero();
  isBuoyant = false;

  listener = listener_adr;

  thrust.header.frame_id = "base_link";

  state_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &ThrusterController::state, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &ThrusterController::depth, this); //<-
  cmd_sub = nh.subscribe<geometry_msgs::Accel>("command/accel", 1, &ThrusterController::callback, this);
  cmd_pub = nh.advertise<riptide_msgs::ThrustStamped>("command/thrust", 1);

  listener->waitForTransform("/base_link", "/surge_port_lo_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/surge_port_lo_link", ros::Time(0), tf_surge[0]);
  listener->waitForTransform("/base_link", "/surge_stbd_lo_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/surge_stbd_lo_link", ros::Time(0), tf_surge[1]);
  listener->waitForTransform("/base_link", "/sway_fwd_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/sway_fwd_link", ros::Time(0), tf_sway[0]);
  listener->waitForTransform("/base_link", "/sway_aft_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/sway_aft_link", ros::Time(0), tf_sway[1]);
  listener->waitForTransform("/base_link", "/heave_port_fwd_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/heave_port_fwd_link", ros::Time(0), tf_heave[0]);
  listener->waitForTransform("/base_link", "/heave_stbd_fwd_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/heave_stbd_fwd_link", ros::Time(0), tf_heave[1]);
  listener->waitForTransform("/base_link", "/heave_port_aft_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/heave_port_aft_link", ros::Time(0), tf_heave[2]);
  listener->waitForTransform("/base_link", "/heave_stbd_aft_link", ros::Time(0), ros::Duration(10.0));
  listener->lookupTransform("/base_link", "/heave_stbd_aft_link", ros::Time(0), tf_heave[3]);

  get_transform(&pos_surge_port_lo, &tf_surge[0]);
  get_transform(&pos_surge_stbd_lo, &tf_surge[1]);
  get_transform(&pos_sway_fwd, &tf_sway[0]);
  get_transform(&pos_sway_aft, &tf_sway[1]);
  get_transform(&pos_heave_port_fwd, &tf_heave[0]);
  get_transform(&pos_heave_stbd_fwd, &tf_heave[1]);
  get_transform(&pos_heave_port_aft, &tf_heave[2]);
  get_transform(&pos_heave_stbd_aft, &tf_heave[3]);

  google::InitGoogleLogging(argv[0]);

  // PROBLEM SETUP

  // Add residual blocks (equations)

  // Linear
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<surge, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new surge), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<sway, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new sway), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<heave, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new heave), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_stbd_aft, &heave_port_aft);

  // Angular
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<roll, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new roll), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<pitch, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new pitch), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<yaw, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new yaw), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_stbd_aft, &heave_port_aft);

  // Set constraints (min/max thruster force)

  // Surge thrusters
  problem.SetParameterLowerBound(&surge_port_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_port_lo, 0, MAX_THRUST);

  problem.SetParameterLowerBound(&surge_stbd_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_stbd_lo, 0, MAX_THRUST);

  // Sway thrusters
  problem.SetParameterLowerBound(&sway_fwd, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&sway_fwd, 0, MAX_THRUST);

  problem.SetParameterLowerBound(&sway_aft, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&sway_aft, 0, MAX_THRUST);

  // Heave thrusters
  problem.SetParameterLowerBound(&heave_port_fwd, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&heave_port_fwd, 0, MAX_THRUST);

  problem.SetParameterLowerBound(&heave_stbd_fwd, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&heave_stbd_fwd, 0, MAX_THRUST);

  problem.SetParameterLowerBound(&heave_port_aft, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&heave_port_aft, 0, MAX_THRUST);

  problem.SetParameterLowerBound(&heave_stbd_aft, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&heave_stbd_aft, 0, MAX_THRUST);

  // Configure solver
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;

#ifdef progress
  options.minimizer_progress_to_stdout = true;
#endif
}

//Get orientation from IMU
void ThrusterController::state(const riptide_msgs::Imu::ConstPtr &msg)
{
  //Get euler angles, convert to radians, and make two rotation matrices
  tf::Vector3 tf;
  vector3MsgToTF(msg->euler_rpy, tf);
  tf.setValue(tf.x()*PI/180, tf.y()*PI/180, tf.y()*PI/180);
  R_wRelb.setRPY(tf.x(), tf.y(), tf.z());
  R_bRelw = R_bRelw.transpose();

  //Get angular velocity and convert to [rad/s]
  vector3MsgToTF(msg->ang_v, ang_v);
  ang_v.setValue(ang_v.x()*PI/180, ang_v.y()*PI/180, ang_v.y()*PI/180);
}

//Get depth and determine if buoyancy should be included
void ThrusterController::depth(const riptide_msgs::Depth::ConstPtr &msg)
{
  if(msg->depth > 0.2){
    isBuoyant = true;
  } else {
    isBuoyant = false;
  }
}

void ThrusterController::callback(const geometry_msgs::Accel::ConstPtr &a)
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

  // Create stamped thrust message
  thrust.header.stamp = ros::Time::now();

  //Forces are in POS dxn of the vehicle, where thrusts are what the
  //thruster outputs (POS thrust equals NEG vehicle dxn)
  thrust.force.surge_port_lo = -surge_port_lo;
  thrust.force.surge_stbd_lo = -surge_stbd_lo;
  thrust.force.sway_fwd = -sway_fwd;
  thrust.force.sway_aft = -sway_aft;
  thrust.force.heave_port_aft = -heave_port_aft;
  thrust.force.heave_stbd_aft = -heave_stbd_aft;
  thrust.force.heave_stbd_fwd = -heave_stbd_fwd;
  thrust.force.heave_port_fwd = -heave_port_fwd;

  cmd_pub.publish(thrust);
}

void ThrusterController::loop()
{
  ros::spin();
}
