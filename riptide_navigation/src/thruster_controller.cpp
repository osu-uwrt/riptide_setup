#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/ThrustStamped.h"
#include "tf/transform_listener.h"

#include <math.h>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

#undef debug
#undef report
#undef progress

tf::Matrix3x3 rotation_matrix;
tf::Vector3 ang_v;

// Thrust limits (N):
double MIN_THRUST = -18.0;
double MAX_THRUST = 18.0;

// Vehicle mass (kg):
double MASS = 34.47940950;

// Moments of inertia (kg*m^2)
double Ixx = 0.50862680;
double Iyy = 1.70892348;
double Izz = 1.77586420;

// Acceleration commands (m/s^):
double cmdSurge = 0.0;
double cmdSway = 0.0;
double cmdHeave = 0.0;
double cmdRoll = 0.0;
double cmdPitch = 0.0;
double cmdYaw = 0.0;

struct vector
{
		double x;
		double y;
		double z;
};

void get_transform(vector &v, tf::StampedTransform tform)
{
	v.x = tform.getOrigin().x();
	v.y = tform.getOrigin().y();
	v.z = tform.getOrigin().z();
	return;
}

/*** Thruster Positions ***/
// Positions are in meters relative to the center of mass.
vector pos_surge_stbd_hi;
vector pos_surge_port_hi;
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
struct surge {
	template <typename T> bool operator()(const T* const surge_port_hi,
																				const T* const surge_stbd_hi,
																				const T* const surge_port_lo,
																				const T* const surge_stbd_lo,
																				const T* const sway_fwd,
																				const T* const sway_aft,
																				const T* const heave_port_fwd,
																				const T* const heave_stbd_fwd,
																				const T* const heave_port_aft,
																				const T* const heave_stbd_aft,
																				T* residual) const
	{
		residual[0] = (rotation_matrix.getRow(0).x()*(surge_port_hi[0] + surge_stbd_hi[0] + surge_port_lo[0] + surge_stbd_lo[0]) + rotation_matrix.getRow(0).y()*(sway_fwd[0] + sway_aft[0]) + rotation_matrix.getRow(0).z()*(heave_port_fwd[0] + heave_stbd_fwd[0] + heave_port_aft[0] + heave_stbd_aft[0])) / T(MASS) - T(cmdSurge);
		return true;
	}
};

struct sway {
	template <typename T> bool operator()(const T* const surge_port_hi,
																				const T* const surge_stbd_hi,
																				const T* const surge_port_lo,
																				const T* const surge_stbd_lo,
																				const T* const sway_fwd,
																				const T* const sway_aft,
																				const T* const heave_port_fwd,
																				const T* const heave_stbd_fwd,
																				const T* const heave_port_aft,
																				const T* const heave_stbd_aft,
																				T* residual) const
	{
		residual[0] = (rotation_matrix.getRow(1).x()*(surge_port_hi[0] + surge_stbd_hi[0] + surge_port_lo[0] + surge_stbd_lo[0]) + rotation_matrix.getRow(1).y()*(sway_fwd[0] + sway_aft[0]) + rotation_matrix.getRow(1).z()*(heave_port_fwd[0] + heave_stbd_fwd[0] + heave_stbd_aft[0] + heave_port_aft[0])) / T(MASS) - T(cmdSway);
		return true;
	}
};

struct heave {
	template <typename T> bool operator()(const T* const surge_port_hi,
																				const T* const surge_stbd_hi,
																				const T* const surge_port_lo,
																				const T* const surge_stbd_lo,
																				const T* const sway_fwd,
																				const T* const sway_aft,
																				const T* const heave_port_fwd,
																				const T* const heave_stbd_fwd,
																				const T* const heave_port_aft,
																				const T* const heave_stbd_aft,
																				T* residual) const
	{
		residual[0] = (rotation_matrix.getRow(2).x()*(surge_port_hi[0] + surge_stbd_hi[0] + surge_port_lo[0] + surge_stbd_lo[0]) + rotation_matrix.getRow(2).y()*(sway_fwd[0] + sway_aft[0]) + rotation_matrix.getRow(2).z()*(heave_port_fwd[0] + heave_stbd_fwd[0] + heave_port_aft[0] + heave_stbd_aft[0])) / T(MASS) - T(cmdHeave);
		return true;
	}
};

// Angular equations
struct roll {
	template <typename T> bool operator()(const T* const sway_fwd,
																				const T* const sway_aft,
																				const T* const heave_port_fwd,
																				const T* const heave_stbd_fwd,
																				const T* const heave_port_aft,
																				const T* const heave_stbd_aft,
																				T* residual) const
	{
		residual[0] = (heave_port_fwd[0]*T(pos_heave_port_fwd.y)+heave_stbd_fwd[0]*T(pos_heave_stbd_fwd.y)+heave_port_aft[0]*T(pos_heave_port_aft.y)+heave_stbd_aft[0]*T(pos_heave_stbd_aft.y)-(sway_fwd[0]*T(pos_sway_fwd.z)+sway_aft[0]*T(pos_sway_aft.z)) + T(Iyy) * T(ang_v.y()) * T(ang_v.z()) - T(Izz) * T(ang_v.y()) * T(ang_v.z())) / T(Ixx) - T(cmdRoll);
		return true;
	}
};

struct pitch {
	template <typename T> bool operator()(const T* const surge_port_hi,
																				const T* const surge_stbd_hi,
																				const T* const surge_port_lo,
																				const T* const surge_stbd_lo,
																				const T* const heave_port_fwd,
																				const T* const heave_stbd_fwd,
																				const T* const heave_port_aft,
																				const T* const heave_stbd_aft,
																				T* residual) const
	{
		residual[0] = (surge_port_hi[0] * T(pos_surge_port_hi.z) + surge_stbd_hi[0]*T(pos_surge_stbd_hi.z) + surge_port_lo[0]*T(pos_surge_port_lo.z) + surge_stbd_lo[0]*T(pos_surge_stbd_lo.z) + heave_port_fwd[0]*T(-pos_heave_port_fwd.x) + heave_stbd_fwd[0]*T(-pos_heave_stbd_fwd.x) + heave_port_aft[0]*T(-pos_heave_port_aft.x) + heave_stbd_aft[0]*T(-pos_heave_stbd_aft.x) + T(Izz) * T(ang_v.x()) * T(ang_v.z()) - T(Ixx) * T(ang_v.x()) * T(ang_v.z())) / T(Iyy) - T(cmdPitch);
		return true;
	}
};

struct yaw{
	template <typename T> bool operator()(const T* const surge_port_hi,
																				const T* const surge_stbd_hi,
																				const T* const surge_port_lo,
																				const T* const surge_stbd_lo,
																				const T* const sway_fwd,
																				const T* const sway_aft,
																			 	T* residual) const
	{
		residual[0] = (surge_port_hi[0]*T(-pos_surge_port_hi.y) + surge_stbd_hi[0]*T(-pos_surge_stbd_hi.y) + surge_port_lo[0]*T(-pos_surge_port_lo.y) + surge_stbd_lo[0]*T(-pos_surge_stbd_lo.y) + sway_fwd[0]*T(pos_sway_fwd.x) + sway_aft[0]*T(pos_sway_aft.x) + T(Ixx) * T(ang_v.x()) * T(ang_v.y()) - T(Iyy) * T(ang_v.x()) * T(ang_v.y())) / T(Izz) - T(cmdYaw);
		return true;
	}
};

class Solver
{
	private:
		// Comms
		ros::NodeHandle nh;
		ros::Subscriber state_sub;
		ros::Subscriber cmd_sub;
		ros::Publisher cmd_pub;
		riptide_msgs::ThrustStamped thrust;
		// Math
		ceres::Problem problem;
		ceres::Solver::Options options;
		ceres::Solver::Summary summary;
		// Results
		double surge_stbd_hi, surge_port_hi, surge_port_lo, surge_stbd_lo;
		double sway_fwd, sway_aft;
		double heave_port_aft, heave_stbd_aft, heave_stbd_fwd, heave_port_fwd;
		// TF
		tf::TransformListener *listener;
		tf::StampedTransform tf_surge[4];
		tf::StampedTransform tf_sway[2];
		tf::StampedTransform tf_heave[4];

	public:
		Solver(char** argv, tf::TransformListener& listener_adr);
		void state(const sensor_msgs::Imu::ConstPtr& msg);
		void callback(const geometry_msgs::Accel::ConstPtr& a);
		void loop();
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "thruster_controller");
  tf::TransformListener tf_listener;
	Solver solver(argv, tf_listener);
	solver.loop();
}

Solver::Solver(char** argv, tf::TransformListener& listener_adr)
{
  rotation_matrix.setIdentity();
	ang_v.setZero();

	listener = &listener_adr;

	thrust.header.frame_id = "base_link";

	state_sub = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, &Solver::state, this);
	cmd_sub = nh.subscribe<geometry_msgs::Accel>("command/accel", 1, &Solver::callback, this);
	cmd_pub = nh.advertise<riptide_msgs::ThrustStamped>("command/thrust", 1);

	listener->waitForTransform("/base_link", "/surge_port_hi_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/surge_port_hi_link", ros::Time(0), tf_surge[0]);
	listener->waitForTransform("/base_link", "/surge_stbd_hi_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/surge_stbd_hi_link", ros::Time(0), tf_surge[1]);
  	listener->waitForTransform("/base_link", "/surge_port_lo_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/surge_port_lo_link", ros::Time(0), tf_surge[2]);
  	listener->waitForTransform("/base_link", "/surge_stbd_lo_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/surge_stbd_lo_link", ros::Time(0), tf_surge[3]);
	listener->waitForTransform("/base_link", "/sway_fwd_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/sway_fwd_link", ros::Time(0), tf_sway[0]);
	listener->waitForTransform("/base_link", "/sway_aft_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/sway_aft_link", ros::Time(0), tf_sway[1]);
	listener->waitForTransform("/base_link", "/heave_port_fwd_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/heave_port_fwd_link", ros::Time(0), tf_heave[0]);
	listener->waitForTransform("/base_link", "/heave_stbd_fwd_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/heave_stbd_fwd_link", ros::Time(0), tf_heave[1]);
	listener->waitForTransform("/base_link", "/heave_port_aft_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/heave_port_aft_link", ros::Time(0), tf_heave[2]);
	listener->waitForTransform("/base_link", "/heave_stbd_aft_link", ros::Time(0), ros::Duration(10.0) );
	listener->lookupTransform("/base_link", "/heave_stbd_aft_link", ros::Time(0), tf_heave[3]);

	get_transform(pos_surge_port_hi, tf_surge[0]);
	get_transform(pos_surge_stbd_hi, tf_surge[1]);
	get_transform(pos_surge_port_lo, tf_surge[2]);
	get_transform(pos_surge_stbd_lo, tf_surge[3]);
	get_transform(pos_sway_fwd, tf_sway[0]);
	get_transform(pos_sway_aft, tf_sway[1]);
	get_transform(pos_heave_port_fwd, tf_heave[0]);
	get_transform(pos_heave_stbd_fwd, tf_heave[1]);
	get_transform(pos_heave_port_aft, tf_heave[2]);
	get_transform(pos_heave_stbd_aft, tf_heave[3]);

	google::InitGoogleLogging(argv[0]);

	// PROBLEM SETUP

	// Add residual blocks (equations)

	// Linear
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<surge, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new surge),
		NULL,
		&surge_port_hi, &surge_stbd_hi, &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft, &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<sway, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new sway),
		NULL,
		&surge_port_hi, &surge_stbd_hi, &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft, &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<heave, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new heave),
		NULL,
		&surge_port_hi, &surge_stbd_hi, &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft, &heave_port_fwd, &heave_stbd_fwd, &heave_stbd_aft, &heave_port_aft);

	// Angular
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<roll, 1, 1, 1, 1, 1, 1, 1>(new roll),
		NULL,
		&sway_fwd, &sway_aft, &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<pitch, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new pitch),
		NULL,
		&surge_port_hi, &surge_stbd_hi, &surge_port_lo, &surge_stbd_lo, &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<yaw, 1, 1, 1, 1, 1, 1, 1>(new yaw),
		NULL,
		&surge_port_hi, &surge_stbd_hi, &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft);

	// Set constraints (min/max thruster force)

	// Surge thrusters
	problem.SetParameterLowerBound(&surge_port_hi, 0, MIN_THRUST);
	problem.SetParameterUpperBound(&surge_port_hi, 0, MAX_THRUST);

	problem.SetParameterLowerBound(&surge_stbd_hi, 0, MIN_THRUST);
	problem.SetParameterUpperBound(&surge_stbd_hi, 0, MAX_THRUST);

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

void Solver::state(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion tf;
  quaternionMsgToTF(msg->orientation, tf);
	rotation_matrix.setRotation(tf.normalized());
	vector3MsgToTF(msg->angular_velocity, ang_v);
}

void Solver::callback(const geometry_msgs::Accel::ConstPtr& a)
{
	cmdSurge = a->linear.x;
	cmdSway = a->linear.y;
	cmdHeave = a->linear.z;

	cmdRoll = a->angular.x;
	cmdPitch = a->angular.y;
	cmdYaw = a->angular.z;

	// These forced initial guesses don't make much of a difference.
	// We currently experience a sort of gimbal lock w/ or w/o them.
	surge_stbd_hi = 0.0;
	surge_port_hi = 0.0;
	surge_port_lo = 0.0;
	surge_stbd_lo = 0.0;
	sway_fwd = 0.0;
	sway_aft = 0.0;
	heave_port_aft = 0.0;
	heave_stbd_aft = 0.0;
	heave_stbd_fwd = 0.0;
	heave_port_fwd = 0.0;

#ifdef debug
	std::cout << "Initial surge_stbd_hi = " << surge_stbd_hi
						<< ", surge_port_hi = " << surge_port_hi
						<< ", surge_port_lo = " << surge_port_lo
						<< ", surge_stbd_lo = " << surge_stbd_lo
						<< ", sway_fwd = " << sway_fwd
						<< ", sway_aft = " << sway_aft
						<< ", heave_port_aft = " << heave_port_aft
						<< ", heave_stbd_aft = " << heave_stbd_aft
						<< ", heave_stbd_fwd = " << heave_stbd_fwd
						<< ", heave_port_fwd = " << heave_port_fwd
						<< std::endl;
#endif

	// Solve all my problems
	ceres::Solve(options, &problem, &summary);

#ifdef report
	std::cout << summary.FullReport() << std::endl;
#endif

#ifdef debug
	std::cout << "Final surge_stbd_hi = " << surge_stbd_hi
						<< ", surge_port_hi = " << surge_port_hi
						<< ", surge_port_lo = " << surge_port_lo
						<< ", surge_stbd_lo = " << surge_stbd_lo
						<< ", sway_fwd = " << sway_fwd
						<< ", sway_aft = " << sway_aft
						<< ", heave_port_aft = " << heave_port_aft
						<< ", heave_stbd_aft = " << heave_stbd_aft
						<< ", heave_stbd_fwd = " << heave_stbd_fwd
						<< ", heave_port_fwd = " << heave_port_fwd
						<< std::endl;
#endif

	// Create stamped thrust message
	thrust.header.stamp = ros::Time::now();;

	thrust.force.surge_stbd_hi = surge_stbd_hi;
	thrust.force.surge_port_hi = surge_port_hi;
	thrust.force.surge_port_lo = surge_port_lo;
	thrust.force.surge_stbd_lo = surge_stbd_lo;
	thrust.force.sway_fwd = sway_fwd;
	thrust.force.sway_aft = sway_aft;
	thrust.force.heave_port_aft = heave_port_aft;
	thrust.force.heave_stbd_aft = heave_stbd_aft;
	thrust.force.heave_stbd_fwd = heave_stbd_fwd;
	thrust.force.heave_port_fwd = heave_port_fwd;

	cmd_pub.publish(thrust);
}

void Solver::loop()
{
	ros::spin();
}
