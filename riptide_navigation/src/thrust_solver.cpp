#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/JointState.h"
#include "jaws2_msgs/ThrustStamped.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

#undef debug

double mass = 24.9476;	// Mass of the vehicle (KG)

double desiredSurge = 0.0; // Desired linear acceleration in X (m/s^2)
double desiredSway = 0.0; // Desired linear acceleration in Y (m/s^2)
double desiredHeave = 0.0; // Desired linear acceleration in Z (m/s^2)

double desiredRoll = 0.0; // Desired angular acceleration about X (rad/s^2)
double desiredPitch = 0.0; // Desired angular acceleration about Y (rad/s^2)
double desiredYaw = 0.0; // Desired angular acceleration about Z (rad/s^2)

double IX = 1.0; // Moment of inertia about X 
double IY = 1.0; // Moment of inertia about Y
double IZ = 1.0; // Moment of inertia about Z

/*** Thruster Positions ***/
// Positions are in meters relative to the center of mass.
// Forward Thrusters
geometry_msgs::Vector3 pos_u1 = new geometry_msgs::Vector3(-.25, .127, -.05);
geometry_msgs::Vector3 pos_u2 = new geometry_msgs::Vector3(-.25, -.127, -.05);
geometry_msgs::Vector3 pos_u3 = new geometry_msgs::Vector3(.25, -.127, -.05);
geometry_msgs::Vector3 pos_u4 = new geometry_msgs::Vector3(.25, .127, -.05);
// Upwards Thrusters
geometry_msgs::Vector3 pos_f1 = new geometry_msgs::Vector3(-.3, -.127, 1);
geometry_msgs::Vector3 pos_f2 = new geometry_msgs::Vector3(-.3, .127, 1);
geometry_msgs::Vector3 pos_f3 = new geometry_msgs::Vector3(-.3, .127, -1);
geometry_msgs::Vector3 pos_f4 = new geometry_msgs::Vector3(-.3, -.127, -1);
// Sideways Thrusters
geometry_msgs::Vector3 pos_s1 = new geometry_msgs::Vector3(.5, 0, -1);
geometry_msgs::Vector3 pos_s2 = new geometry_msgs::Vector3(-.5, 0, -1);

/*** Thruster Definitions ***/
// f1 = forward top right		u1 = vertical back left		s1 = sideways front
// f2 = forward top left		u2 = vertical back right	s2 = sideways back
// f3 = forward bottom left		u3 = vertical front right
// f4 = forward bottom right	u4 = vertical front left

/*** EQUATIONS ***/
// These equations solve for linear/angular acceleration in all axes
// Linear Equations
struct a_surge {
	template <typename T> bool operator()(const T* const f1,
		const T* const f2,
		const T* const f3,
		const T* const f4,
		T* residual) const {
		residual[0] = (f1[0] + f2[0] + f3[0] + f4[0]) / T(mass) - T(desiredSurge); // Sum forces and subtract desired
		return true;
	}
};

struct a_sway {
	template <typename T> bool operator()(const T* const s1,
		const T* const s2,
		T* residual) const {
		residual[0] = (s1[0] + s2[0]) / T(mass) - T(desiredSway);
		return true;
	}
};

struct a_heave {
	template <typename T> bool operator()(const T* const u1,
		const T* const u2,
		const T* const u3,
		const T* const u4,
		T* residual) const {
		residual[0] = (u1[0] + u2[0] + u3[0] + u4[0]) / T(mass) - T(desiredHeave);
		return true;
	}
};

// Angular equations
struct a_roll {
	template <typename T> bool operator()(const T* const u1,
		const T* const u2,
		const T* const u3,
		const T* const u4,
		const T* const s1,
		const T* const s2,
		T* residual) const {
		residual[0] = (u1*T(pos_u1.y)+u2*T(pos_u2.y)+u3*T(pos_u3.y)+u4*T(pos_u4.y)+s1*T(pos_s1.z)+s1*T(pos_s2.z)) / T(IX) - T(desiredRoll);
		return true;
	}
};

struct a_pitch {
	template <typename T> bool operator()(const T* const f1,
		const T* const f2,
		const T* const f3,
		const T* const f4,
		const T* const u1,
		const T* const u2,
		const T* const u3,
		const T* const u4,
		T* residual) const {
		residual[0] = (f1*T(pos_f1.z) + f2*T(pos_f2.z) + f3*T(pos_f3.z) + f4*T(pos_f4.z) + u1*T(pos_u1.x) + u2*T(pos_u2.x) + u3*T(pos_u3.x) + u4*T(pos_u4.x))) / T(IY) - T(desiredPitch);
		return true;
	}
};

struct a_yaw{
	template <typename T> bool operator()(const T* const f1,
		const T* const f2,
		const T* const f3,
		const T* const f4,
		const T* const s1,
		const T* const s2,
		T* residual) const {
	residual[0] = (f1*T(pos_u1.y) + f2*T(pos_u2.y) + f3*T(pos_u3.y) + f4*T(pos_u4.y) + s1*T(pos_s1.x) + s1*T(pos_s2.x)) / T(IZ) - T(desiredYaw);
	return true;
}
};

// Solver class
class Solver
{
private:
	ros::NodeHandle nh;
	ros::Subscriber accels;
	ros::Publisher forces;
	jaws2_msgs::ThrustStamped thrust;
	double f1, f2, f3, f4;
	double u1, u2, u3, u4;
	double s1, s2;
	ceres::Problem problem;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

public:
	Solver(char** argv);
	void callback(const geometry_msgs::Accel::ConstPtr& a);
	void loop();
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "thrust_solver");
	Solver solver(argv);
	solver.loop();
}

Solver::Solver(char** argv)
{
	accels = nh.subscribe<geometry_msgs::Accel>("accel_error", 1, &Solver::callback, this); // Message to give desired accelerations
	forces = nh.advertise<jaws2_msgs::ThrustStamped>("solver/thrust", 1); // Message containing force required from each thruster to achieve given acceleration

	google::InitGoogleLogging(argv[0]);

	// PROBLEM SETUP
	// Add residual blocks (equations)
	// Linear
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_surge, 1, 1, 1, 1>(new a_surge),
		NULL,
		&f1, &f2, &f3, &f4);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_sway, 1, 1>(new a_sway),
		NULL,
		&s1, &s2);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_heave, 1, 1, 1, 1>(new a_heave),
		NULL,
		&u1, &u2, &u3, &u4);
	// Angular
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_roll, 1, 1, 1, 1, 1, 1>(new a_roll),
		NULL,
		&u1, &u2, &u3, &u4, &s1, &s2);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_pitch, 1, 1, 1, 1, 1, 1, 1, 1>(new a_pitch),
		NULL,
		&f1, &f2, &f3, &f4, &u1, &u2, &u3, &u4);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<a_yaw, 1, 1, 1, 1, 1, 1>(new a_yaw),
		NULL,
		&f1, &f2, &f3, &f4, &s1, &s2);
	// Set constraints (min, max thruster force)
	// Forward thrusters
	problem.SetParameterLowerBound(&f1, 0, -25.0);
	problem.SetParameterUpperBound(&f1, 0, 25.0);

	problem.SetParameterLowerBound(&f2, 0, -25.0);
	problem.SetParameterUpperBound(&f2, 0, 25.0);

	problem.SetParameterLowerBound(&f3, 0, -25.0);
	problem.SetParameterUpperBound(&f3, 0, 25.0);

	problem.SetParameterLowerBound(&f4, 0, -25.0);
	problem.SetParameterUpperBound(&f4, 0, 25.0);
	// Upward thrusters
	problem.SetParameterLowerBound(&u1, 0, -25.0);
	problem.SetParameterUpperBound(&u1, 0, 25.0);

	problem.SetParameterLowerBound(&u2, 0, -25.0);
	problem.SetParameterUpperBound(&u2, 0, 25.0);

	problem.SetParameterLowerBound(&u3, 0, -25.0);
	problem.SetParameterUpperBound(&u3, 0, 25.0);

	problem.SetParameterLowerBound(&u4, 0, -25.0);
	problem.SetParameterUpperBound(&u4, 0, 25.0);
	// Sideways thrusters
	problem.SetParameterLowerBound(&s1, 0, -25.0);
	problem.SetParameterUpperBound(&s1, 0, 25.0);

	problem.SetParameterLowerBound(&s2, 0, -25.0);
	problem.SetParameterUpperBound(&s2, 0, 25.0);

	// Configure solver
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;

#ifdef debug
	options.minimizer_progress_to_stdout = true;
#endif
}

void Solver::callback(const geometry_msgs::Accel::ConstPtr& a)
{
	desiredSurge = a->linear.x;
	desiredSway = a->linear.y;
	desiredHeave = a->linear.z;

	desiredRoll = a->angular.x;
	desiredPitch = a->angular.y;
	desiredYaw = a->angular.z;

	// These forced initial guesses don't make much of a difference.
	// We currently experience a sort of gimbal lock w/ or w/o them.
	f1 = 0.0;
	f2 = 0.0;
	f3 = 0.0;
	f4 = 0.0;
	u1 = 0.0;
	u2 = 0.0;
	u3 = 0.0;
	u4 = 0.0;
	s1 = 0.0;
	s2 = 0.0;

#ifdef debug
	std::cout << "Initial f1 = " << f1
		<< ", f2 = " << f2
		<< ", f3 = " << f3
		<< ", f4 = " << f4
		<< ", u1 = " << u1
		<< ", u2 = " << u2
		<< ", u3 = " << u3
		<< ", u4 = " << u4
		<< ", s1 = " << s1
		<< ", s2 = " << s2
		<< std::endl;
#endif
	
	// Solve all my problems
	ceres::Solve(options, &problem, &summary);

#ifdef debug
	std::cout << summary.FullReport() << std::endl;
	std::cout << "Final f1 = " << f1
		<< ", f2 = " << f2
		<< ", f3 = " << f3
		<< ", f4 = " << f4
		<< ", u1 = " << u1
		<< ", u2 = " << u2
		<< ", u3 = " << u3
		<< ", u4 = " << u4
		<< ", s1 = " << s1
		<< ", s2 = " << s2
		<< std::endl;
#endif

	// Get time
	ros::Time time = ros::Time::now();

	// Create stamped thrust message
	thrust.header.stamp = time;
	thrust.thrust.f1 = f1;
	thrust.thrust.f2 = f2;
	thrust.thrust.f3 = f3;
	thrust.thrust.f4 = f4;
	thrust.thrust.u1 = u1;
	thrust.thrust.u2 = u2;
	thrust.thrust.u3 = u3;
	thrust.thrust.u4 = u4;
	thrust.thrust.s1 = s1;
	thrust.thrust.s2 = s2;
	forces.publish(thrust);
}

void Solver::loop()
{
	ros::spin();
}
