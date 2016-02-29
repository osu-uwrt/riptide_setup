#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/JointState.h"
#include "jaws2_msgs/ThrustStamped.h"

#include <math.h>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

#undef debug

double r = 0.10;
double l = 0.60;

double r_x = 0.14;
double r_y = 0.13;
double r_z = 0.10;

double mass = 9.4;

double lin_x = 0.0;
double lin_z = 0.0;
double ang_x = 0.0;
double ang_y = 0.0;
double ang_z = 0.0;

struct af_x {
  template <typename T> bool operator()(const T* const f_a,
                                        const T* const f_s,
                                        const T* const f_p,
                                        const T* const t_s,
                                        const T* const t_p,
                                        T* residual) const {
    residual[0] = ( f_a[0] + f_s[0] * sin( t_s[0] ) + f_p[0] * sin( t_p[0] ) ) / T(mass) - T(lin_x);
    return true;
  }
};

struct af_z {
  template <typename T> bool operator()(const T* const f_s,
                                        const T* const f_p,
                                        const T* const t_s,
                                        const T* const t_p,
                                        T* residual) const {
    residual[0] = ( f_s[0] * cos( t_s[0] ) + f_p[0] * cos( t_p[0] ) ) / T(mass) - T(lin_z);
    return true;
  }
};

struct at_x {
  template <typename T> bool operator()(const T* const f_s,
                                        const T* const f_p,
                                        const T* const t_s,
                                        const T* const t_p,
                                        T* residual) const {
    residual[0] = ( T(-r_y) * f_s[0] * cos( t_s[0] ) + T(r_y) * f_p[0] * cos( t_p[0] ) ) / T(mass * r * r / 2) - T(ang_x);
    return true;
  }
};

struct at_y {
  template <typename T> bool operator()(const T* const f_a,
                                        const T* const f_s,
                                        const T* const f_p,
                                        const T* const t_s,
                                        const T* const t_p,
                                        T* residual) const {
    residual[0] = ( T(r_z) * f_a[0] - T(r_x) * f_s[0] * cos( t_s[0] ) - T(r_x) * f_p[0] * cos( t_p[0] ) ) / T(mass * (3 * r * r + l * l) / 12) - T(ang_y);
    return true;
  }
};

struct at_z {
  template <typename T> bool operator()(const T* const f_s,
                                        const T* const f_p,
                                        const T* const t_s,
                                        const T* const t_p,
                                        T* residual) const {
    residual[0] = ( T(r_y) * f_s[0] * sin( t_s[0] ) - T(r_y) * f_p[0] * sin( t_p[0] ) ) / T(mass * (3 * r * r + l * l) / 12) - T(ang_z);
    return true;
  }
};

class Solver
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber accels;
    ros::Publisher joints;
    ros::Publisher forces;
    sensor_msgs::JointState angles;
    jaws2_msgs::ThrustStamped thrust;
    double f_a;
    double f_s;
    double f_p;
    double t_s;
    double t_p;
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
  ros::init(argc, argv, "thrust_mapper");
  Solver solver(argv);
  solver.loop();
}

Solver::Solver(char** argv)
{
  accels = nh.subscribe<geometry_msgs::Accel>("accel_error", 1, &Solver::callback, this);
  joints = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  forces = nh.advertise<jaws2_msgs::ThrustStamped>("solver/thrust", 1);

  angles.name.resize(2);
  angles.name[0] = "port-base";
  angles.name[1] = "stbd-base";
  angles.position.resize(2);

  google::InitGoogleLogging(argv[0]);

  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<af_x, 1, 1, 1, 1, 1, 1>(new af_x),
                           NULL,
                           &f_a, &f_s, &f_p, &t_s, &t_p);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<af_z, 1, 1, 1, 1, 1>(new af_z),
                           NULL,
                           &f_s, &f_p, &t_s, &t_p);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<at_x, 1, 1, 1, 1, 1>(new at_x),
                           NULL,
                           &f_s, &f_p, &t_s, &t_p);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<at_y, 1, 1, 1, 1, 1, 1>(new at_y),
                           NULL,
                           &f_a, &f_s, &f_p, &t_s, &t_p);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<at_z, 1, 1, 1, 1, 1>(new at_z),
                           NULL,
                           &f_s, &f_p, &t_s, &t_p);

  problem.SetParameterLowerBound(&f_a, 0, -25.0);
  problem.SetParameterUpperBound(&f_a, 0, 25.0);

  problem.SetParameterLowerBound(&f_s, 0, -25.0);
  problem.SetParameterUpperBound(&f_s, 0, 25.0);

  problem.SetParameterLowerBound(&f_p, 0, -25.0);
  problem.SetParameterUpperBound(&f_p, 0, 25.0);

  problem.SetParameterLowerBound(&t_s, 0, -M_PI/2);
  problem.SetParameterUpperBound(&t_s, 0, M_PI/2);

  problem.SetParameterLowerBound(&t_p, 0, -M_PI/2);
  problem.SetParameterUpperBound(&t_p, 0, M_PI/2);

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;

#ifdef debug
  options.minimizer_progress_to_stdout = true;
#endif
}

void Solver::callback(const geometry_msgs::Accel::ConstPtr& a)
{
  lin_x = a->linear.x;
  lin_z = a->linear.z;

  ang_x = a->angular.x;
  ang_y = a->angular.y;
  ang_z = a->angular.z;

  // These forced initial guesses don't make much of a difference.
  // We currently experience a sort of gimbal lock w/ or w/o them.
  f_a = 0.0;
  f_s = 0.0;
  f_p = 0.0;
  t_s = 0.0;
  t_p = 0.0;

#ifdef debug
  std::cout << "Initial f_a = " << f_a
            << ", f_s = " << f_s
            << ", f_p = " << f_p
            << ", t_s = " << t_s
            << ", t_p = " << t_p
            << std::endl;
#endif

  ceres::Solve(options, &problem, &summary);

#ifdef debug
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Final f_a = " << f_a
            << ", f_s = " << f_s
            << ", f_p = " << f_p
            << ", t_s = " << t_s
            << ", t_p = " << t_p
            << std::endl;
#endif

  ros::Time time = ros::Time::now();
  angles.header.stamp = time;
  angles.position[0] = t_p;
  angles.position[1] = t_s;
  joints.publish(angles);


  thrust.header.stamp = time;
  thrust.thrust.aft = f_a;
  thrust.thrust.stbd = f_s;
  thrust.thrust.port = f_p;
  forces.publish(thrust);
}

void Solver::loop()
{
  ros::spin();
}
