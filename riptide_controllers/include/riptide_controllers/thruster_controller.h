#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include <math.h>

#include "ceres/ceres.h"
//#include "glog/logging.h"

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <riptide_controllers/VehiclePropertiesConfig.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/NetLoad.h"

#include <yaml-cpp/yaml.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
using namespace Eigen;
using namespace std;

#define PI 3.141592653
#define GRAVITY 9.81         //[m/s^2]
#define WATER_DENSITY 1000.0 //[kg/m^3]

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, Dynamic, Dynamic, RowMajor> RowMatrixXd;

class ThrusterController
{
private:
  ros::NodeHandle nh;
  ros::Subscriber state_sub, cmd_sub, depth_sub;
  ros::Publisher cmd_pub, cob_pub;

  riptide_msgs::ThrustStamped thrust_msg;
  geometry_msgs::Vector3Stamped cob_msg;

  // Dynamic Reconfigure Setup
  bool tune; // If true, key params can be input via messages
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig> server;
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig>::CallbackType cb;

  YAML::Node properties;
  string properties_file;
  vector<int> thrustersEnabled;
  Vector3d CoB;
  double mass, volume, Fg, Fb, Ixx, Iyy, Izz, depth_fully_submerged;
  bool isSubmerged;

  // Variables that get passed to class EOM and FindCoB
  MatrixXd thrustCoeffs, thrusters;
  Vector6d weightLoad_eig;
  int numThrusters;
  double inertia[6], weightLoad[6], transportThm[6], command[6], Fb_vector[3];
  double solver_forces[8]; // Solved forces go here
  double solver_cob[3];    // Solved buoyancy positions go here

  // EOMs
  ceres::Problem problemEOM;
  ceres::Solver::Options optionsEOM;
  ceres::Solver::Summary summaryEOM;

  // Locate Buoyancy Position
  ceres::Problem problemBuoyancy;
  ceres::Solver::Options optionsBuoyancy;
  ceres::Solver::Summary summaryBuoyancy;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThrusterController();
  template <typename T>
  void LoadParam(std::string param, T &var);
  void LoadVehicleProperties();
  void SetThrusterCoeffs();
  void InitThrustMsg();
  void DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels);
  void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void NetLoadCB(const riptide_msgs::NetLoad::ConstPtr &load_msg);
  void Loop();
};

// Class EOM defines the 6 equations of motion that ceres needs to solve
class EOM
{
private:
  int numThrusters;      // Number of thrusters
  MatrixXd thrustCoeffs; // Thrust coefficients (effective contributions of each thruster for force and moments)
  double *inertia;       // (Pointer) Inertia-related values (mass, mass, mass, Ixx, Iyy, Izz)
  double *weightLoad;    // (Pointer) Forces/moments due to weight forces (gravity and buoyancy)
  double *transportThm;  // (Pointer) Vector containng terms related to the transport theorem
  double *command;       // (Pointer) Command for each EOM

public:
  EOM(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *inertiaIn, double *weightLoadIn, double *transportThmIn, double *commandIn)
  {
    numThrusters = numThrust;
    thrustCoeffs = thrustCoeffsIn;
    inertia = inertiaIn;
    weightLoad = weightLoadIn;
    transportThm = transportThmIn;
    command = commandIn;
  }

  template <typename T>
  bool operator()(const T *const forces, T *residual) const
  {
    for (int i = 0; i < 6; i++)
    {
      residual[i] = T(0);

      // Account for each thruster's contribution to force/moment
      for (int j = 0; j < numThrusters; j++)
      {
        residual[i] = residual[i] + T(thrustCoeffs(i, j)) * forces[j];
      }

      // Account for weight-related forces/moments and transportThm
      residual[i] = residual[i] + T(weightLoad[i] + transportThm[i]);
      residual[i] = residual[i] - T(command[i]);
    }
    return true;
  }
};

// class FindCoB will determine an estimate for the location of the center of buoyancy relative
// to the center of mass
class FindCoB
{
private:
  int numThrusters;      // Number of thrusters
  int x, y, z;           // Axis indeces
  MatrixXd thrustCoeffs; // Thrust coefficients (effective contributions of each thruster for force and moments)
  double *Fb;            // (Pointer) Buoyant forces along body-frame x, y, and x axes
  double *transportThm;  // (Pointer) Vector containng terms related to the transport theorem
  double *forces;        // (Pointer) Solved thruster forces from class EOM

public:
  FindCoB(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *FbIn, double *transportThmIn, double *forcesIn)
  {
    numThrusters = numThrust;
    thrustCoeffs = thrustCoeffsIn;
    Fb = FbIn;
    transportThm = transportThmIn;
    forces = forcesIn;
    x = 0, y = 1, z = 2;
  }

  // rFb is the position vector for the center of buoyancy from the center of mass
  template <typename T>
  bool operator()(const T *const rFb, T *residual) const
  {
    for (int i = 0; i < 3; i++)
    {
      residual[i] = T(transportThm[i + 3]);
      for (int j = 0; j < numThrusters; j++)
        residual[i] = residual[i] + T(thrustCoeffs(j, i) * forces[j]);
    }
    residual[x] = residual[x] + rFb[y] * Fb[z] - rFb[z] * Fb[y];
    residual[y] = residual[y] + rFb[z] * Fb[x] - rFb[x] * Fb[z];
    residual[z] = residual[z] + rFb[x] * Fb[y] - rFb[y] * Fb[x];
  }
};

#endif
