/*
*******************************************************************************
* vesseldata.h:
* define the property of each vessel
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _VESSELDATA_H_
#define _VESSELDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <vector>

namespace ASV::common {

// 船体的参数
struct vessel {
  // mass property
  Eigen::Matrix3d Mass;
  Eigen::Matrix3d AddedMass;
  Eigen::Matrix3d LinearDamping;
  Eigen::Matrix3d QuadraticDamping;
  Eigen::Vector3d cog;  // x, y, z

  // thrust limit
  Eigen::Vector2d x_thrust;   // min, max
  Eigen::Vector2d y_thrust;   // min, max
  Eigen::Vector2d mz_thrust;  // min, max

  // velocity limit
  Eigen::Vector2d surge_v;  // min, max
  Eigen::Vector2d sway_v;   // min, max
  Eigen::Vector2d yaw_v;    // min, max
  Eigen::Vector2d roll_v;   // min, max

  // geometry
  double L;  // total length of vessel
  double B;  // total width of vessel
};

// 测试的模式
enum class TESTMODE {
  SIMULATION_DP = 0,     // simulation for dynamic positioning
  SIMULATION_LOS,        // simulation for los
  SIMULATION_FRENET,     // simulation for frenet planner
  SIMULATION_AVOIDANCE,  // simulation for obstacle avoidance
  EXPERIMENT_DP = 10,    // experiment for dynamic positioning
  EXPERIMENT_LOS,        // experiment for los
  EXPERIMENT_FRENET,     // experiment for frenet planner
  EXPERIMENT_AVOIDANCE,  // experiment for obstacle avoidance

};

}  // namespace ASV::common

#endif /* _VESSELDATA_H_ */
