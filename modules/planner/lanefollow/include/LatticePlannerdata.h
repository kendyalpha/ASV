/*
****************************************************************************
* LatticePlannerdata.h:
* define the data struct used in the controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _LATTICEPLANNERDATA_H_
#define _LATTICEPLANNERDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include "modules/planner/common/include/plannerdata.h"

namespace ASV::planning {

struct Frenet_path {
  Eigen::VectorXd t;
  Eigen::VectorXd d;
  Eigen::VectorXd d_dot;
  Eigen::VectorXd d_ddot;
  Eigen::VectorXd s;
  Eigen::VectorXd s_dot;
  Eigen::VectorXd s_ddot;
  Eigen::VectorXd d_prime;
  Eigen::VectorXd d_pprime;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd yaw;
  Eigen::VectorXd kappa;
  Eigen::VectorXd speed;
  Eigen::VectorXd dspeed;
  double cd;
  double cv;
  double cf;
};

struct LatticeData {
  double SAMPLE_TIME;  //[s]
  double MAX_SPEED;    //[m/s]
  /* end condition of Frenet Lattics */
  double TARGET_COURSE_ARC_STEP;  //[m]
  double MAX_ROAD_WIDTH;          // max lateral deviation [m]
  double ROAD_WIDTH_STEP;         // road width sampling length [m]
  double MAXT;                    // max prediction time [s]
  double MINT;                    // min prediction time [s]
  double DT;                      // time tick [s]
  double MAX_SPEED_DEVIATION;     // Max speed deviation [m/s]
  double TRAGET_SPEED_STEP;       // target speed sampling length [m/s]
};

struct CollisionData {
  /* constraints */
  double MAX_SPEED;      // maximum speed [m/s]
  double MAX_ACCEL;      // maximum acceleration [m/ss]
  double MIN_ACCEL;      // minimum acceleration [m/ss]
  double MAX_CURVATURE;  // max curvature [1/m]

  /* collision check */
  double HULL_LENGTH;   // [m] Length of vessel hull
  double HULL_WIDTH;    // [m] Width of vessel hull
  double ROBOT_RADIUS;  // robot radius[m]
};

}  // namespace ASV::planning

#endif /*_LATTICEPLANNERDATA_H_*/
