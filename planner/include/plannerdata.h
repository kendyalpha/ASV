/*
*******************************************************************************
* plannerdata.h:
* define the data struct used in the controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _PLANNERDATA_H_
#define _PLANNERDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace ASV {
// real time data in planner
struct plannerRTdata {
  double curvature;           // desired curvature of path
  double speed;               // desired speed forward
  Eigen::Vector2d waypoint0;  // x, y in the global coordinate
  Eigen::Vector2d waypoint1;  // x, y in the global coordinate
  Eigen::Vector3d command;    // command from joystick (human)
};

struct plannerdata {
  double sample_time;
};

// constant data using in Frenet planner
struct Frenetdata {
  const double SAMPLE_TIME;  //[s]

  /* constraints */
  const double MAX_SPEED;      // maximum speed [m/s]
  const double MAX_ACCEL;      // maximum acceleration [m/ss]
  const double MIN_ACCEL;      // minimum acceleration [m/ss]
  const double MAX_CURVATURE;  // max curvature [1/m]

  /* end condition of Frenet Lattics */
  const double TARGET_COURSE_ARC_STEP;  //[m]
  const double MAX_ROAD_WIDTH;          // max lateral deviation [m]
  const double ROAD_WIDTH_STEP;         // road width sampling length [m]
  const double MAXT;                    // max prediction time [s]
  const double MINT;                    // min prediction time [s]
  const double DT;                      // time tick [s]
  const double MAX_SPEED_DEVIATION;     // Max speed deviation [m/s]
  const double TRAGET_SPEED_STEP;       // target speed sampling length [m/s]

  /* collision check */
  const double ROBOT_RADIUS;  // robot radius[m]
};

// state in the Cartesian coodinate
struct CartesianState {
  double x;
  double y;
  double theta;
  double kappa;
  double speed;   // vx
  double dspeed;  // ax
};

// state in the Frenet coodinate
struct FrenetState {
  double s;         // s: longitudinal coordinate w.r.t reference line.
  double s_dot;     // ds / dt
  double s_ddot;    // d(s_dot) / dt
  double d;         // d: lateral coordinate w.r.t. reference line
  double d_dot;     // dd / dt
  double d_ddot;    // d(d_dot) / dt
  double d_prime;   // dd / ds
  double d_pprime;  // d(d_prime)/ ds
};

}  // end namespace ASV

#endif /*_PLANNERDATA_H_*/