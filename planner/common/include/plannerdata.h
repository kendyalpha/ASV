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
#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <vector>

namespace ASV::planning {
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
  double SAMPLE_TIME;  //[s]

  /* constraints */
  double MAX_SPEED;      // maximum speed [m/s]
  double MAX_ACCEL;      // maximum acceleration [m/ss]
  double MIN_ACCEL;      // minimum acceleration [m/ss]
  double MAX_CURVATURE;  // max curvature [1/m]

  /* end condition of Frenet Lattics */
  double TARGET_COURSE_ARC_STEP;  //[m]
  double MAX_ROAD_WIDTH;          // max lateral deviation [m]
  double ROAD_WIDTH_STEP;         // road width sampling length [m]
  double MAXT;                    // max prediction time [s]
  double MINT;                    // min prediction time [s]
  double DT;                      // time tick [s]
  double MAX_SPEED_DEVIATION;     // Max speed deviation [m/s]
  double TRAGET_SPEED_STEP;       // target speed sampling length [m/s]

  /* collision check */
  double HULL_LENGTH;   // [m] Length of vessel hull
  double HULL_WIDTH;    // [m] Width of vessel hull
  double ROBOT_RADIUS;  // robot radius[m]
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

}  // namespace ASV::planning

#endif /*_PLANNERDATA_H_*/