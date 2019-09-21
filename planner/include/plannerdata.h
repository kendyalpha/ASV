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

// state in the Cartesian coodinate
struct CartesianState {
  double x;
  double y;
  double theta;
  double kappa;
  double speed;
  double dspeed;
};

// state in the Frenet coodinate
struct FrenetState {
  double s;      // s
  double ds_t;   // ds/dt
  double dds_t;  // dds/dt2
  double d;      // d
  double dd_t;   // dd/dt
  double ddd_t;  // ddd/dt2
  double dd_s;   // dd/ds
  double ddd_s;  // ddd/ds2
};

#endif /*_PLANNERDATA_H_*/