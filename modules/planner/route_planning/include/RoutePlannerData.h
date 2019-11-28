
/*
*******************************************************************************
* RoutePlannerData.h:
* define the data struct used in the route planner
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _ROUTEPLANNERDATA_H_
#define _ROUTEPLANNERDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <vector>
#include "common/property/include/priority.h"
#include "common/property/include/vesseldata.h"

namespace ASV::planning {

// real time data in route planner (GUI-dependent or auto), but the these data
// should not be modified too fast
struct RoutePlannerRTdata {
  /********************* state toggle  *********************/
  common::STATETOGGLE state_toggle;

  // DP data
  double setpoints_X;          // X in the marine coordinate
  double setpoints_Y;          // Y in the marine coordinate
  double setpoints_heading;    // theta in the UTM coordinate
  double setpoints_longitude;  // longitude of the setpoint
  double setpoints_latitude;   // latitude of the setpoint

  //
  std::string utm_zone;  // UTM zone

  //
  double speed;  // desired speed forward
  double los_capture_radius;
  Eigen::VectorXd Waypoint_X;          // X in the marine coordinate
  Eigen::VectorXd Waypoint_Y;          // Y in the marine coordinate
  Eigen::VectorXd Waypoint_longitude;  // longitude
  Eigen::VectorXd Waypoint_latitude;   // latitude
};

}  // namespace ASV::planning

#endif /*_ROUTEPLANNERDATA_H_*/
