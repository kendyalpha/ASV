/*
*******************************************************************************
* testrouteplanning.cc:
* unit test for route planning
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/RoutePlanning.h"
#include "common/fileIO/include/utilityio.h"

using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  common::vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // LinearDamping
      Eigen::Matrix3d::Zero(),  // LinearDamping
      Eigen::Vector3d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      3,                        // L
      0                         // B
  };

  planning::RoutePlannerRTdata _plannerRTdata{
      common::STATETOGGLE::IDLE,  // state_toggle
      0,                          // setpoints_X
      0,                          // setpoints_Y;
      0,                          // setpoints_heading;
      0,                          // setpoints_longitude;
      0,                          // setpoints_latitude;
      "0n",                       // UTM_zone
      0,                          // speed
      0,                          // los_capture_radius
      Eigen::VectorXd::Zero(2),   // Waypoint_X
      Eigen::VectorXd::Zero(2),   // Waypoint_Y
      Eigen::VectorXd::Zero(2),   // Waypoint_longitude
      Eigen::VectorXd::Zero(2)    // Waypoint_latitude
  };

  planning::RoutePlanning _planner(_plannerRTdata, _vessel);

  _plannerRTdata = _planner.generate_Coarse_Circle(0, -1, 1, -5, 0, 4)
                       .getRoutePlannerRTdata();
  std::cout << "speed: " << _plannerRTdata.speed << std::endl;
  std::cout << "los_capture_radius: " << _plannerRTdata.los_capture_radius
            << std::endl;
  std::cout << "Waypoint_X: " << _plannerRTdata.Waypoint_X << std::endl;
  std::cout << "Waypoint_Y: " << _plannerRTdata.Waypoint_Y << std::endl;
}