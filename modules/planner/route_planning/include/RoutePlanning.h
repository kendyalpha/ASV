/*
***********************************************************************
* RoutePlanning.h:
* function for route navigation
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _ROUTEPLANNING_H_
#define _ROUTEPLANNING_H_

#include <cmath>
#include "RoutePlannerData.h"
#include "common/logging/include/easylogging++.h"

namespace ASV::planning {

class RoutePlanning {
 public:
  explicit RoutePlanning(const RoutePlannerRTdata &_rtdata)
      : routeplanner_RTdata(_rtdata) {}
  virtual ~RoutePlanning() = default;

  // setup waypoints using UTM_N and UTM_E
  void setWaypoints_NE(const Eigen::VectorXd &_Waypoint_X,
                       const Eigen::VectorXd &_Waypoint_Y) {
    routeplanner_RTdata.Waypoint_X = _Waypoint_X;
    routeplanner_RTdata.Waypoint_Y = _Waypoint_Y;
  }
  // setup waypoints using longitude and latitude
  void setWaypoints_LL(const Eigen::VectorXd &_Waypoint_longitude,
                       const Eigen::VectorXd &_Waypoint_latitude) {
    routeplanner_RTdata.Waypoint_longitude = _Waypoint_longitude;
    routeplanner_RTdata.Waypoint_latitude = _Waypoint_latitude;
    // TODO: convert longitude and latitude to UTM
  }

  auto getRoutePlannerRTdata() const noexcept { return routeplanner_RTdata; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  RoutePlannerRTdata routeplanner_RTdata;

  //
  Eigen::MatrixXd generatecirclepoints(double _radius, double _center_x,
                                       double _center_y, double _start_angle,
                                       double _end_angle, int n) {
    Eigen::VectorXd angles =
        Eigen::VectorXd::LinSpaced(n, _start_angle, _end_angle);
    Eigen::MatrixXd waypoints_set(2, n);
    for (int i = 0; i != n; ++i) {
      waypoints_set(0, i) = _center_x + _radius * std::cos(angles(i));
      waypoints_set(1, i) = _center_y + _radius * std::sin(angles(i));
    }
    return waypoints_set;
  }
  //
  Eigen::MatrixXd generatecirclepoints(const Eigen::Vector2d &_center,
                                       double _radius, int n,
                                       double _start_angle, double _end_angle) {
    Eigen::VectorXd angles =
        Eigen::VectorXd::LinSpaced(n, _start_angle, _end_angle);
    Eigen::MatrixXd waypoints_set(2, n);
    for (int i = 0; i != n; ++i) {
      waypoints_set.col(i) =
          _center + _radius * computevectorlocation(angles(i));
    }
    return waypoints_set;
  }

  // compute the vector length
  double computevectorlength(double _vy, double _vx) noexcept {
    return std::sqrt(std::pow(_vy, 2) + std::pow(_vx, 2));
  }
  // compute the
  Eigen::Vector2d computevectorlocation(double _angle) {
    return (Eigen::Vector2d() << std::cos(_angle), std::sin(_angle)).finished();
  }
};  // end class routeplanning

}  // namespace ASV::planning

#endif /* _ROUTEPLANNING_H_ */