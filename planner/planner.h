/*
***********************************************************************
* planner.h:
* function for motion planning of USV
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _PLANNER_H_
#define _PLANNER_H_

#include <cmath>
#include "common/logging/include/easylogging++.h"
#include "planner/common/include/plannerdata.h"

namespace ASV::planning {

class planner {
 public:
  explicit planner(const plannerdata &_plannerdata)
      : sample_time(_plannerdata.sample_time) {}
  planner() = delete;
  ~planner() {}

  planner &initializewaypoint(const Eigen::MatrixXd &_wpset) {
    PlannerRTdata.waypoint0 = _wpset.col(0);
    PlannerRTdata.waypoint1 = _wpset.col(1);
    return *this;
  }

  auto getPlannerRTdata() const noexcept { return PlannerRTdata; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  double sample_time;
  plannerRTdata PlannerRTdata;

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
};  // end class planner

}  // namespace ASV::planning

#endif /* _PLANNER_H_ */