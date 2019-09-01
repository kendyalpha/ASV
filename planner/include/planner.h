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
#include "easylogging++.h"
#include "plannerdata.h"

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

  void setcommandfromjoystick(double tau_x, double tau_y, double tau_theta) {
    PlannerRTdata.command << tau_x, tau_y, tau_theta;
  }

  auto getPlannerRTdata() const noexcept { return PlannerRTdata; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  double sample_time;
  plannerRTdata PlannerRTdata;

  // restrict heading angle (0-2PI) to (-PI ~ PI)
  double restrictheadingangle(double _heading) noexcept {
    double heading = 0.0;
    if (_heading > M_PI)
      heading = _heading - 2 * M_PI;
    else if (_heading < -M_PI)
      heading = _heading + 2 * M_PI;
    else
      heading = _heading;
    return heading;
  }

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
};

#endif /* _PLANNER_H_ */