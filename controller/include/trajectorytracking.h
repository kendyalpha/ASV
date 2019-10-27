/*
*******************************************************************************
* lineofsight.h:
* path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _TRAJECTORYTRACKING_H_
#define _TRAJECTORYTRACKING_H_

#include <iostream>
#include <stdexcept>
#include <vector>

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

#include "common/logging/include/easylogging++.h"
#include "controllerdata.h"

namespace ASV::control {

class lineofsight {
 public:
  explicit lineofsight(double _los_radius, double _capture_radius = 0)
      : los_radius(_los_radius),
        capture_radius(_capture_radius),
        desired_theta(0),
        trackerror(Eigen::Vector2d::Zero()),
        R(Eigen::Matrix2d::Zero()) {}
  lineofsight() = delete;
  virtual ~lineofsight() = default;

  bool switch2next_waypoint(const Eigen::Vector2d &_vesselposition,
                            const Eigen::Vector2d &_wp1) {
    Eigen::Vector2d _error = _vesselposition - _wp1;
    double _distance =
        std::sqrt(std::pow(_error(0), 2) + std::pow(_error(1), 2));
    if (_distance < capture_radius) return true;
    return false;
  }  // switch2next_waypoint

  // compute the orientation of LOS vector and cross-track error
  void computelospoint(const Eigen::Vector2d &_vesselposition,
                       const Eigen::Vector2d &_p0, const Eigen::Vector2d &_p1) {
    auto delta_pos = _p1 - _p0;
    double thetaK = std::atan2(delta_pos(1), delta_pos(0));
    // rotation matrix
    computeR(thetaK);

    // track error
    trackerror = R.transpose() * (_vesselposition - _p0);

    double e = trackerror(1);  // cross-track error
    double thetar = 0;
    if (e > los_radius)
      thetar = -M_PI / 2;
    else if (e < -los_radius)
      thetar = M_PI / 2;
    else
      thetar = std::asin(-e / los_radius);

    desired_theta = thetar + thetaK;

  }  // computelospoint

  double getdesired_theta() const noexcept { return desired_theta; }
  auto gettrackerror() const noexcept { return trackerror; }

  void setcaptureradius(double _captureradius) {
    capture_radius = _captureradius;
  }
  void setlosradius(double _radius) { los_radius = _radius; }

 protected:
  double los_radius;
  double capture_radius;
  double desired_theta;
  Eigen::Vector2d trackerror;
  Eigen::Matrix2d R;

 private:
  // compute the rotation matrix
  void computeR(double theta) {
    double svalue = std::sin(theta);
    double cvalue = std::cos(theta);
    R(0, 0) = cvalue;
    R(0, 1) = -svalue;
    R(1, 0) = svalue;
    R(1, 1) = cvalue;
  }  // computeR
};   // end class lineofsight

class trajectorytracking final : public lineofsight {
 public:
  trajectorytracking(const controllerdata &_controllerdata,
                     const trackerRTdata &_TrackerRTdata)
      : lineofsight(_controllerdata.los_radius,
                    _controllerdata.los_capture_radius),
        TrackerRTdata(_TrackerRTdata),
        sample_time(_controllerdata.sample_time),
        grid_points_index(0) {}

  ~trajectorytracking() = default;

  // Eigen::MatrixXd followcircle(const Eigen::Vector2d &_startposition,
  //                              const Eigen::Vector2d &_endposition,
  //                              double _radius, double _vesselheading,
  //                              double _desiredspeed) {
  //   Eigen::Vector2d delta_pos = _endposition - _startposition;
  //   double length = computevectorlength(delta_pos(1), delta_pos(0));
  //   double thetaK = computevectororientation(delta_pos(1), delta_pos(0));
  //   Eigen::MatrixXd waypoints(2, 2);
  //   if (length > 2 * _radius) {
  //     waypoints.col(0) = _startposition;
  //     waypoints.col(1) = _endposition;
  //   } else {
  //     double gamma = std::acos(length / (2 * _radius));
  //     int n = static_cast<int>(std::floor((M_PI - 2 * gamma) * 6));
  //     waypoints.resize(Eigen::NoChange, n);

  //     double _clockwise_angle = thetaK + gamma;
  //     double _anticlockwise_angle = thetaK - gamma;

  //     double delta_clockwise_angle = std::abs(
  //         restrictheadingangle(_vesselheading - _clockwise_angle + 0.5 *
  //         M_PI));
  //     double delta_anticlockwise_angle = std::abs(restrictheadingangle(
  //         _vesselheading - _anticlockwise_angle - 0.5 * M_PI));
  //     if (delta_clockwise_angle < delta_anticlockwise_angle) {
  //       // follow the clockwise circle
  //       waypoints = generatecirclepoints(
  //           _startposition + _radius *
  //           computevectorlocation(_clockwise_angle), _radius, n,
  //           _clockwise_angle - M_PI, _anticlockwise_angle);
  //     } else {
  //       // follow the anti-clockwise circle
  //       waypoints = generatecirclepoints(
  //           _startposition +
  //               _radius * computevectorlocation(_anticlockwise_angle),
  //           _radius, n, _clockwise_angle, _anticlockwise_angle + M_PI);
  //     }
  //   }
  //   return waypoints;
  // }

  // follow a set of points like grid using LOS
  void Grid_LOS(double _desired_u, const Eigen::Vector2d &_vesselposition) {
    Eigen::Vector2d wp0 =
        (Eigen::Vector2d() << grid_points_x(grid_points_index),
         grid_points_y(grid_points_index))
            .finished();
    Eigen::Vector2d wp1 =
        (Eigen::Vector2d() << grid_points_x(grid_points_index + 1),
         grid_points_y(grid_points_index + 1))
            .finished();

    if (lineofsight::switch2next_waypoint(_vesselposition, wp1)) {
      // switch waypoints
      if (grid_points_index == grid_points_x.size() - 2) {
        TrackerRTdata.trackermode = TRACKERMODE::FINISHED;
        CLOG(INFO, "LOS") << "reach the last waypoint!";
        return;
      } else
        ++grid_points_index;
    }
    CircularArcLOS(0, _desired_u, _vesselposition, wp0, wp1);
    TrackerRTdata.trackermode = TRACKERMODE::TRACKING;
    return;

  }  // Grid_LOS

  // follow a circular arc using LOS
  trajectorytracking &CircularArcLOS(double _curvature, double _desired_u,
                                     const Eigen::Vector2d &_vesselposition,
                                     const Eigen::Vector2d &_rp0,
                                     const Eigen::Vector2d &_rp1) {
    // desired u and r
    double _rot = _curvature * _desired_u;
    TrackerRTdata.v_setpoint(0) = _desired_u;
    TrackerRTdata.v_setpoint(2) = _rot;

    // desired heading
    lineofsight::computelospoint(_vesselposition, _rp0, _rp1);
    TrackerRTdata.setpoint(2) = desired_theta;

    return *this;
  }  // CircularArcLOS

  // follow a circular arc without LOS
  trajectorytracking &CircularArcLOS(double _desired_curvature,
                                     double _desired_speed,
                                     double _desired_heading) {
    // desired u and r
    double _rot = _desired_curvature * _desired_speed;
    TrackerRTdata.v_setpoint(0) = _desired_speed;
    TrackerRTdata.v_setpoint(2) = _rot;

    // desired heading
    TrackerRTdata.setpoint(2) = _desired_heading;

    return *this;
  }  // CircularArcLOS

  auto gettrackerRTdata() const noexcept { return TrackerRTdata; }
  void set_grid_points(const Eigen::VectorXd &_grid_points_x,
                       const Eigen::VectorXd &_grid_points_y) {
    assert(_grid_points_x.size() == _grid_points_y.size());
    assert(_grid_points_x.size() >= 2);
    TrackerRTdata.trackermode = TRACKERMODE::STARTED;
    grid_points_x = _grid_points_x;
    grid_points_y = _grid_points_y;
    grid_points_index = 0;  // reset the index
  }

 private:
  trackerRTdata TrackerRTdata;
  const double sample_time;  // sample time of controller
  int grid_points_index;
  Eigen::VectorXd grid_points_x;  // a set of points like grid
  Eigen::VectorXd grid_points_y;

};  // end class trajectorytracking
}  // namespace ASV::control

#endif /* _TRAJECTORYTRACKING_H_ */