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

#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include "RoutePlannerData.h"
#include "common/logging/include/easylogging++.h"
#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::planning {

class RoutePlanning {
 public:
  explicit RoutePlanning(const RoutePlannerRTdata &_rtdata,
                         const common::vessel &_vesseldata)
      : routeplanner_RTdata(_rtdata), L(_vesseldata.L) {}
  virtual ~RoutePlanning() = default;

  // generate a polygon-like waypoints
  RoutePlanning &generate_Coarse_Circle(double _center_x, double _center_y,
                                        double _initial_x, double _initial_y,
                                        double _initial_heading,
                                        double _desired_speed) {
    double capture_radius = compute_capture_radius(_desired_speed, L);
    double delta_X = _initial_x - _center_x;
    double delta_Y = _initial_y - _center_y;
    double r = std::sqrt(delta_X * delta_X + delta_Y * delta_Y);
    double theta0 = std::atan2(delta_Y, delta_X);

    // ensure the waypoints are far enough
    if (r < 1.414 * capture_radius) r = 2 * capture_radius;

    // decide the clockwise or anti-clockwise
    double theta_clockwise = std::abs(common::math::Normalizeheadingangle(
        theta0 + 0.75 * M_PI - _initial_heading));
    double theta_anticlockwise = std::abs(common::math::Normalizeheadingangle(
        theta0 - 0.75 * M_PI - _initial_heading));

    double Isclockwise = theta_clockwise < theta_anticlockwise ? 1.0 : -1.0;

    // generate the waypoints
    Eigen::VectorXd Waypoint_x(5);
    Eigen::VectorXd Waypoint_y(5);
    for (int i = 0; i != 4; ++i) {
      double angle = theta0 + 0.5 * M_PI * i * Isclockwise;
      Waypoint_x(i) = _center_x + r * std::cos(angle);
      Waypoint_y(i) = _center_y + r * std::sin(angle);
    }
    Waypoint_x(4) = Waypoint_x(0);
    Waypoint_y(4) = Waypoint_y(0);

    //
    routeplanner_RTdata.speed = _desired_speed;
    routeplanner_RTdata.los_capture_radius = capture_radius;
    routeplanner_RTdata.Waypoint_X = Waypoint_x;
    routeplanner_RTdata.Waypoint_Y = Waypoint_y;
    routeplanner_RTdata.Waypoint_longitude = Waypoint_x;
    routeplanner_RTdata.Waypoint_latitude = Waypoint_y;
    routeplanner_RTdata.state_toggle = common::STATETOGGLE::READY;

    return *this;
  }  // generate_Coarse_Circle

  // RoutePlanning &generate_Grid_Points(double _center_x, double _center_y,
  //                                     double _radius, std::size_t edges = 4)
  //                                     {
  //   return *this;
  // }  // generate_Grid_Points

  // setup waypoints using longitude and latitude
  void setWaypoints(const Eigen::VectorXd &_Waypoint_longitude,
                    const Eigen::VectorXd &_Waypoint_latitude) {
    routeplanner_RTdata.utm_zone = waypoints_geo2utm(
        _Waypoint_longitude, _Waypoint_latitude, routeplanner_RTdata.Waypoint_X,
        routeplanner_RTdata.Waypoint_Y);

    routeplanner_RTdata.state_toggle = common::STATETOGGLE::READY;
  }

  // setup DP data using longitude, latitude, and heading
  void setSetpoints(double _longitude, double _latitude, double _heading) {
    int zone = 0;
    bool northp = true;
    double utm_x = 0;
    double utm_y = 0;

    routeplanner_RTdata.setpoints_longitude = _longitude;
    routeplanner_RTdata.setpoints_latitude = _latitude;

    GeographicLib::UTMUPS::Forward(_latitude, _longitude, zone, northp, utm_x,
                                   utm_y);
    routeplanner_RTdata.utm_zone =
        GeographicLib::UTMUPS::EncodeZone(zone, northp);

    std::tie(routeplanner_RTdata.setpoints_X, routeplanner_RTdata.setpoints_Y) =
        common::math::UTM2Marine(utm_x, utm_y);

    routeplanner_RTdata.setpoints_heading =
        common::math::Normalizeheadingangle(common::math::Degree2Rad(_heading));
  }  // setSetpoints

  auto getRoutePlannerRTdata() const noexcept { return routeplanner_RTdata; }

 private:
  RoutePlannerRTdata routeplanner_RTdata;
  const double L;  // Hull length

  // compute the capture radius in LOS based on Hull length and speed
  double compute_capture_radius(double _desired_speed, double _basic_radius) {
    return 1.5 * std::sqrt(_desired_speed) * _basic_radius;
  }  // compute_capture_radius

  // convert the geographic coordinate of waypoints into UTM
  std::string waypoints_geo2utm(const Eigen::VectorXd &_Waypoint_longitude,
                                const Eigen::VectorXd &_Waypoint_latitude,
                                Eigen::VectorXd &_Waypoint_X,
                                Eigen::VectorXd &_Waypoint_Y) {
    assert(_Waypoint_longitude.size() == _Waypoint_latitude.size());

    int num_wp = _Waypoint_longitude.size();
    _Waypoint_X = Eigen::VectorXd::Zero(num_wp);
    _Waypoint_Y = Eigen::VectorXd::Zero(num_wp);

    //  decide zone using the first waypoint
    std::string basic_zone_str;
    int basic_zone;
    bool basic_northp;
    double basic_utm_x = 0;
    double basic_utm_y = 0;
    GeographicLib::UTMUPS::Forward(_Waypoint_latitude(0),
                                   _Waypoint_longitude(0), basic_zone,
                                   basic_northp, basic_utm_x, basic_utm_y);
    basic_zone_str =
        GeographicLib::UTMUPS::EncodeZone(basic_zone, basic_northp);
    std::tie(_Waypoint_X(0), _Waypoint_Y(0)) =
        common::math::UTM2Marine(basic_utm_x, basic_utm_y);

    // the rest of waypoints
    for (int i = 1; i != num_wp; ++i) {
      int zone = 0;
      bool northp = true;
      double utm_x = 0;
      double utm_y = 0;
      GeographicLib::UTMUPS::Forward(_Waypoint_latitude(i),
                                     _Waypoint_longitude(i), zone, northp,
                                     utm_x, utm_y);
      zone_str = GeographicLib::UTMUPS::EncodeZone(zone, northp);

      if (zone_str != basic_zone_str) {
        // transfer one zone to another
        GeographicLib::UTMUPS::Transfer(
            zone, northp, GPSdata.UTM_x, GPSdata.UTM_y, zone_planning,
            northp_planning, smooth_utm_x, smooth_utm_y, zone_t);

        GPSdata.UTM_x = smooth_utm_x;
        GPSdata.UTM_y = smooth_utm_y;
      }

      std::tie(_Waypoint_X(i), _Waypoint_Y(i)) =
          common::math::UTM2Marine(utm_x, utm_y);
    }
    return basic_zone_str;
  }

};  // end class routeplanning

}  // namespace ASV::planning

#endif /* _ROUTEPLANNING_H_ */