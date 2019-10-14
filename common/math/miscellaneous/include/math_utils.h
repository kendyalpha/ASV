
/*
***********************************************************************
* math_utils.h: Math-related util functions
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include <cmath>
#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <tuple>

namespace ASV::common::math {

// restrict heading angle or delta heading to (-PI ~ PI)
// compute the delta heading to find the shortest way to rotate
double Normalizeheadingangle(double _heading) noexcept {
  double a = std::fmod(_heading + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}  // Normalizeheadingangle

// convert marine coordinate to cartesian coordinate
std::tuple<double, double, double> Marine2Cart(double marine_y,
                                               double marine_theta,
                                               double marine_kappa) noexcept {
  return {-marine_y, -marine_theta, -marine_kappa};
}  // Marine2Cart

// convert marine coordinate to cartesian coordinate
Eigen::VectorXd Marine2Cart(const Eigen::VectorXd &_marine_y) noexcept {
  return (-1) * _marine_y;
}  // Marine2Cart

// convert marine coordinate to cartesian coordinate
std::tuple<double, double, double> Cart2Marine(double cart_y, double cart_theta,
                                               double cart_kappa) noexcept {
  return {-cart_y, -cart_theta, -cart_kappa};

}  // Marine2Cart

// convert rad to degree
double Rad2Degree(double _rad) noexcept {
  return 180.0 * _rad / M_PI;
}  // Rad2Degree

// convert marine coordinate to cartesian coordinate
double Degree2Rad(double _degree) noexcept {
  return M_PI * _degree / 180.0;
}  // Marine2Cart

// convert Cartesian coordinates to Polar coordinates
std::tuple<double, double> Cartesian2Polar(const double x, const double y) {
  return {std::sqrt(x * x + y * y), std::atan2(y, x)};
}  // Cartesian2Polar

}  // namespace ASV::common::math

#endif /* _MATH_UTILS_H_ */