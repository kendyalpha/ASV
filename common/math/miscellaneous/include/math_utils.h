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

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

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
std::tuple<double, double> Marine2Cart(double marine_x,
                                       double marine_y) noexcept {
  return {marine_x, -marine_y};
}  // Marine2Cart
Eigen::VectorXd Marine2Cart(const Eigen::VectorXd &_marine_y) noexcept {
  return (-1) * _marine_y;
}  // Marine2Cart

// convert cartesian coordinate to marine coordinate
std::tuple<double, double, double> Cart2Marine(double cart_y, double cart_theta,
                                               double cart_kappa) noexcept {
  return {-cart_y, -cart_theta, -cart_kappa};
}  // Cart2Marine
std::tuple<double, double, double, double, double, double> Cart2Marine(
    double cart_x, double cart_y, double cart_theta, double cart_kappa,
    double cart_speed, double cart_dspeed) noexcept {
  return {cart_x, -cart_y, -cart_theta, -cart_kappa, cart_speed, cart_dspeed};
}  // Cart2Marine

// convert marine X and Y to UTM_N(y) and UTM_E(x)
std::tuple<double, double> Marine2UTM(double marine_x,
                                      double marine_y) noexcept {
  return {marine_y, marine_x};
}  // Marine2UTM

// convert  UTM_N(y) and UTM_E(x) to marine X and Y
std::tuple<double, double> UTM2Marine(double utm_x, double utm_y) noexcept {
  return {utm_y, utm_x};
}  // UTM2Marine

// convert rad to degree
double Rad2Degree(double _rad) noexcept {
  return 180.0 * _rad / M_PI;
}  // Rad2Degree

// convert marine coordinate to cartesian coordinate
double Degree2Rad(double _degree) noexcept {
  return M_PI * _degree / 180.0;
}  // Degree2Rad

// convert Cartesian coordinates to Polar coordinates
std::tuple<double, double> Cartesian2Polar(const double x, const double y) {
  return {std::sqrt(x * x + y * y), std::atan2(y, x)};
}  // Cartesian2Polar

// calculate the angle between two 2d vectors
double VectorAngle_2d(const double x1, const double y1,  // [x1, y1]
                      const double x2, const double y2   // [x2, y2]
) {
  double dot = x1 * x2 + y1 * y2;  // dot product between[x1, y1] and [ x2, y2 ]
  double det = x1 * y2 - y1 * x2;  // determinant
  return std::atan2(det, dot);     // atan2(y, x) or atan2(sin, cos)
}  // VectorAngle_2d

// returns -1 for negative numbers and +1 for positive numbers.
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}  // sgn

}  // namespace ASV::common::math

#endif /* _MATH_UTILS_H_ */