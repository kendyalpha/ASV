
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

namespace ASV {

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
void Marine2Cart(double marine_y, double marine_theta, double marine_kappa,
                 double &cart_y, double &cart_theta,
                 double &cart_kappa) noexcept {
  cart_y = -marine_y;
  cart_theta = -marine_theta;
  cart_kappa = -marine_kappa;

}  // Marine2Cart
// convert marine coordinate to cartesian coordinate
Eigen::VectorXd Marine2Cart(const Eigen::VectorXd &_marine_y) noexcept {
  return (-1) * _marine_y;
}  // Marine2Cart
// convert marine coordinate to cartesian coordinate
void Cart2Marine(double cart_y, double cart_theta, double cart_kappa,
                 double &marine_y, double &marine_theta,
                 double &marine_kappa) noexcept {
  marine_y = -cart_y;
  marine_theta = -cart_theta;
  marine_kappa = -cart_kappa;

}  // Marine2Cart

// convert rad to degree
double Rad2Degree(double _rad) noexcept {
  return 180.0 * _rad / M_PI;
}  // Rad2Degree

// convert marine coordinate to cartesian coordinate
double Degree2Rad(double _degree) noexcept {
  return M_PI * _degree / 180.0;
}  // Marine2Cart

}  // end namespace ASV

#endif /* _MATH_UTILS_H_ */