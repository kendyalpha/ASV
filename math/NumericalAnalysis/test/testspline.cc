/*
***********************************************************************
* testspline.cc:
* Utility test for cubic spline library
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include <cstdlib>
#include "spline.h"
#include "utilityio.h"
using namespace ASV;
int main() {
  Eigen::VectorXd X(5);
  Eigen::VectorXd Y(5);
  X << 0.4, 0.1, 2.0, 1.8, 1.2;
  Y << 0.7, 0.1, 0.9, 1.1, 0.6;

  std::vector<double> x(300), y(300);

  spline s;
  s.set_points(X, Y);
  for (int j = 0; j < 300; j++) {
    double _t = 0.01 * (j - 50);
    x[j] = _t;
    y[j] = s(_t);
  }

  utilityio _utilityio;
  _utilityio.write2csvfile("../data/x.csv", X);
  _utilityio.write2csvfile("../data/y.csv", Y);
  _utilityio.write2csvfile("../data/spline_x.csv",
                           _utilityio.convertstdvector2EigenMat(x, 300, 1));
  _utilityio.write2csvfile("../data/spline_y.csv",
                           _utilityio.convertstdvector2EigenMat(y, 300, 1));

  // Eigen::Matrix<double, 6, 1> a;
  // a << 1, 1, 1, 1, 1, 1;
  Eigen::Matrix<double, 5, 1> a;
  a << 7, 1, 23, 12, 4;
  polynomialvalue<4> _quintic_polynomial(a);
  std::cout << _quintic_polynomial.compute_order_derivative<0>(1) << std::endl;
  std::cout << _quintic_polynomial.compute_order_derivative<1>(2) << std::endl;
  std::cout << _quintic_polynomial.compute_order_derivative<2>(3) << std::endl;
  std::cout << _quintic_polynomial.compute_order_derivative<3>(4) << std::endl;

  return EXIT_SUCCESS;
}
