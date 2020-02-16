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
#include "../include/spline.h"
#include "common/plotting/include/gnuplot-iostream.h"

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

  Gnuplot gp;

  std::vector<std::pair<double, double> > xy_pts_A;
  for (int i = 0; i != X.rows(); ++i) {
    xy_pts_A.push_back(std::make_pair(X(i), Y(i)));
  }

  std::vector<std::pair<double, double> > xy_pts_B;
  for (std::size_t i = 0; i != x.size(); ++i) {
    xy_pts_B.push_back(std::make_pair(x[i], y[i]));
  }

  // gp << "set xrange [-2:2]\nset yrange [-2:2]\n";

  gp << "set title 'spline interpolation'\n";
  gp << "set xlabel 'X'\n";
  gp << "set ylabel 'Y'\n";

  gp << "plot"
        " '-' with points ps 2 lw 2 title 'waypoints',"
        " '-' with lines lw 2 title 'interpolation'\n";
  gp.send1d(xy_pts_A);
  gp.send1d(xy_pts_B);

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
