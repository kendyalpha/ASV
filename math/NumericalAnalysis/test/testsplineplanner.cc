/*
***********************************************************************
* testsplineplanner.cc:
* Utility test for cubic spline library
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include <cstdlib>
#include "spline.h"
#include "utilityio.h"

int main() {
  Eigen::VectorXd X(7);
  Eigen::VectorXd Y(7);
  X << -2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0;
  Y << 0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0;
  // std::vector<double> X = {2.0, 1.732, 1.0, 0.0};
  // std::vector<double> Y = {0.0, 1.0, 1.732, 2.0};
  std::vector<double> rx, ry;
  std::vector<double> ryaw;
  std::vector<double> rk;

  Spline2D _Spline2D(X, Y);
  Eigen::VectorXd s = _Spline2D.getarclength();

  double step = 0.05;
  int n = 1 + static_cast<int>(s[s.size() - 1] / step);
  rx.resize(n);
  ry.resize(n);
  ryaw.resize(n);
  rk.resize(n);

  double is = 0.0;
  for (int i = 0; i != n; i++) {
    is = step * i;
    Eigen::Vector2d position = _Spline2D.compute_position(is);
    rx[i] = position(0);
    ry[i] = position(1);
    rk[i] = _Spline2D.compute_curvature(is);
    ryaw[i] = _Spline2D.compute_yaw(is);
  }

  utilityio _utilityio;
  _utilityio.write2csvfile("../data/x.csv", X);
  _utilityio.write2csvfile("../data/y.csv", Y);
  _utilityio.write2csvfile(
      "../data/rx.csv", _utilityio.convertstdvector2EigenMat(rx, rx.size(), 1));
  _utilityio.write2csvfile(
      "../data/ry.csv", _utilityio.convertstdvector2EigenMat(ry, ry.size(), 1));
  _utilityio.write2csvfile(
      "../data/yaw.csv",
      _utilityio.convertstdvector2EigenMat(ryaw, ryaw.size(), 1));
  _utilityio.write2csvfile(
      "../data/k.csv", _utilityio.convertstdvector2EigenMat(rk, rk.size(), 1));
  return EXIT_SUCCESS;
}
