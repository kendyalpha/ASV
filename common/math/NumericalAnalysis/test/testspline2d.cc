/*
***********************************************************************
* testspline2d.cc:
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
  Eigen::VectorXd X(7);
  Eigen::VectorXd Y(7);
  X << -2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0;
  Y << 0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0;

  std::vector<double> arclength;
  std::vector<double> rx, ry;
  std::vector<double> ryaw;
  std::vector<double> rk;
  std::vector<double> rdk;

  Spline2D _Spline2D(X, Y);

  X.resize(5);
  Y.resize(5);
  X << 0.0, 10.0, 20.5, 35.0, 70.5;
  Y << 0.0, -6.0, 5.0, 6.5, 0.0;
  // X = Eigen::VectorXd::LinSpaced(5, 0, 5);
  // Y = Eigen::VectorXd::LinSpaced(5, 0, 0);

  _Spline2D.reinterpolation(X, Y);
  Eigen::VectorXd s = _Spline2D.getarclength();

  double step = 0.05;
  int n = 1 + static_cast<int>(s[s.size() - 1] / step);
  arclength.resize(n);
  rx.resize(n);
  ry.resize(n);
  ryaw.resize(n);
  rk.resize(n);
  rdk.resize(n);

  double is = 0.0;
  for (int i = 0; i != n; i++) {
    is = step * i;
    Eigen::Vector2d position = _Spline2D.compute_position(is);
    arclength[i] = is;
    rx[i] = position(0);
    ry[i] = position(1);
    rk[i] = _Spline2D.compute_curvature(is);
    ryaw[i] = _Spline2D.compute_yaw(is);
    rdk[i] = _Spline2D.compute_dcurvature(is);
  }

  for (int i = 0; i != (n - 1); i++) {
    double ds = std::sqrt(std::pow(rx[i + 1] - rx[i], 2) +
                          std::pow(ry[i + 1] - ry[i], 2));
    double dtheta = ryaw[i + 1] - ryaw[i];
    std::cout << dtheta / ds << std::endl;
  }

  Gnuplot gp;

  std::vector<std::pair<double, double> > xy_pts_A;
  std::vector<std::pair<double, double> > xy_pts_B;

  gp << "set multiplot layout 2, 2 \n";
  gp << "set title 'Polynomial interpolation using spline' \n";
  gp << "plot"
        " '-' with points ps 2 lw 2 title 'waypoints',"
        " '-' with lines lw 2 title 'interpolation'\n";
  for (int i = 0; i != X.rows(); ++i) {
    xy_pts_A.push_back(std::make_pair(X(i), Y(i)));
  }

  for (std::size_t i = 0; i != rx.size(); ++i) {
    xy_pts_B.push_back(std::make_pair(rx[i], ry[i]));
  }
  gp.send1d(xy_pts_A);
  gp.send1d(xy_pts_B);
  // gp << "set xrange [0:20] \n";
  // gp << "set yrange [0:20] \n";

  gp << "set title 'cuvature' \n";
  gp << "set xrange [0:40] \n";

  gp << "plot"
        " '-' with lines lw 2\n";
  xy_pts_A.clear();
  for (std::size_t i = 0; i != arclength.size(); ++i) {
    xy_pts_A.push_back(std::make_pair(arclength[i], rk[i]));
  }
  gp.send1d(xy_pts_A);

  gp << "set title 'yaw' \n";
  gp << "set xrange [0:40] \n";
  gp << "plot"
        " '-' with lines lw 2\n";
  xy_pts_A.clear();
  for (std::size_t i = 0; i != arclength.size(); ++i) {
    xy_pts_A.push_back(std::make_pair(arclength[i], ryaw[i]));
  }
  gp.send1d(xy_pts_A);

  gp << "set title 'dk' \n";
  gp << "set xrange [0:40] \n";
  gp << "plot"
        " '-' with lines lw 2\n";
  xy_pts_A.clear();
  for (std::size_t i = 0; i != arclength.size(); ++i) {
    xy_pts_A.push_back(std::make_pair(arclength[i], rdk[i]));
  }
  gp.send1d(xy_pts_A);

  gp << "unset multiplot\n";

  return EXIT_SUCCESS;
}
