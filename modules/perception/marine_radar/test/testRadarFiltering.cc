/*
****************************************************************************
* testRadarFiltering.cc:
* unit test for Target tracking using marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <iostream>
#include "../include/RadarFiltering.h"
#include "common/math/miscellaneous/include/eigenmvnd.hpp"
#include "common/plotting/include/gnuplot-iostream.h"

using namespace ASV;

int main() {
  //
  const int totalnum = 100;
  const double sample_time = 60 / 24;

  // generate true value
  std::vector<double> true_vx(totalnum, 5);
  std::vector<double> true_vy(totalnum, 6);
  for (int i = 50; i != totalnum; ++i) true_vy[i] = 1;
  std::vector<double> true_x(totalnum);
  std::vector<double> true_y(totalnum);
  double true_x0 = 1;
  double true_y0 = 1;
  true_x[0] = true_x0;
  true_y[0] = true_y0;
  for (int i = 1; i != totalnum; ++i) {
    true_x[i] = sample_time * true_vx[i] + true_x[i - 1];
    true_y[i] = sample_time * true_vy[i] + true_y[i - 1];
  }

  // generate multivariate normal distribution to simulate gaussian noise
  common::math::eigenmvnd normal_Q(Eigen::VectorXd::Zero(2),
                                   0.05 * Eigen::MatrixXd::Identity(2, 2),
                                   totalnum);
  Eigen::MatrixXd sample_normal_Q = normal_Q.perform_mvnd().get_mvnd_matrix();

  std::vector<double> meas_x(totalnum);
  std::vector<double> meas_y(totalnum);
  for (int i = 0; i != totalnum; ++i) {
    meas_x[i] = sample_normal_Q(0, i) + true_x[i];
    meas_y[i] = sample_normal_Q(1, i) + true_y[i];
  }

  // filtering
  std::vector<double> est_x(totalnum);
  std::vector<double> est_y(totalnum);
  std::vector<double> est_vx(totalnum, 0);
  std::vector<double> est_vy(totalnum, 0);
  est_x[0] = true_x[0];
  est_y[0] = true_y[0];

  perception::RadarFiltering _RadarFiltering;
  for (int i = 1; i != totalnum; ++i) {
    std::tie(est_x[i], est_vx[i], est_y[i], est_vy[i]) =
        _RadarFiltering.NormalFilter(est_x[i - 1], est_vx[i - 1], meas_x[i],
                                     est_y[i - 1], est_vy[i - 1], meas_y[i],
                                     sample_time);
  }

  // plotting
  Gnuplot gp;

  std::vector<std::pair<double, double>> xy_pts_A;
  std::vector<std::pair<double, double>> xy_pts_B;
  std::vector<std::pair<double, double>> xy_pts_C;
  std::vector<std::pair<double, double>> xy_pts_D;

  for (std::size_t i = 0; i != true_x.size(); ++i)
    xy_pts_A.push_back(std::make_pair(true_x[i], true_y[i]));
  for (std::size_t i = 0; i != meas_x.size(); ++i)
    xy_pts_B.push_back(std::make_pair(meas_x[i], meas_y[i]));
  for (std::size_t i = 0; i != est_x.size(); ++i)
    xy_pts_C.push_back(std::make_pair(est_x[i], est_y[i]));
  // the first window
  gp << "set terminal x11 size 1500, 1000 0\n";
  gp << "set multiplot layout 2, 1 title 'Radar Filtering' font ',14'\n";
  gp << "set title 'Plot 1'\n";
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb 'red' title 'true',"
     << gp.file1d(xy_pts_B)
     << " with linespoints lt 2 lw 2 lc rgb 'black' title 'meas',"
     << gp.file1d(xy_pts_C)
     << " with linespoints pt 2 lw 1 lc rgb 'blue' title 'esti'\n";

  // second window
  xy_pts_A.clear();
  xy_pts_B.clear();
  xy_pts_C.clear();
  xy_pts_D.clear();

  for (std::size_t i = 0; i != true_vx.size(); ++i)
    xy_pts_A.push_back(std::make_pair(i, true_vx[i]));
  for (std::size_t i = 0; i != est_vx.size(); ++i)
    xy_pts_B.push_back(std::make_pair(i, est_vx[i]));
  for (std::size_t i = 0; i != true_vy.size(); ++i)
    xy_pts_C.push_back(std::make_pair(i, true_vy[i]));
  for (std::size_t i = 0; i != est_vy.size(); ++i)
    xy_pts_D.push_back(std::make_pair(i, est_vy[i]));

  gp << "set title 'Plot 2'\n";
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb 'red' title 'true-vx',"
     << gp.file1d(xy_pts_B)
     << " with lines lt 2 lw 2 lc rgb 'red' title 'est-vx',"
     << gp.file1d(xy_pts_C)
     << " with lines lt 3 lw 2 lc rgb 'black' title 'true-vy',"
     << gp.file1d(xy_pts_D)
     << " with lines lt 4 lw 2 lc rgb 'black' title 'est-vy'\n";

  gp << "unset multiplot\n";
}