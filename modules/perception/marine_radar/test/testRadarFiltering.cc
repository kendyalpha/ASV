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
#include "common/plotting/include/matplotlibcpp.h"

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
                                   0.02 * Eigen::MatrixXd::Identity(2, 2),
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
  // Set the size of output image = 1200x780 pixels
  matplotlibcpp::figure_size(1600, 700);
  matplotlibcpp::subplot(1, 2, 1);
  matplotlibcpp::plot(true_x, true_y, "r-");
  matplotlibcpp::plot(meas_x, meas_y, ".");
  matplotlibcpp::plot(est_x, est_y, "g--");

  matplotlibcpp::axis("equal");

  matplotlibcpp::subplot(1, 2, 2);
  matplotlibcpp::plot(true_vx, "r-");
  matplotlibcpp::plot(est_vx, "r--");
  matplotlibcpp::plot(true_vy, "g-");
  matplotlibcpp::plot(est_vy, "g--");

  matplotlibcpp::title("Alpha-Beta Filtering");

  matplotlibcpp::show();
}