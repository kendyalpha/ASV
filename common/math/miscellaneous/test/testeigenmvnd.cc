/*
***********************************************************************
* testeigenmvnd.cc: Multivariate Normal distribution sampling
* function to simulate the multivariate normal distribution using C++11
* and Eigen matrices.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#include <fstream>
#include <iostream>
#include "../include/eigenmvnd.hpp"
#include "common/plotting/include/gnuplot-iostream.h"

/**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d genCovar(double v0, double v1, double theta) {
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();
  return rot * Eigen::DiagonalMatrix<double, 2, 2>(v0, v1) * rot.transpose();
}

int main() {
  using namespace ASV::common;

  Eigen::Vector2d mean;
  Eigen::Matrix2d covar;
  mean << -1, 0.5;  // Set the mean
  // Create a covariance matrix
  // Much wider than it is tall
  // and rotated clockwise by a bit
  covar = genCovar(3, 0.1, M_PI / 5.0);

  // Create a bivariate gaussian distribution of doubles.
  // with our chosen mean and covariance
  math::eigenmvnd normX_solver(mean, covar, 5000);

  Eigen::MatrixXd sample_normal_Q =
      normX_solver.perform_mvnd().get_mvnd_matrix();

  // plotting
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 1000 0\n";
  gp << "set title 'normal distribution'\n";
  gp << "set size ratio -1\n";
  std::vector<std::pair<double, double>> xy_pts_A;

  for (int i = 0; i != sample_normal_Q.cols(); ++i) {
    xy_pts_A.push_back(
        std::make_pair(sample_normal_Q(0, i), sample_normal_Q(1, i)));
  }
  gp << "plot '-' with points notitle\n";
  gp.send1d(xy_pts_A);
}