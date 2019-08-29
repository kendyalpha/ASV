/*
*******************************************************************************
* testlos.cc:
* unit test for path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "lineofsight.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  double L = 3.6;
  Eigen::Vector2d wp1 = Eigen::Vector2d::Zero();
  Eigen::Vector2d wp2 = Eigen::Vector2d::Zero();
  Eigen::Vector2d wp3 = Eigen::Vector2d::Zero();
  wp1 << 3433875, 351046;
  wp2 << 3433895, 351058;
  wp3 << 3433890, 351050;
  lineofsight _lineofsight(L, 0);

  Eigen::Vector2d vp = Eigen::Vector2d::Zero();
  vp << 3433894.584, 351058.1;
  std::cout << _lineofsight.computelospoint(vp, wp1, wp2).getdesired_theta()
            << std::endl;
  std::cout << _lineofsight.computelospoint(vp, wp2, wp3).getdesired_theta()
            << std::endl;
}