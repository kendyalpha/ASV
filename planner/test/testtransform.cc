/*
***********************************************************************
* testtransform.cc:
* Utility test for transformation from Cartesian x,y coordinates
* to Frenet s,d coordinates
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <cstdlib>
#include "timecounter.h"
#include "trajectorygenerator.h"
#include "utilityio.h"

int main() {
  // define the waypoints on the target course
  Eigen::VectorXd X(7);
  Eigen::VectorXd Y(7);
  X << -2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0;
  Y << 0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0;

  trajectorygenerator _trajectorygenerator(X, Y);

  X.resize(5);
  Y.resize(5);
  X << 00.0, 20.0, 30.0, 50.0;
  Y << 00.0, 00.5, 04.5, 07.0;

  _trajectorygenerator.regenerate_target_course(X, Y);

  // define the position of vessel
  Eigen::VectorXd faObjects_x(5);
  Eigen::VectorXd faObjects_y(5);
  faObjects_x << 10.0, 15.0, 20.0, 30.0, 45.0;
  faObjects_y << -0.5, 2.0, 0.0, 3.0, 3.5;

  FrenetState _rtFS{
      0,  // s
      0,  // ds_t
      0,  // dds_t
      0,  // d
      0,  // dd_t
      0,  // ddd_t
      0,  // dd_s
      0   // ddd_s
  };
  CartesianState _rtCS{
      0,  // x
      0,  // y
      0,  // theta
      0,  // kappa
      0,  // speed
      0   // dspeed
  };

  for (int i = 0; i != faObjects_x.size(); ++i) {
    _rtCS.x = faObjects_x(i);
    _rtCS.y = faObjects_y(i);
    transformf2c(_trajectorygenerator, _rtFS, _rtCS);
  }

  auto refx = _trajectorygenerator.getCartRefX();
  auto refy = _trajectorygenerator.getCartRefY();

  utilityio _utilityio;
  _utilityio.write2csvfile("../data/refx.csv", refx);
  _utilityio.write2csvfile("../data/refy.csv", refy);
  // _utilityio.write2csvfile(
  //     "../data/s.csv",
  //     _utilityio.convertstdvector2EigenMat(arclength, arclength.size(), 1));
}