/*
***********************************************************************
* testtrajectory.cc:
* Utility test for Frenet optimal trajectory generator
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include <cstdlib>
#include "trajectorygenerator.h"
#include "utilityio.h"

int main() {
  Eigen::VectorXd X(5);
  Eigen::VectorXd Y(5);
  Eigen::VectorXd ob_x(5);
  Eigen::VectorXd ob_y(5);
  X << 0.0, 10.0, 20.5, 35.0, 70.5;
  Y << 0.0, -6.0, 5.0, 6.5, 0.0;
  ob_x << 20.0, 30.0, 30.0, 35.0, 50.0;
  ob_y << 10.0, 6.0, 8.0, 8.0, 3.0;
  trajectorygenerator _trajectorygenerator(X, Y);
  _trajectorygenerator.setobstacle(ob_x, ob_y);
  _trajectorygenerator.test();

  // utilityio _utilityio;
  // _utilityio.write2csvfile("../data/x.csv", X);
  // _utilityio.write2csvfile("../data/y.csv", Y);
  // _utilityio.write2csvfile("../data/rx.csv", _trajectorygenerator.getrx());
  // _utilityio.write2csvfile("../data/ry.csv", _trajectorygenerator.getry());
  // _utilityio.write2csvfile("../data/yaw.csv",
  // _trajectorygenerator.getryaw()); _utilityio.write2csvfile("../data/k.csv",
  // _trajectorygenerator.getrk());
}
