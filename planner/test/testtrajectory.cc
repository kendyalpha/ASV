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
  Eigen::VectorXd X(7);
  Eigen::VectorXd Y(7);
  X << -2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0;
  Y << 0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0;

  trajectorygenerator _trajectorygenerator(X, Y);

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
