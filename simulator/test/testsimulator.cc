/*
***********************************************************************
* testsimulator.cc:
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/simulator.h"
#include "common/fileIO/include/utilityio.h"

using namespace ASV;

int main() {
  using state_type = Eigen::Matrix<double, 6, 1>;

  vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // Damping
      Eigen::Vector3d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      0,                        // L
      0                         // B
  };

  state_type x = (state_type() << 0, 1, 0.1, 0, 0, 0).finished();
  Eigen::Matrix3d P =
      (Eigen::Matrix3d() << 10, 0, 0, 0, 10, 0, 0, 0, 100).finished();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();

  int total_step = 5000;
  Eigen::MatrixXd save_x(total_step, 6);

  simulator _simulator(0.1, _vessel, x);

  for (int i = 0; i != total_step; ++i) {
    u = -P * x.head(3);
    x = _simulator.simulator_onestep(0, u).getX();
    save_x.row(i) = x.transpose();
  }
  ASV::common::write2csvfile("../data/x.csv", save_x);
}