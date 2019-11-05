/*
***********************************************************************
* testFrenetTrajectoryGenerator.cc:
* Utility test for Frenet optimal trajectory generator
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include <cstdlib>
#include "../include/LatticePlanner.h"
#include "common/communication/include/tcpserver.h"
#include "common/fileIO/include/utilityio.h"
#include "common/timer/include/timecounter.h"

using namespace ASV;

union trajectorymsg {
  double double_msg[100];
  char char_msg[800];
};

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // trajectory generator
  Eigen::VectorXd marine_X(5);
  Eigen::VectorXd marine_Y(5);
  Eigen::VectorXd ob_x(6);
  Eigen::VectorXd ob_y(6);
  marine_X << 0.0, 10.0, 20.5, 35.0, 70.5;
  marine_Y << 0.0, 6.0, -5.0, -6.5, 0.0;
  ob_x << 20.0, 30.0, 30.0, 35.0, 34.0, 50.0;
  ob_y << 10.0, 6.0, 8.0, 8.0, 8.0, 3.0;

  planning::LatticeData _latticedata{
      0.1,         // SAMPLE_TIME
      50.0 / 3.6,  // MAX_SPEED
      0.05,        // TARGET_COURSE_ARC_STEP
      7.0,         // MAX_ROAD_WIDTH
      1,           // ROAD_WIDTH_STEP
      7.0,         // MAXT
      6.0,         // MINT
      0.5,         // DT
      0.5,         // MAX_SPEED_DEVIATION
      0.1          // TRAGET_SPEED_STEP
  };

  planning::CollisionData _collisiondata{
      50.0 / 3.6,  // MAX_SPEED
      4.0,         // MAX_ACCEL
      -3.0,        // MIN_ACCEL
      1.0,         // MAX_CURVATURE
      3,           // HULL_LENGTH
      1,           // HULL_WIDTH
      2.5          // ROBOT_RADIUS
  };

  // real time data
  planning::CartesianState Plan_cartesianstate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
  };

  planning::CartesianState estimate_marinestate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      1,           // speed
      0,           // dspeed
  };

  planning::LatticePlanner _trajectorygenerator(_latticedata, _collisiondata);
  _trajectorygenerator.regenerate_target_course(marine_X, marine_Y);
  _trajectorygenerator.setobstacle(ob_x, ob_y);

  // socket
  tcpserver _tcpserver("9340");
  const int recv_size = 10;
  const int send_size = 800;
  char recv_buffer[recv_size];
  trajectorymsg _sendmsg = {0.0, 0.0, 0.0, 0.0, 0.0};

  // timer
  common::timecounter _timer;

  for (int i = 0; i != 500; ++i) {
    Plan_cartesianstate =
        _trajectorygenerator
            .trajectoryonestep(
                estimate_marinestate.x, estimate_marinestate.y,
                estimate_marinestate.theta, estimate_marinestate.kappa,
                estimate_marinestate.speed, estimate_marinestate.dspeed, 2)
            .getnextcartesianstate();

    estimate_marinestate = Plan_cartesianstate;

    std::tie(estimate_marinestate.y, estimate_marinestate.theta,
             estimate_marinestate.kappa) =
        common::math::Cart2Marine(Plan_cartesianstate.y,
                                  Plan_cartesianstate.theta,
                                  Plan_cartesianstate.kappa);

    auto cart_rx = _trajectorygenerator.getCartRefX();
    auto cart_ry = _trajectorygenerator.getCartRefY();
    auto cart_bestX = _trajectorygenerator.getbestX();
    auto cart_bestY = _trajectorygenerator.getbestY();
    auto cart_bestspeed = _trajectorygenerator.getbestSpeed();

    _sendmsg.double_msg[0] = estimate_marinestate.x;      // vessel x
    _sendmsg.double_msg[1] = estimate_marinestate.y;      // vessel y
    _sendmsg.double_msg[2] = estimate_marinestate.theta;  // vessel heading
    _sendmsg.double_msg[3] = estimate_marinestate.speed;  // vessel speed

    _sendmsg.double_msg[4] = ob_x.size();  // the length of vector
    for (int j = 0; j != ob_x.size(); j++) {
      _sendmsg.double_msg[2 * j + 5] = ob_x(j);   // obstacle x
      _sendmsg.double_msg[2 * j + 6] = -ob_y(j);  // obstacle y
    }

    int index = 2 * ob_x.size() + 5;
    _sendmsg.double_msg[index] = cart_bestX.size();  // the length of vector
    for (int j = 0; j != cart_bestX.size(); j++) {
      _sendmsg.double_msg[3 * j + index + 1] = cart_bestX(j);      // best X
      _sendmsg.double_msg[3 * j + index + 2] = -cart_bestY(j);     // best Y
      _sendmsg.double_msg[3 * j + index + 3] = cart_bestspeed(j);  // best speed
    }

    // _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
    //                         send_size);

    // if ((std::pow(estimate_marinestate.x - cart_rx(cart_rx.size() - 1), 2) +
    //      std::pow(estimate_marinestate.y + cart_ry(cart_ry.size() - 1), 2))
    //      <=
    //     1.0) {
    //   std::cout << "goal\n";
    //   break;
    // }

    long int et = _timer.timeelapsed();
    std::cout << et << std::endl;
  }

  // utilityio _utilityio;
  // _utilityio.write2csvfile("../data/x.csv", X);
  // _utilityio.write2csvfile("../data/y.csv", Y);
  // _utilityio.write2csvfile("../data/rx.csv", _trajectorygenerator.getrx());
  // _utilityio.write2csvfile("../data/ry.csv", _trajectorygenerator.getry());
  // _utilityio.write2csvfile("../data/yaw.csv",
  // _trajectorygenerator.getryaw()); _utilityio.write2csvfile("../data/k.csv",
  // _trajectorygenerator.getrk());
}
