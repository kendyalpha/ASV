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
#include "../include/FrenetTrajectoryGenerator.h"
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
  Eigen::VectorXd ob_x(5);
  Eigen::VectorXd ob_y(5);
  marine_X << 0.0, 10.0, 20.5, 35.0, 70.5;
  marine_Y << 0.0, 6.0, -5.0, -6.5, 0.0;
  ob_x << 20.0, 30.0, 30.0, 35.0, 50.0;
  ob_y << 10.0, 6.0, 8.0, 8.0, 3.0;

  Frenetdata _frenetdata{
      0.1,         // SAMPLE_TIME
      50.0 / 3.6,  // MAX_SPEED
      4.0,         // MAX_ACCEL
      -3.0,        // MIN_ACCEL
      1.0,         // MAX_CURVATURE
      0.05,        // TARGET_COURSE_ARC_STEP
      7.0,         // MAX_ROAD_WIDTH
      1.0,         // ROAD_WIDTH_STEP
      5.0,         // MAXT
      4.0,         // MINT
      0.2,         // DT
      1.2,         // MAX_SPEED_DEVIATION
      0.3,         // TRAGET_SPEED_STEP
      3,           // HULL_LENGTH
      1,           // HULL_WIDTH
      2.0          // ROBOT_RADIUS
  };

  // real time data
  CartesianState Plan_cartesianstate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
  };

  CartesianState estimate_marinestate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      1,           // speed
      0,           // dspeed
  };

  FrenetTrajectoryGenerator _trajectorygenerator(_frenetdata);
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
            .trajectoryonestep(estimate_marinestate.x, estimate_marinestate.y,
                               estimate_marinestate.theta,
                               estimate_marinestate.kappa,
                               estimate_marinestate.speed,
                               estimate_marinestate.dspeed, 10 / 3.6)
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

    _sendmsg.double_msg[0] = estimate_marinestate.x;      // vessel x
    _sendmsg.double_msg[1] = estimate_marinestate.y;      // vessel y
    _sendmsg.double_msg[2] = estimate_marinestate.theta;  // vessel heading

    for (int j = 0; j != 5; j++) {
      _sendmsg.double_msg[2 * j + 3] = ob_x(j);   // obstacle x
      _sendmsg.double_msg[2 * j + 4] = -ob_y(j);  // obstacle y
    }

    _sendmsg.double_msg[13] = cart_bestX.size();  // the length of vector
    for (int j = 0; j != cart_bestX.size(); j++) {
      _sendmsg.double_msg[2 * j + 14] = cart_bestX(j);   // best X
      _sendmsg.double_msg[2 * j + 15] = -cart_bestY(j);  // best Y
    }

    // _sendmsg.double_msg[13 + 2 * _bestX.size()] =
    //     _rx.size();  // the length of vector
    // for (int j = 0; j != _rx.size(); j++) {
    //   _sendmsg.double_msg[2 * j + 14] = _bestX(j);  // best X
    //   _sendmsg.double_msg[2 * j + 15] = _bestY(j);  // best Y
    // }

    _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                            send_size);

    if ((std::pow(estimate_marinestate.x - cart_rx(cart_rx.size() - 1), 2) +
         std::pow(estimate_marinestate.y + cart_ry(cart_ry.size() - 1), 2)) <=
        1.0) {
      std::cout << "goal\n";
      break;
    }

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
