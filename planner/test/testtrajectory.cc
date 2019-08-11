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
#include "tcpserver.h"
#include "trajectorygenerator.h"
#include "utilityio.h"

union trajectorymsg {
  double double_msg[13];
  char char_msg[104];
};

int main() {
  // trajectory generator
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

  // socket
  tcpserver _tcpserver("9340");
  const int recv_size = 10;
  const int send_size = 104;
  char recv_buffer[recv_size];
  trajectorymsg _sendmsg = {0.0, 0.0, 0.0, 0.0, 0.0};

  for (int i = 0; i != 500; ++i) {
    _trajectorygenerator.trajectoryonestep();

    auto _rx = _trajectorygenerator.getrx();
    auto _ry = _trajectorygenerator.getry();
    auto _cx = _trajectorygenerator.getcx();
    auto _cy = _trajectorygenerator.getcy();
    auto _cyaw = _trajectorygenerator.getcyaw();

    _sendmsg.double_msg[0] = _cx;    // vessel x
    _sendmsg.double_msg[1] = _cy;    // vessel y
    _sendmsg.double_msg[2] = _cyaw;  // vessel heading

    for (int j = 0; j != 5; j++) {
      _sendmsg.double_msg[j + 3] = ob_x(j);  // obstacle x
      _sendmsg.double_msg[j + 4] = ob_y(j);  // obstacle y
    }

    _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                            send_size);

    if ((std::pow(_cx - _rx(_rx.size() - 1), 2) +
         std::pow(_cy - _ry(_ry.size() - 1), 2)) <= 1.0) {
      std::cout << "goal\n";
      break;
    }
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
