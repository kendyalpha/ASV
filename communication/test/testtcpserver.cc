/*
*******************************************************************************
* testtcpserver.cc:
* unit test for socket server using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <cmath>
#include <iostream>
#include "tcpserver.h"

union lidarmsg {
  double double_msg[5];
  char char_msg[40];
};

void testtcpserver() {
  tcpserver _tcpserver("9340");
  const int recv_size = 10;
  const int send_size = 40;
  static int count = 0;
  char recv_buffer[recv_size];
  lidarmsg _sendmsg = {0.0, 0.0, 0.0, 0.0, 0.0};
  while (1) {
    if (++count > 100) count = 0;
    _sendmsg.double_msg[0] = 0.1;                // vessel x
    _sendmsg.double_msg[1] = 0.12 + count / 10;  // vessel y
    _sendmsg.double_msg[2] = count * M_PI / 8;   // vessel heading
    _sendmsg.double_msg[3] = 0;                  // obstacle x -1
    _sendmsg.double_msg[4] = 1;                  // obstacle y -1
    _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                            send_size);
    printf("The buffer recived: %s\n", recv_buffer);
    printf("The socket status: %d\n", _tcpserver.getsocketresults());
    printf("The clients connected: %d\n", _tcpserver.getconnectioncount());
  }
}
int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  testtcpserver();
  LOG(INFO) << "Shutting down.";
}