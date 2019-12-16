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
#include "../include/tcpserver.h"

#define SAMPLES_PER_SPOKE 20

union lidarmsg {
  double double_msg[5];
  char char_msg[40];
};

union radarmsg {
  uint32_t header32[11];
  char header4[44];
};

void testtcpserver() {
  ASV::tcpserver _tcpserver("9340");

  const int recv_size = 10;
  const int send_size = 44 + SAMPLES_PER_SPOKE / 2;

  static int count = 0;
  char recv_buffer[recv_size];
  radarmsg _radarmsg = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  uint8_t spokedata[SAMPLES_PER_SPOKE / 2] = {0xff, 0xff, 0x09, 0xff, 0xff,
                                              0xfa, 0xff, 0xac, 0x10, 0x16};
  char sendmsg[send_size];
  while (1) {
    if (++count > 100) count = 0;
    _radarmsg.header32[0] = 1;          // vessel x
    _radarmsg.header32[1] = count - 2;  // vessel y
    _radarmsg.header32[2] = count;      // vessel heading
    _radarmsg.header32[3] = count;      // obstacle x -1
    _radarmsg.header32[4] = count + 1;  // obstacle y -1

    for (int i = 0; i != 44; ++i) sendmsg[i] = _radarmsg.header4[i];
    for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i)
      sendmsg[i + 44] = (char)spokedata[i];

    // strcpy(sendmsg, _radarmsg.header4);
    // strcat(sendmsg, (char*)spokedata);

    _tcpserver.selectserver(recv_buffer, sendmsg, recv_size, send_size);
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