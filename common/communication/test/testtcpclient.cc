/*
*******************************************************************************
* testtcpclient.cc:
* unit test for socket client using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "tcpclient.h"

union lidarmsg {
  double double_msg[5];
  char char_msg[40];
};

void test() {
  tcpclient _tcpclient("127.0.0.1", "9340");
  const int recv_size = 40;
  const int send_size = 10;
  lidarmsg _recvmsg;
  char send_buffer[send_size] = "socket";
  while (1) {
    _tcpclient.senddata(_recvmsg.char_msg, send_buffer, recv_size, send_size);
    for (int i = 0; i != 5; ++i)
      printf("The buffer recived: %lf\n", _recvmsg.double_msg[i]);
    printf("The socket status: %d\n", _tcpclient.getsocketresults());
  }
}

int main() { test(); }