/*
*******************************************************************************
* testtcpclient.cc:
* unit test for socket client using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include "../include/tcpclient.h"
#define SAMPLES_PER_SPOKE 20

union lidarmsg {
  double double_msg[5];
  char char_msg[40];
};

union radarmsg {
  uint32_t header32[11];
  char header4[44];
};

void test() {
  tcpclient _tcpclient("127.0.0.1", "9340");
  const int recv_size = 45 + SAMPLES_PER_SPOKE / 2;
  const int send_size = 10;
  radarmsg _radarmsg;
  char send_buffer[send_size] = "socket";
  char recv_buffer[recv_size];
  uint8_t spokedata[SAMPLES_PER_SPOKE / 2];
  while (1) {
    _tcpclient.senddata(recv_buffer, send_buffer, recv_size, send_size);

    for (int i = 0; i != 44; ++i) _radarmsg.header4[i] = recv_buffer[i];
    for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i) {
      spokedata[i] = (uint8_t)recv_buffer[i + 44];
      printf("%u\n", spokedata[i]);
    }

    // memcpy(_radarmsg.header4, recv_buffer, 44);
    // memcpy(spokedata, &recv_buffer[44], SAMPLES_PER_SPOKE / 2);
    for (int i = 0; i != 11; ++i)
      printf("The buffer recived: %d\n", _radarmsg.header32[i]);

    printf("The socket status: %d\n", _tcpclient.getsocketresults());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main() { test(); }