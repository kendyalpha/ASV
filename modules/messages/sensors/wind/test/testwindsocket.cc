/*
*******************************************************************************
* testwind.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "../include/wind.h"
#include "common/communication/include/tcpserver.h"

using namespace ASV;

// real time wind sensor
windRTdata _windRTdata{
    0,  // speed
    0   // orientation
};

void readloop() {
  try {
    const int recv_size = 10;
    const int send_size = 16;
    char recv_buffer[recv_size];
    windsocketmsg send_msg = {0.0, 0.0};
    wind _wind(9600);
    tcpserver _tcpserver("9340");
    while (1) {
      _windRTdata = _wind.readwind().getwindRTdata();
      send_msg.double_msg[0] = _windRTdata.speed;
      send_msg.double_msg[1] = _windRTdata.orientation;

      _tcpserver.selectserver(recv_buffer, send_msg.char_msg, recv_size,
                              send_size);
      printf("The buffer recived: %s\n", recv_buffer);
      printf("The socket status: %d\n", _tcpserver.getsocketresults());
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

int main() { readloop(); }
