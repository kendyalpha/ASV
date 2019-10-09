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

using namespace ASV;

// real time wind sensor
windRTdata _windRTdata{
    0,  // speed
    0   // orientation
};

void readloop() {
  try {
    wind _wind(9600);  // zone 30n

    while (1) {
      _windRTdata = _wind.readwind().getwindRTdata();
      std::cout << "wind speed: " << _windRTdata.speed << std::endl;
      std::cout << "wind orientation: " << _windRTdata.orientation << std::endl;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

int main() {
  std::thread _thread(readloop);
  _thread.detach();
  while (1) {
    std::cout << "OK" << _windRTdata.speed << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
