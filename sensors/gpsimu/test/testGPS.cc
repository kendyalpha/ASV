/*
*******************************************************************************
* testGPS.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <cstdio>
#include <thread>
#include "gps.h"
#include "timecounter.h"

using std::setprecision;
int main() {
  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,  // UTC
      0,  // latitude
      0,  // longitude
      0,  // heading
      0,  // pitch
      0,  // roll
      0,  // altitude
      0,  // Ve
      0,  // Vn
      0,  // status
      0,  // UTM_x
      0,  // UTM_y
      0,  // u
      0   // v
  };
  try {
    timecounter _timer;
    gpsimu _gpsimu(51, true, 115200);  // zone 51 N
    long int totaltime = 0;
    while (1) {
      _gpsimu.gpsonestep(gps_data);
      std::string gps_buffer = _gpsimu.getserialbuffer();
      long int et = _timer.timeelapsed();
      totaltime += et;
      std::cout << "[" << totaltime << "]" << gps_buffer;

      {
        std::cout << "UTC:      " << gps_data.UTC << std::endl;
        std::cout << "latitude:   " << std::fixed << std::setprecision(7)
                  << gps_data.latitude << std::endl;
        std::cout << "longitude: " << std::fixed << std::setprecision(7)
                  << gps_data.longitude << std::endl;
        std::cout << "heading:   " << std::fixed << std::setprecision(2)
                  << gps_data.heading << std::endl;
        std::cout << "pitch:     " << std::fixed << std::setprecision(2)
                  << gps_data.pitch << std::endl;
        std::cout << "roll:      " << std::fixed << std::setprecision(2)
                  << gps_data.roll << std::endl;
        std::cout << "UTM_x:     " << std::fixed << std::setprecision(7)
                  << gps_data.UTM_x << std::endl;
        std::cout << "UTM_y:     " << std::fixed << std::setprecision(7)
                  << gps_data.UTM_y << std::endl;
        std::cout << "altitude:  " << std::fixed << std::setprecision(2)
                  << gps_data.altitude << std::endl;
        std::cout << "Ve:   " << std::fixed << std::setprecision(3)
                  << gps_data.Ve << std::endl;
        std::cout << "Vn:   " << std::fixed << std::setprecision(3)
                  << gps_data.Vn << std::endl;
        std::cout << "rot:   " << std::fixed << std::setprecision(2)
                  << gps_data.roti << std::endl;
        switch (gps_data.status) {
          case 0:
            std::cout << "GPS no fix" << std::endl;
            break;
          case 1:
            std::cout << "GPS fix" << std::endl;
            break;
          case 2:
            std::cout << "Diff GPX fix" << std::endl;
            break;
          default:
            std::cout << "Satus:     状态未知" << std::endl;
            break;
        }
        std::cout << std::endl;
      }
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

