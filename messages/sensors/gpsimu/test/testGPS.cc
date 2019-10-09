/*
*******************************************************************************
* testGPS.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/gps.h"
#include "common/timer/include/timecounter.h"

using std::setprecision;
using namespace ASV;
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
      0,  // roti
      0,  // status
      0,  // UTM_x
      0   // UTM_y
  };
  try {
    timecounter _timer;
    GPS _gpsimu(gps_data, 51, true, 115200);  // zone 51 N
    long int totaltime = 0;
    int count = 0;
    while (1) {
      std::string gps_buffer = _gpsimu.gpsonestep().getserialbuffer();
      gps_data = _gpsimu.getgpsRTdata();
      long int et = _timer.timeelapsed();
      totaltime += et;
      std::cout << "[" << totaltime << "]" << gps_buffer;
      ++count;
      if (count == 4) {
        count = 0;
        std::cout << "UTC:      " << gps_data.UTC << std::endl;
        std::cout << "latitude:   " << std::fixed << setprecision(7)
                  << gps_data.latitude << std::endl;
        std::cout << "longitude: " << std::fixed << setprecision(7)
                  << gps_data.longitude << std::endl;
        std::cout << "heading:   " << std::fixed << setprecision(2)
                  << gps_data.heading << std::endl;
        std::cout << "pitch:     " << std::fixed << setprecision(2)
                  << gps_data.pitch << std::endl;
        std::cout << "roll:      " << std::fixed << setprecision(2)
                  << gps_data.roll << std::endl;
        std::cout << "UTM_x:     " << std::fixed << setprecision(7)
                  << gps_data.UTM_x << std::endl;
        std::cout << "UTM_y:     " << std::fixed << setprecision(7)
                  << gps_data.UTM_y << std::endl;
        std::cout << "altitude:  " << std::fixed << setprecision(2)
                  << gps_data.altitude << std::endl;
        std::cout << "Ve:   " << std::fixed << setprecision(3) << gps_data.Ve
                  << std::endl;
        std::cout << "Vn:   " << std::fixed << setprecision(3) << gps_data.Vn
                  << std::endl;
        std::cout << "rot:   " << std::fixed << setprecision(2) << gps_data.roti
                  << std::endl;
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

