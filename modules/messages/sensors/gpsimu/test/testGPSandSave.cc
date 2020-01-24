/*
*******************************************************************************
* testGPSandSave.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/gps.h"
#include "common/communication/include/tcpserver.h"
#include "common/fileIO/recorder/include/datarecorder.h"
#include "common/timer/include/timecounter.h"
using std::setprecision;
using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  // real time GPS/IMU data
  messages::gpsRTdata gps_data{
      0,    // UTC
      0,    // latitude
      0,    // longitude
      0,    // heading
      0,    // pitch
      0,    // roll
      0,    // altitude
      0,    // Ve
      0,    // Vn
      0,    // roti
      0,    // status
      0,    // UTM_x
      0,    // UTM_y
      "0n"  // UTM_zone
  };

  union socketmsg {
    double double_msg[2];
    char char_msg[16];
  };
  const int recv_size = 10;
  const int send_size = 16;
  char recv_buffer[recv_size];
  socketmsg _sendmsg = {0.0, 0.0};

  try {
    common::gps_db gps_db("./");
    gps_db.create_table();
    common::timecounter _timer;
    messages::GPS _gpsimu(115200, "/dev/ttyUSB0");  // zone 51 N
    tcpserver _tcpserver("9340");

    while (1) {
      gps_data = _gpsimu.parseGPS().getgpsRTdata();
      long int et = _timer.timeelapsed();

      gps_db.update_table(gps_data.UTC, gps_data.latitude, gps_data.longitude,
                          gps_data.heading, gps_data.pitch, gps_data.roll,
                          gps_data.altitude, gps_data.Ve, gps_data.Vn,
                          gps_data.roti, gps_data.status, gps_data.UTM_x,
                          gps_data.UTM_y, gps_data.UTM_zone);

      _sendmsg.double_msg[0] = gps_data.heading;
      _sendmsg.double_msg[1] =
          std::sqrt(gps_data.Ve * gps_data.Ve + gps_data.Vn * gps_data.Vn);

      _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                              send_size);

      std::cout << "UTC:      " << gps_data.UTC << std::endl;
      std::cout << "heading:   " << std::fixed << setprecision(2)
                << gps_data.heading << std::endl;
      std::cout << "pitch:     " << std::fixed << setprecision(2)
                << gps_data.pitch << std::endl;
      std::cout << "roll:      " << std::fixed << setprecision(2)
                << gps_data.roll << std::endl;
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
          std::cout << "Satus: Unknown" << std::endl;
          break;
      }
      std::cout << std::endl;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}