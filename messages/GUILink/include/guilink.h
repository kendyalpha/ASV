/*
***********************************************************************
* guiserver.h: communication for gui server
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _GUILINK_H_
#define _GUILINK_H_

#include <boost/lexical_cast.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "crc.h"
#include "easylogging++.h"
#include "guilinkdata.h"
#include "serial/serial.h"
#include "timecounter.h"

namespace ASV {

template <int m, int n = 3>
class guilink_serial {
 public:
  // constructor using serial
  guilink_serial(const guilinkRTdata<m> &_guilinkRTdata,
                 unsigned long _rate = 115200,
                 const std::string &_port = "/dev/ttyUSB0")
      : guilinkrtdata(_guilinkRTdata),
        gui_serial(_port, _rate, serial::Timeout::simpleTimeout(100)),
        send_buffer(""),
        recv_buffer(""),
        bytes_send(0),
        bytes_reci(0),
        gui_connetion_failure_count(21),
        crc16(CRC16::eCCITT_FALSE) {
    checkserialstatus();
  }

  virtual ~guilink_serial() = default;

  guilink_serial &guicommunication() {
    checkconnection(guilinkrtdata);
    senddata2gui(guilinkrtdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    parsedatafromgui(_guilinkRTdata);
    return *this;
  }  // guicommunication

 private:
  guilinkRTdata<m> guilinkrtdata;
  serial::Serial gui_serial;
  std::string send_buffer;
  std::string recv_buffer;
  std::size_t bytes_send;
  std::size_t bytes_reci;

  int gui_connetion_failure_count;

  CRC16 crc16;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
      std::cout << device.port.c_str() << ", " << device.description.c_str()
                << ", " << device.hardware_id.c_str() << std::endl;
    }
  }

  void checkserialstatus() {
    if (gui_serial.isOpen())
      CLOG(INFO, "gui-serial") << " serial port open successful!";
    else
      CLOG(INFO, "gui-serial") << " serial port open failure!";
  }  // checkserialstatus

  void checkconnection(guilinkRTdata<m> &_RTdata) {
    if (gui_connetion_failure_count > 20)
      _RTdata.linkstatus = LINKSTATUS::DISCONNECTED;
    else
      _RTdata.linkstatus = LINKSTATUS::CONNECTED;
  }  // checkconnection

  // convert real time GPS data to sql string
  void convert2string(const guilinkRTdata<m> &_guilinkRTdata,
                      std::string &_str) {
    _str += ",";
    _str += _guilinkRTdata.UTC_time;
    _str += ",";
    _str += std::to_string(static_cast<int>(_guilinkRTdata.linkstatus));
    _str += ",";
    _str += to_string_with_precision<double>(_guilinkRTdata.latitude, 6);
    _str += ",";
    _str += to_string_with_precision<double>(_guilinkRTdata.longitude, 6);
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str += to_string_with_precision<double>(_guilinkRTdata.State(i), 3);
    }
    _str += ",";
    _str += to_string_with_precision<double>(_guilinkRTdata.roll, 3);
    _str += ",";
    _str += to_string_with_precision<double>(_guilinkRTdata.pitch, 3);

    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_guilinkRTdata.feedback_rotation(i));
    }
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_guilinkRTdata.feedback_alpha(i));
    }
  }  // convert2string

  void parsedatafromgui(guilinkRTdata &_guilinkRTdata) {
    recv_buffer = gui_serial.readline(300, "\n");

    std::size_t pos = recv_buffer.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      recv_buffer = recv_buffer.substr(pos + 1);
      std::size_t rpos = recv_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected crc checksum value
        std::string expected_crc = recv_buffer.substr(rpos + 1);
        expected_crc.pop_back();
        recv_buffer = recv_buffer.substr(0, rpos);
        if (std::to_string(crc16.crcCompute(recv_buffer.c_str(), rpos)) ==
            expected_crc) {
          gui_connetion_failure_count = 0;  // reset to zero

          double _heading = 0.0;
          double wp1_x = 0.0;
          double wp1_y = 0.0;
          double wp2_x = 0.0;
          double wp2_y = 0.0;
          double wp3_x = 0.0;
          double wp3_y = 0.0;
          double wp4_x = 0.0;
          double wp4_y = 0.0;
          double wp5_x = 0.0;
          double wp5_y = 0.0;
          double wp6_x = 0.0;
          double wp6_y = 0.0;
          double wp7_x = 0.0;
          double wp7_y = 0.0;
          double wp8_x = 0.0;
          double wp8_y = 0.0;
          sscanf(recv_buffer.c_str(),
                 "CORE,"
                 "%d,"       // indicator_autocontrolmode
                 "%d,"       // indicator_windstatus
                 "%lf,"      // desired_speed
                 "%lf,"      // heading
                 ",%lf,%lf"  // waypoint1
                 ",%lf,%lf"  // waypoint2
                 ",%lf,%lf"  // waypoint3
                 ",%lf,%lf"  // waypoint4
                 ",%lf,%lf"  // waypoint5
                 ",%lf,%lf"  // waypoint6
                 ",%lf,%lf"  // waypoint7
                 ",%lf,%lf"  // waypoint8
                 ,
                 &(_guilinkRTdata.indicator_autocontrolmode),  // int
                 &(_guilinkRTdata.indicator_windstatus),       // int
                 &(_guilinkRTdata.desired_speed),              // double
                 &_heading,                                    // double
                 &wp1_x, &wp1_y,  // waypoint1_x, waypoint1_y
                 &wp2_x, &wp2_y,  // waypoint2_x, waypoint2_y
                 &wp3_x, &wp3_y,  // waypoint3_x, waypoint3_y
                 &wp4_x, &wp4_y,  // waypoint4_x, waypoint4_y
                 &wp5_x, &wp5_y,  // waypoint5_x, waypoint5_y
                 &wp6_x, &wp6_y,  // waypoint6_x, waypoint6_y
                 &wp7_x, &wp7_y,  // waypoint7_x, waypoint7_y
                 &wp8_x, &wp8_y   // waypoint8_x, waypoint8_y
          );

          _guilinkRTdata.setpoints(0) = wp1_x;
          _guilinkRTdata.setpoints(1) = wp1_y;
          _guilinkRTdata.setpoints(2) = Degree2Rad(_heading);
          _guilinkRTdata.waypoints(0, 0) = wp1_x;
          _guilinkRTdata.waypoints(1, 0) = wp1_y;
          _guilinkRTdata.waypoints(0, 1) = wp2_x;
          _guilinkRTdata.waypoints(1, 1) = wp2_y;
          _guilinkRTdata.waypoints(0, 2) = wp3_x;
          _guilinkRTdata.waypoints(1, 2) = wp3_y;
          _guilinkRTdata.waypoints(0, 3) = wp4_x;
          _guilinkRTdata.waypoints(1, 3) = wp4_y;
          _guilinkRTdata.waypoints(0, 4) = wp5_x;
          _guilinkRTdata.waypoints(1, 4) = wp5_y;
          _guilinkRTdata.waypoints(0, 5) = wp6_x;
          _guilinkRTdata.waypoints(1, 5) = wp6_y;
          _guilinkRTdata.waypoints(0, 6) = wp7_x;
          _guilinkRTdata.waypoints(1, 6) = wp7_y;
          _guilinkRTdata.waypoints(0, 7) = wp8_x;
          _guilinkRTdata.waypoints(1, 7) = wp8_y;
        } else {
          connectionstatus++;
          CLOG(INFO, "gui-link") << " checksum error!";
        }
      }
    }
  }  // parsedatafromgui

  void senddata2gui(const guilinkRTdata<m> &_guilinkRTdata;) {
    send_buffer.clear();
    send_buffer = "GUI";
    convert2string(_guilinkRTdata, send_buffer);
    unsigned short crc =
        crc16.crcCompute(send_buffer.c_str(), send_buffer.length());
    send_buffer = "$" + send_buffer + "*" + std::to_string(crc) + "\n";
    bytes_send = gui_serial.write(send_buffer);
  }  // senddata2gui

 public:
  template <int _m, int _n>
  friend std::ostream &operator<<(std::ostream &, const guilink<_m, _n> &);
};

template <int _m, int _n>
std::ostream &operator<<(std::ostream &os,
                         const guiserver<_m, _n> &_guiserver) {
  os << "Buffer sent is: " << _guiserver.send_buffer;
  os << "length of Buffer sent is: " << _guiserver.bytes_send;
  os << "Buffer recv is: " << _guiserver.recv_buffer << std::endl;
  return os;
}

}  // end namespace ASV

#endif /* _GUILINK_H_ */