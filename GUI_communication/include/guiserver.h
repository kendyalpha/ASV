/*
***********************************************************************
* guiserver.h: communication for gui server
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _GUISERVER_H_
#define _GUISERVER_H_

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

template <int m, int n = 3>
class guiserver {
  template <int _m, int _n>
  friend std::ostream &operator<<(std::ostream &, const guiserver<_m, _n> &);

 public:
  guiserver(unsigned long _rate = 115200,
            const std::string &_port = "/dev/ttyUSB0")
      : my_serial(_port, _rate, serial::Timeout::simpleTimeout(100)),
        send_buffer(""),
        recv_buffer(""),
        bytes_send(0),
        bytes_reci(0),
        gui_connetion_failure_count(21),
        _crc16(CRC16::eCCITT_FALSE) {
    judgeserialstatus();
  }
  ~guiserver() {}

  void guicommunication(guilinkRTdata &_guilinkRTdata,
                        const indicators &_indicators,
                        const estimatorRTdata &_estimatorRTdata,
                        const plannerRTdata &_plannerRTdata,
                        const gpsRTdata &_gpsRTdata,
                        const motorRTdata<m> &_motorRTdata,
                        const windRTdata &_windRTdata) {
    _guilinkRTdata.gui_connection = checkguiconnection();
    senddata2gui(_indicators, _estimatorRTdata, _plannerRTdata, _gpsRTdata,
                 _motorRTdata, _windRTdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    parsedatafromgui(_guilinkRTdata);
    //
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  int checkguiconnection() const {
    if (gui_connetion_failure_count > 20) return 0;
    return 1;
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;
  std::size_t bytes_send;
  std::size_t bytes_reci;

  int gui_connetion_failure_count;

  CRC16 _crc16;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
      std::cout << device.port.c_str() << ", " << device.description.c_str()
                << ", " << device.hardware_id.c_str() << std::endl;
    }
  }

  void judgeserialstatus() {
    if (my_serial.isOpen())
      CLOG(INFO, "gui-serial") << " serial port open successful!";
    else
      CLOG(INFO, "gui-serial") << " serial port open failure!";
  }

  // convert real time GPS data to sql string
  void convert2string(const indicators &_indicators, std::string &_str) {
    _str += ",";
    _str += std::to_string(_indicators.indicator_controlmode);
    _str += ",";
    _str += std::to_string(_indicators.indicator_windstatus);
  }
  // convert real time GPS data to sql string
  void convert2string(const gpsRTdata &_gpsRTdata, std::string &_str) {
    _str += ",";
    _str += std::to_string(_gpsRTdata.latitude);
    _str += ",";
    _str += std::to_string(_gpsRTdata.longitude);
  }

  // convert real time motor data to sql string
  void convert2string(const motorRTdata<m> &_motorRTdata, std::string &_str) {
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_alpha[i]);
    }
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_rotation[i]);
    }
    for (int i = 0; i != (2 * m); ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_torque[i]);
    }
    _str += ",";
    _str += std::to_string(_motorRTdata.feedback_allinfo);
  }
  void convert2string(const controllerRTdata<m, n> &_RTdata,
                      std::string &_str) {
    // the angle of each propeller (command)
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.alpha_deg(i));
    }
    // the speed of each propeller (command)
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.rotation(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // State
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.State(i));
    }
    // Roll and Pitch
    _str += ",";
    _str += std::to_string(_RTdata.motiondata_6dof(3));
    _str += ",";
    _str += std::to_string(_RTdata.motiondata_6dof(4));
  }

  void convert2string(const plannerRTdata &_RTdata, std::string &_str) {
    // setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.setpoint(i));
    }
  }

  void convert2string(const windRTdata &_RTdata, std::string &_str) {
    // wind
    _str += ",";
    _str += std::to_string(_RTdata.speed);
    _str += ",";
    _str += std::to_string(_RTdata.orientation);
  }

  void parsedatafromgui(guilinkRTdata &_guilinkRTdata) {
    recv_buffer = my_serial.readline(300, "\n");

    std::size_t pos = recv_buffer.find("$IPAC");
    if (pos != std::string::npos) {
      // crc check
      std::size_t rpos = recv_buffer.rfind(",");
      if (rpos != std::string::npos) {
        std::string crcstring = recv_buffer.substr(rpos + 1);
        crcstring.pop_back();
        unsigned short crc =
            _crc16.crcCompute(recv_buffer.substr(0, rpos).c_str(),
                              recv_buffer.substr(0, rpos).length());
        if (std::to_string(crc) == crcstring) {
          recv_buffer = recv_buffer.substr(pos);
          int _controlmode = 0;
          int _windindicator = 0;
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
                 "$IPAC,%d,%d,%lf"
                 ",%lf,%lf"  // waypoint1
                 ",%lf,%lf"  // waypoint2
                 ",%lf,%lf"  // waypoint3
                 ",%lf,%lf"  // waypoint4
                 ",%lf,%lf"  // waypoint5
                 ",%lf,%lf"  // waypoint6
                 ",%lf,%lf"  // waypoint7
                 ",%lf,%lf"  // waypoint8
                 ,
                 &_controlmode,    // indicator_controlmode
                 &_windindicator,  // indicator_windstatus
                 &_heading,        // heading
                 &wp1_x, &wp1_y,   // waypoint1_x, waypoint1_y
                 &wp2_x, &wp2_y,   // waypoint2_x, waypoint2_y
                 &wp3_x, &wp3_y,   // waypoint3_x, waypoint3_y
                 &wp4_x, &wp4_y,   // waypoint4_x, waypoint4_y
                 &wp5_x, &wp5_y,   // waypoint5_x, waypoint5_y
                 &wp6_x, &wp6_y,   // waypoint6_x, waypoint6_y
                 &wp7_x, &wp7_y,   // waypoint7_x, waypoint7_y
                 &wp8_x, &wp8_y    // waypoint8_x, waypoint8_y
          );
          _guilinkRTdata.indicator_autocontrolmode = _controlmode;  //
          _guilinkRTdata.indicator_windstatus = _windindicator;
          _guilinkRTdata.setpoints(0) = wp1_x;
          _guilinkRTdata.setpoints(1) = wp1_y;
          _guilinkRTdata.setpoints(2) = M_PI * _heading / 180;
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

          gui_connetion_failure_count = 0;
        }
      }

    } else {
      recv_buffer = "error";
      ++gui_connetion_failure_count;
    }
  }

  void senddata2gui(const indicators &_indicators,
                    const controllerRTdata<m, n> &_controllerRTdata,
                    const estimatorRTdata &_estimatorRTdata,
                    const plannerRTdata &_plannerRTdata,
                    const gpsRTdata &_gpsRTdata,
                    const motorRTdata<m> &_motorRTdata,
                    const windRTdata &_windRTdata) {
    send_buffer.clear();
    send_buffer = "$IPAC";
    convert2string(_indicators, send_buffer);
    convert2string(_gpsRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_controllerRTdata, send_buffer);
    convert2string(_motorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);
    convert2string(_windRTdata, send_buffer);
    send_buffer += "\n";
    bytes_send = my_serial.write(send_buffer);
  }

  void senddata2gui(const indicators &_indicators,
                    const estimatorRTdata &_estimatorRTdata,
                    const plannerRTdata &_plannerRTdata,
                    const gpsRTdata &_gpsRTdata,
                    const motorRTdata<m> &_motorRTdata,
                    const windRTdata &_windRTdata) {
    send_buffer.clear();
    send_buffer = "$IPAC";
    convert2string(_indicators, send_buffer);
    convert2string(_gpsRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_motorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);
    convert2string(_windRTdata, send_buffer);
    unsigned short crc =
        _crc16.crcCompute(send_buffer.c_str(), send_buffer.length());
    send_buffer += "," + std::to_string(crc) + "\n";
    bytes_send = my_serial.write(send_buffer);
  }
};

template <int _m, int _n>
std::ostream &operator<<(std::ostream &os,
                         const guiserver<_m, _n> &_guiserver) {
  os << "Buffer sent is: " << _guiserver.send_buffer;
  os << "length of Buffer sent is: " << _guiserver.bytes_send;
  os << "Buffer recv is: " << _guiserver.recv_buffer << std::endl;
  return os;
}

#endif /* _GUISERVER_H_ */