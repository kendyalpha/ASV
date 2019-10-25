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

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "common/communication/include/crc.h"
#include "common/fileIO/include/utilityio.h"
#include "common/logging/include/easylogging++.h"
#include "common/math/miscellaneous/include/math_utils.h"
#include "common/timer/include/timecounter.h"
#include "guilinkdata.h"
#include "third_party/serial/include/serial/serial.h"

namespace ASV::messages {

template <int m, int n = 3>
class guilink_serial {
 public:
  // constructor using serial
  guilink_serial(const guilinkRTdata<m> &_guilinkRTdata,
                 unsigned long _rate = 115200,
                 const std::string &_port = "/dev/ttyUSB0")
      : guilinkrtdata(_guilinkRTdata),
        gui_serial(_port, _rate, serial::Timeout::simpleTimeout(200)),
        send_buffer(""),
        recv_buffer(""),
        bytes_send(0),
        bytes_reci(0),
        gui_connetion_failure_count(0),
        crc16(CRC16::eCCITT_FALSE) {
    checkserialstatus();
  }

  virtual ~guilink_serial() = default;

  // communication with gui
  guilink_serial &guicommunication() {
    checkconnection(guilinkrtdata);
    senddata2gui(guilinkrtdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (parsedata_from_gui(guilinkrtdata)) {
      // parse successfully
      gui_connetion_failure_count =
          std::min(gui_connetion_failure_count + 1, 20);
    } else {
      // fail to parse
      gui_connetion_failure_count =
          std::max(gui_connetion_failure_count - 1, 0);
    }
    return *this;
  }  // guicommunication

  guilink_serial &setguilinkRTdata(const guilinkRTdata<m> &_guilinkRTdata) {
    guilinkrtdata = _guilinkRTdata;
    return *this;
  }  // setguilinkRTdata
  auto getguilinkRTdata() const noexcept { return guilinkrtdata; }
  std::string getrecv_buffer() const noexcept { return recv_buffer; }
  std::string getsend_buffer() const noexcept { return send_buffer; }

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
      _RTdata.linkstatus = common::LINKSTATUS::DISCONNECTED;
    else
      _RTdata.linkstatus = common::LINKSTATUS::CONNECTED;
  }  // checkconnection

  // convert real time gui data to sql string
  void convert2string(const guilinkRTdata<m> &_guilinkRTdata,
                      std::string &_str) {
    _str += ",";
    _str += _guilinkRTdata.UTC_time;
    _str += ",";
    _str += std::to_string(static_cast<int>(_guilinkRTdata.linkstatus));
    _str += ",";
    _str +=
        common::to_string_with_precision<double>(_guilinkRTdata.latitude, 6);
    _str += ",";
    _str +=
        common::to_string_with_precision<double>(_guilinkRTdata.longitude, 6);
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str +=
          common::to_string_with_precision<double>(_guilinkRTdata.State(i), 3);
    }
    _str += ",";
    _str += common::to_string_with_precision<double>(_guilinkRTdata.roll, 3);
    _str += ",";
    _str += common::to_string_with_precision<double>(_guilinkRTdata.pitch, 3);

    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_guilinkRTdata.feedback_rotation(i));
    }
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_guilinkRTdata.feedback_alpha(i));
    }
  }  // convert2string

  // convert real time gui data to sql string (lightweight)
  void convert2string_lightweight(const guilinkRTdata<m> &_guilinkRTdata,
                                  std::string &_str) {
    _str += ",";
    _str += _guilinkRTdata.UTC_time;
    _str += ",";
    _str += std::to_string(static_cast<int>(_guilinkRTdata.linkstatus));
    _str += ",";
    _str +=
        common::to_string_with_precision<double>(_guilinkRTdata.latitude, 6);
    _str += ",";
    _str +=
        common::to_string_with_precision<double>(_guilinkRTdata.longitude, 6);
    for (int i = 0; i != 3; ++i) {
      _str += ",";
      _str +=
          common::to_string_with_precision<double>(_guilinkRTdata.State(i), 3);
    }
    _str += ",";
    _str += common::to_string_with_precision<double>(_guilinkRTdata.roll, 3);
    _str += ",";
    _str += common::to_string_with_precision<double>(_guilinkRTdata.pitch, 3);

    for (int i = 0; i != m; ++i) {
      _str += ",";
      if (_guilinkRTdata.feedback_alpha(i) < M_PI / 4)
        _str += std::to_string(_guilinkRTdata.feedback_rotation(i));
      else
        _str += std::to_string(-_guilinkRTdata.feedback_rotation(i));
    }

  }  // convert2string_lightweight

  bool parsedata_from_gui(guilinkRTdata<m> &_guilinkRTdata) {
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
          double _heading = 0.0;
          double wp1_x = 0.0;
          double wp1_y = 0.0;
          double wp2_x = 0.0;
          double wp2_y = 0.0;
          sscanf(recv_buffer.c_str(),
                 "GUI,"
                 "%d,"       // indicator_autocontrolmode
                 "%lf,"      // heading
                 ",%lf,%lf"  // waypoint1
                 ",%lf,%lf"  // waypoint2
                 ,
                 &(_guilinkRTdata.indicator_autocontrolmode),  // int
                 &_heading,                                    // double
                 &wp1_x, &wp1_y,  // waypoint1_x, waypoint1_y
                 &wp2_x, &wp2_y   // waypoint2_x, waypoint2_y
          );

          _guilinkRTdata.setpoints(0) = wp1_x;
          _guilinkRTdata.setpoints(1) = wp1_y;
          _guilinkRTdata.setpoints(2) = common::math::Degree2Rad(_heading);

          _guilinkRTdata.startingpoint(0) = wp1_x;
          _guilinkRTdata.startingpoint(1) = wp1_y;
          _guilinkRTdata.endingpoint(0) = wp2_x;
          _guilinkRTdata.endingpoint(1) = wp2_y;

          return true;

        } else {
          CLOG(INFO, "gui-link") << " checksum error!";
        }
      }
    }  // end if
    return false;

  }  // parsedata_from_gui

  void senddata2gui(const guilinkRTdata<m> &_guilinkRTdata) {
    send_buffer.clear();
    send_buffer = "GUI";
    convert2string_lightweight(_guilinkRTdata, send_buffer);
    unsigned short crc =
        crc16.crcCompute(send_buffer.c_str(), send_buffer.length());
    send_buffer = "$" + send_buffer + "*" + std::to_string(crc) + "\n";
    bytes_send = gui_serial.write(send_buffer);
  }  // senddata2gui
};

}  // namespace ASV::messages

#endif /* _GUILINK_H_ */