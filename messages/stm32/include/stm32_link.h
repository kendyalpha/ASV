/*
***********************************************************************
* stm32_link.h: link with stm32, to motor control, error report, etc
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _STM32_LINK_H_
#define _STM32_LINK_H_

#include <chrono>
#include <thread>
#include "crc.h"
#include "easylogging++.h"
#include "serial/serial.h"
#include "stm32data.h"
#include "timecounter.h"

namespace ASV {

class stm32_link {
 public:
  explicit stm32_link(const stm32data& _stm32data,  //
                      unsigned long _baud,          // baudrate
                      const std::string& _port = "/dev/ttyUSB0")
      : stmdata(_stm32data),
        stm32_serial(_port, _baud, serial::Timeout::simpleTimeout(100)),
        send_buffer(""),
        recv_buffer(""),
        bytes_send(0),
        bytes_reci(0),
        crc16(CRC16::eCCITT_FALSE),
        connectionstatus(10) {
    checkserialstatus();
  }
  virtual ~stm32_link() = default;

  // communication with stm32
  stm32_link& stm32onestep() {
    checkconnection(stmdata);
    senddata2stm32(stmdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    parsedata_from_stm32(stmdata);
    return *this;
  }

  stm32_link& setstm32data(const stm32data& _stm32data) {
    stmdata = _stm32data;
    return *this;
  }
  auto getstmdata() const noexcept { return stmdata; }
  std::string getrecv_buffer() const noexcept { return recv_buffer; }
  std::string getsend_buffer() const noexcept { return send_buffer; }

 private:
  stm32data stmdata;
  /** serial data **/
  serial::Serial stm32_serial;
  std::string send_buffer;
  std::string recv_buffer;
  std::size_t bytes_send;
  std::size_t bytes_reci;

  CRC16 crc16;

  int connectionstatus;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }

  void checkconnection(stm32data& _stm32data) {
    if (connectionstatus > 10)
      _stm32data.linkstatus = LINKSTATUS::DISCONNECTED;
    else
      _stm32data.linkstatus = LINKSTATUS::CONNECTED;
  }

  void checkserialstatus() {
    if (stm32_serial.isOpen())
      CLOG(INFO, "stm32-serial") << " serial port open successful!";
    else
      CLOG(INFO, "stm32-serial") << " serial port open failure!";
  }

  void parsedata_from_stm32(stm32data& _stm32data) {
    recv_buffer = stm32_serial.readline(100, "\n");

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
          connectionstatus = 0;  // reset to zero
          int _stm32status = 0;
          sscanf(recv_buffer.c_str(),
                 "PC,"
                 "%d,"   // stm32status
                 "%lf,"  // voltage_b1
                 "%lf,"  // voltage_b2
                 "%lf,"  // voltage_b3
                 "%d,"   // feedback_n1
                 "%d,"   // feedback_n2
                 "%lf,"  // RC_X
                 "%lf,"  // RC_Y
                 "%lf"   // RC_Mz
                 ,
                 &_stm32status,              // int
                 &(_stm32data.voltage_b1),   // double
                 &(_stm32data.voltage_b2),   // double
                 &(_stm32data.voltage_b2),   // double
                 &(_stm32data.feedback_n1),  // int
                 &(_stm32data.feedback_n2),  // int
                 &(_stm32data.RC_X),         // double
                 &(_stm32data.RC_Y),         // double
                 &(_stm32data.RC_Mz)         // double
          );

          _stm32data.stm32status = static_cast<STM32STATUS>(_stm32status);

        } else {
          connectionstatus++;
          CLOG(INFO, "stm32-serial") << " checksum error!";
        }
      }
    }
  }  // parsedata_from_stm32

  void senddata2stm32(const stm32data& _stm32data) {
    send_buffer.clear();
    send_buffer = "STM";
    convert2string(_stm32data, send_buffer);
    unsigned short crc =
        crc16.crcCompute(send_buffer.c_str(), send_buffer.length());
    send_buffer = "$" + send_buffer + "*" + std::to_string(crc) + "\n";
    bytes_send = stm32_serial.write(send_buffer);
  }  // senddata2stm32

  void convert2string(const stm32data& _stm32data, std::string& _str) {
    _str += ",";
    _str += _stm32data.UTC_time;
    _str += ",";
    _str += std::to_string(_stm32data.command_n1);
    _str += ",";
    _str += std::to_string(_stm32data.command_n2);
    _str += ",";
    _str += std::to_string(static_cast<int>(_stm32data.linkstatus));
  }  // convert2string

};  // end class stm32_link

}  // end namespace ASV

#endif /* _STM32_LINK_H_ */