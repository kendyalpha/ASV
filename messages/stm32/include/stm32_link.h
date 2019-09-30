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
        stm32_serial(_port, _baud, serial::Timeout::simpleTimeout(1000)),
        serial_buffer(""),
  {}
  virtual ~stm32_link() = default;

  // communication with stm32
  stm32_link& stm32onestep() {
    serial_buffer = stm32_serial.readline(150);
    hemisphereV102(serial_buffer, GPSdata);
    return *this;
  }

  auto getstmdata() const noexcept { return stmdata; }
  std::string getserialbuffer() const noexcept { return serial_buffer; }

 private:
  stm32data stmdata;
  /** serial data **/
  serial::Serial stm32_serial;
  std::string send_buffer;
  std::string recv_buffer;
  std::size_t bytes_send;
  std::size_t bytes_reci;

  CRC16 crc16;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }

  void judgeserialstatus() {
    if (stm32_serial.isOpen())
      CLOG(INFO, "stm32-serial") << " serial port open successful!";
    else
      CLOG(INFO, "stm32-serial") << " serial port open failure!";
  }

  void parsedata_from_stm32() {}

  void senddata2stm32() {}

};  // end class stm32_link

}  // end namespace ASV

#endif /* _STM32_LINK_H_ */