/*
 *************************************************
 *wind.h
 *function for read and parse the data from wind sensor
 *the header file can be read by C++ compilers
 *
 *by Wei Li (SJTU)
 *************************************************
 */

#ifndef _WIND_H_
#define _WIND_H_

#include <cmath>
#include "third_party/serial/include/serial/serial.h"
#include "winddata.h"

namespace ASV {

class wind {
 public:
  explicit wind(unsigned long _baud,  // baudrate
                const std::string& _port = "/dev/ttyUSB0")
      : ser_wind(_port, _baud, serial::Timeout::simpleTimeout(2000)) {}
  wind() = delete;
  ~wind() {}

  wind& readwind() {
    if (ser_wind.available()) {
      ser_wind.read(s_buffer, 7);
      if (s_buffer[0] == 0xaa) {
        int _wspeed = 0;
        int _orientation = 0;
        _wspeed = s_buffer[2] << 8 | s_buffer[3];
        _orientation = s_buffer[4] << 8 | s_buffer[5];
        restrictdata(_wspeed, _orientation);
        _windRTdata.speed = _wspeed / 10.0;
        _windRTdata.orientation = _orientation * M_PI / 180;
      }
    }

    return *this;
  }

  windRTdata getwindRTdata() const { return _windRTdata; }

 private:
  // serial data
  serial::Serial ser_wind;
  windRTdata _windRTdata;
  uint8_t s_buffer[7];

  void restrictdata(int& _wspeed, int& _orientation) {
    if ((_wspeed < 0) || (_wspeed > 300)) _wspeed = 0;
    if ((_orientation < 0) || (_orientation >= 360)) _orientation = 0;
  }
};  // end class wind

}  // end namespace ASV

#endif /*_WIND_H_*/
