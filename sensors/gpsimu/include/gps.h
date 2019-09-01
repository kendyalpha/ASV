/*
 *************************************************
 *gps.h
 *function for read and transter the data to UTM
 *using serial lib to read the serial data
 *using GeographicLib to transfer to UTM
 *the header file can be read by C++ compilers
 *
 *by ZH.Hu (CrossOcean.ai)
 *************************************************
 */

// TODO: adjust the central longitude and attitude for UTM

#ifndef _GPS_H_
#define _GPS_H_

#include <string.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <cstdio>
#include <exception>
#include <iostream>
#include "gpsdata.h"
#include "nmea.h"
#include "serial/serial.h"

class gpsimu final : public nmea {
 public:
  explicit gpsimu(int _zone,            // the UTM zone
                  bool _northp,         // hemisphere,
                  unsigned long _baud,  // baudrate
                  const std::string& _port = "/dev/ttyUSB0")
      : GPS_serial(_port, _baud, serial::Timeout::simpleTimeout(2000)),
        serial_buffer(""),
        tmercator(6378388,    // equatorial radius
                  1 / 297.0,  // flattening
                  GeographicLib::Constants::UTM_k0()),
        centrallong(6 * _zone - 183),
        falseeasting(5e5),
        falsenorthing(_northp ? 0 : 100e5) {
    if (!(_zone >= 1 && _zone <= 60))
      throw GeographicLib::GeographicErr("zone not in [1,60]");
  }
  gpsimu() = delete;
  ~gpsimu() {}

  // read serial data and transform to UTM
  void gpsonestep(gpsRTdata& _gpsdata) {
    serial_buffer = GPS_serial.readline(150);

    hemisphereV102(serial_buffer, _gpsdata);
  }

  std::string getserialbuffer() const { return serial_buffer; }

  void updateUTMzone(int zone,  // the UTM zone + hemisphere
                     bool northp) {
    centrallong = 6 * zone - 183;
    falsenorthing = (northp ? 0 : 100e5);  // true = N, false = S
  }

 private:
  /** serial data **/
  serial::Serial GPS_serial;
  std::string serial_buffer;
  /** Define a UTM projection for an arbitrary ellipsoid **/
  GeographicLib::TransverseMercator tmercator;  // The projection
  double centrallong;                           // Central longitude
  double falseeasting;
  double falsenorthing;

  /** real time NMEA data **/
  HEROT herot;
  PSAT psat;
  GPVTG gpvtg;
  GPGGA gpgga;

  void hemisphereV102(const std::string& _serial_buffer, gpsRTdata& _gpsdata) {
    if (_serial_buffer.find("$GPGGA") != std::string::npos) {
      nmea::nmea_parse(_serial_buffer, gpgga);
      _gpsdata.UTC = gpgga.UTC;
      _gpsdata.latitude = convertlatitudeunit(gpgga.latitude, gpgga.NS);
      _gpsdata.longitude = convertlongitudeunit(gpgga.longitude, gpgga.EW);
      _gpsdata.altitude = gpgga.altitude;
      _gpsdata.status = gpgga.gps_Q;
      Forward(_gpsdata.latitude, _gpsdata.longitude, _gpsdata.UTM_x,
              _gpsdata.UTM_y);

    } else if (_serial_buffer.find("$HEROT") != std::string::npos) {
      nmea::nmea_parse(_serial_buffer, herot);
      _gpsdata.roti = herot.rateofturning;
    } else if (_serial_buffer.find("$GPVTG") != std::string::npos) {
      nmea::nmea_parse(_serial_buffer, gpvtg);
      decomposespeed(gpvtg.K_speed, gpvtg.TMG, _gpsdata.Ve, _gpsdata.Vn);
    } else if (_serial_buffer.find("$PSAT") != std::string::npos) {
      nmea::nmea_parse(_serial_buffer, psat);
      _gpsdata.heading = psat.heading;
      _gpsdata.pitch = psat.pitch;
      _gpsdata.roll = psat.roll;
    } else {
      printf("No NMEA Data!\n");
    }
  }

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }
  // convert longitude and latitude to UTM
  void Forward(double lat, double lon, double& x, double& y) {
    tmercator.Forward(centrallong, lat, lon, x, y);
    x += falseeasting;
    y += falsenorthing;
  }
  // convert UTM to longitude and latitude
  void Reverse(double x, double y, double& lat, double& lon) {
    x -= falseeasting;
    y -= falsenorthing;
    tmercator.Reverse(centrallong, x, y, lat, lon);
  }

  // convert the unit of latitude (ddmm.mmm -> dd.dddddd)
  double convertlatitudeunit(double _latitude, char _NS) {
    double latitude = convertddmm2dd(_latitude);
    if (_NS == 'S') latitude *= (-1);
    return latitude;
  }  // convertlatitudeunit

  // convert the unit of longitude (dddmm.mmm -> dd.dddddd)
  double convertlongitudeunit(double _longitude, char _EW) {
    double longitude = convertddmm2dd(_longitude);
    if (_EW == 'W') longitude *= (-1);
    return longitude;
  }  // convertlongitudeunit

  // convert ddmm.mmmm to dd.dddddd
  double convertddmm2dd(double _ddmm) {
    double dd = std::floor(0.01 * _ddmm);
    double mm = (_ddmm - dd * 100) / 60.0;
    return dd + mm;
  }  // convertddmm2dd

  // generate the Ve and Vn
  void decomposespeed(double _speed, double _TMG, double& _Ve, double& _Vn) {
    _speed /= 3.6;         // km/h -> m/s
    _TMG *= (M_PI / 180);  // degree -> rad
    double cvalue = std::cos(_TMG);
    double svalue = std::sin(_TMG);
    _Ve = _speed * svalue;
    _Vn = _speed * cvalue;
  }
};

#endif