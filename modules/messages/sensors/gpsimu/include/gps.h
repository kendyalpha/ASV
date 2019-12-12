/*
 *************************************************
 * gps.h
 * function for read and transter the data to UTM
 * using serial lib to read the serial data
 * using GeographicLib to transfer to UTM
 * note: Earth's polar regions (areas of north of 84°N and south of 80°S)
 * are not convered in UTM grids.
 * the header file can be read by C++ compilers
 *
 * by ZH.Hu (CrossOcean.ai)
 *************************************************
 */

#ifndef _GPS_H_
#define _GPS_H_

#include <string.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <cstdio>
#include <exception>
#include <iostream>
#include <tuple>
#include "common/logging/include/easylogging++.h"
#include "modules/messages/sensors/gpsimu/include/gpsdata.h"
#include "modules/messages/sensors/gpsimu/include/nmea.h"
#include "third_party/serial/include/serial/serial.h"

namespace ASV::messages {
class GPS final : public nmea {
 public:
  explicit GPS(unsigned long _baud,                     // baudrate
               const std::string& _port = "/dev/ttyS0"  // serial port
               )
      : GPSdata({
            0,      // UTC
            0,      // latitude
            0,      // longitude
            0,      // heading
            0,      // pitch
            0,      // roll
            0,      // altitude
            0,      // Ve
            0,      // Vn
            0,      // roti
            0,      // status
            0,      // UTM_x
            0,      // UTM_y
            "NULL"  // UTM_zone
        }),
        GPS_serial(_port, _baud, serial::Timeout::simpleTimeout(2000)),
        serial_buffer("") {}
  GPS() = delete;
  ~GPS() {}

  // read serial data and transform to UTM
  GPS& parseGPS(const std::string& _planning_utm_zone = "OFF") {
    serial_buffer = GPS_serial.readline(150);
    hemisphereV102(serial_buffer, _planning_utm_zone, GPSdata);
    return *this;
  }

  auto getgpsRTdata() const noexcept { return GPSdata; }
  std::string getserialbuffer() const noexcept { return serial_buffer; }

 private:
  gpsRTdata GPSdata;
  /** serial data **/
  serial::Serial GPS_serial;
  std::string serial_buffer;

  /** real time NMEA data **/
  HEROT herot;
  PSAT psat;
  GPVTG gpvtg;
  GPGGA gpgga;

  void hemisphereV102(const std::string& _serial_buffer,
                      const std::string& _planning_utm_zone,
                      gpsRTdata& _gpsdata) {
    if (_serial_buffer.find("$GPGGA") != std::string::npos) {
      nmea::nmea_parse(_serial_buffer, gpgga);
      _gpsdata.UTC = gpgga.UTC;
      _gpsdata.latitude = convertlatitudeunit(gpgga.latitude, gpgga.NS);
      _gpsdata.longitude = convertlongitudeunit(gpgga.longitude, gpgga.EW);
      _gpsdata.altitude = gpgga.altitude;
      _gpsdata.status = gpgga.gps_Q;
      auto [utm_x, utm_y, utm_zone] =
          Forward(_gpsdata.latitude, _gpsdata.longitude);
      _gpsdata.UTM_x = utm_x;
      _gpsdata.UTM_y = utm_y;
      _gpsdata.UTM_zone = utm_zone;

      if (_planning_utm_zone != "OFF")
        check_UTM_zone(_planning_utm_zone, _gpsdata);
      check_gps_status(_gpsdata);

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
      CLOG(ERROR, "GPS") << "No NMEA Data!";
    }
  }  // hemisphereV102

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }

  // check the gps status and give the warning
  void check_gps_status(const gpsRTdata& _gpsdata) {
    if (_gpsdata.status == 0) CLOG(ERROR, "GPS") << "GPS no fix!";
  }

  // convert longitude and latitude to UTM
  std::tuple<double, double, std::string> Forward(
      const double lat,  // latitude of point (degrees)
      const double lon   //  longitude of point (degrees)
  ) {
    int zone = 0;        // the UTM zone (zero means UPS).
    bool northp = true;  // hemisphere (true means north, false means south).
    double x = 0.0;      // easting of point (meters)
    double y = 0.0;      // northing of point (meters)
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
    std::string zonestr = GeographicLib::UTMUPS::EncodeZone(zone, northp);
    return {x, y, zonestr};
  }  // Forward

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
  }  // decomposespeed

  // check if the real-time UTM zone is in the planning UTM zone, and smooth
  void check_UTM_zone(const std::string& _planning_utm_zone,
                      gpsRTdata& _gpsdata) {
    if (_gpsdata.UTM_zone != "NULL" &&
        _gpsdata.UTM_zone != _planning_utm_zone) {
      double smooth_utm_x = 0.0;
      double smooth_utm_y = 0.0;

      int zone = 0;
      bool northp = true;
      int zone_planning = 0;
      bool northp_planning = true;
      int zone_t = 0;
      // smooth the utm projection when UTM zone is switched
      GeographicLib::UTMUPS::DecodeZone(_gpsdata.UTM_zone, zone, northp);
      GeographicLib::UTMUPS::DecodeZone(_planning_utm_zone, zone_planning,
                                        northp_planning);

      // transfer one zone to another
      GeographicLib::UTMUPS::Transfer(
          zone, northp, _gpsdata.UTM_x, _gpsdata.UTM_y, zone_planning,
          northp_planning, smooth_utm_x, smooth_utm_y, zone_t);

      _gpsdata.UTM_x = smooth_utm_x;
      _gpsdata.UTM_y = smooth_utm_y;
    }

  }  // check_UTM_zone

};  // end class GPS

}  // namespace ASV::messages

#endif