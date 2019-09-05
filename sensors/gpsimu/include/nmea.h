/*
 *************************************************
 *nmea.h
 *function to parse nmea data format
 *the header file can be read by C++ compilers
 *
 *by ZH.Hu (CrossOcean.ai)
 *************************************************
 */

#ifndef _NMEA_H_
#define _NMEA_H_

#include <iomanip>
#include <string>
// Global Positioning System Fix Data
struct GPGGA {
  double UTC;        // hhmmss.ss = UTC of position
  double latitude;   // 纬度 ddmm.mmmmm
  char NS;           // N or S
  double longitude;  // 经度 dddmm.mmmmm
  char EW;           // E or W
  int gps_Q;  // GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
  int NSV;    // # of satellites in use [not those in view]
  double hd;  // Horizontal dilution of position

  // Antenna altitude above/below mean sea level (geoid) unit: m
  double altitude;
  // Geoidal separation (Diff. between WGS-84 earth ellipsoid and
  // mean sea level.  -=geoid is below WGS-84 ellipsoid), unit: m
  double GS;
  // Age in seconds since last update from diff. reference station
  double age;
  // Differential reference station ID
  int ds_ID;
  // checksum
};

// Recommended minimum specific GPS/Transit data
struct GPRMC {
  int timestamp;     // hhmmss.ss = UTC of position
  char status;       // Data status ( A-ok, V-invalid)
  double latitude;   // 纬度 ddmm.mmmmm
  char NS;           // N or S
  double longitude;  // 经度 dddmm.mmmmm
  char EW;           // E or W
  double speed;      // Speed over ground in knots
  double course;     // True course
  int datestamp;     //
  double mvd;        // Magnetic variation degrees
};

// Track Made Good and Ground Speed
struct GPVTG {
  double TMG;   // Track made good
  char T;       // Fixed text 'T' indicates that track made good is relative to
                // true north
  double MTMG;  // Magnetic track made good
  char M;
  double N_speed;  // Speed over ground in knots
  char N;  // Fixed text 'N' indicates that speed over ground in in knots
  double K_speed;  // Speed over ground in kilometers/hour
  char K;          // Fixed text 'K' indicates that speed over ground is in
                   // kilometers/hour
};

// rate of turning (unit: degrees per min)
struct HEROT {
  double rateofturning;
};

// rotation angle
struct PSAT {
  double UTC;      // hhmmss.ss = UTC of position;
  double heading;  // degree
  double roll;     // degree
  double pitch;    // degree
};

class nmea {
 public:
  nmea() {}
  virtual ~nmea() = default;

  void nmea_parse(const std::string &_str, GPGGA &gps_data) {
    std::size_t pos = _str.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      std::string gpgga_buffer = _str.substr(pos + 1);
      std::size_t rpos = gpgga_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected xor checksum value
        uint8_t expected_chk =
            (uint8_t)strtol(gpgga_buffer.substr(rpos + 1).c_str(), NULL, 16);

        // remove string after "*"
        gpgga_buffer = gpgga_buffer.substr(0, rpos);

        if (xor_checksum(gpgga_buffer) == expected_chk) {
          char meters = '0';
          sscanf(gpgga_buffer.c_str(),
                 "GPGGA,"
                 "%lf,"  // UTC
                 "%lf,"  // latitude
                 "%c,"   // NS
                 "%lf,"  // longitude
                 "%c,"   // EW
                 "%d,"   // gps_Q
                 "%d,"   // NSV
                 "%lf,"  // hd
                 "%lf,"  // altitude
                 "%c,"   // meters
                 "%lf,"  // GS
                 "%c,"   // meters
                 "%lf,"  // age
                 "%d"    // ds_ID
                 ,
                 &(gps_data.UTC),        // double
                 &(gps_data.latitude),   // double
                 &(gps_data.NS),         // char
                 &(gps_data.longitude),  // double
                 &(gps_data.EW),         // char
                 &(gps_data.gps_Q),      // int
                 &(gps_data.NSV),        // int
                 &(gps_data.hd),         // double
                 &(gps_data.altitude),   // double
                 &meters,                // char
                 &(gps_data.GS),         // double
                 &meters,                // char
                 &(gps_data.age),        // double
                 &(gps_data.ds_ID)       // int
          );
        } else {
          printf("GPGGA checksum not OK!\n");
        }
      }
    }
  }

  void nmea_parse(const std::string &_str, GPRMC &gps_data) {
    std::size_t pos = _str.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      std::string gprmc_buffer = _str.substr(pos + 1);
      std::size_t rpos = gprmc_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected xor checksum value
        uint8_t expected_chk =
            (uint8_t)strtol(gprmc_buffer.substr(rpos + 1).c_str(), NULL, 16);

        // remove string after "*"
        gprmc_buffer = gprmc_buffer.substr(0, rpos);

        if (xor_checksum(gprmc_buffer) == expected_chk) {
          sscanf(gprmc_buffer.c_str(),
                 "GPRMC,"
                 "%d,"   // timestamp
                 "%c,"   // status
                 "%lf,"  // latitude
                 "%c,"   // NS
                 "%lf,"  // longitude
                 "%c,"   // EW
                 "%lf,"  // speed
                 "%lf,"  // course
                 "%d,"   // datestamp
                 "%lf"   // mvd
                 ,
                 &(gps_data.timestamp),  // int
                 &(gps_data.status),     // char
                 &(gps_data.latitude),   // double
                 &(gps_data.NS),         // char
                 &(gps_data.longitude),  // double
                 &(gps_data.EW),         // char
                 &(gps_data.speed),      // double
                 &(gps_data.course),     // double
                 &(gps_data.datestamp),  // int
                 &(gps_data.mvd)         // double
          );
        } else {
          printf("GPRMC checksum not OK!\n");
        }
      }
    }
  }

  void nmea_parse(const std::string &_str, GPVTG &gps_data) {
    std::size_t pos = _str.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      std::string gpvtg_buffer = _str.substr(pos + 1);
      std::size_t rpos = gpvtg_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected xor checksum value
        uint8_t expected_chk =
            (uint8_t)strtol(gpvtg_buffer.substr(rpos + 1).c_str(), NULL, 16);

        // remove string after "*"
        gpvtg_buffer = gpvtg_buffer.substr(0, rpos);

        if (xor_checksum(gpvtg_buffer) == expected_chk) {
          sscanf(gpvtg_buffer.c_str(),
                 "GPVTG,"
                 "%lf,"  // TMG
                 "%c,"   // T
                 "%lf,"  // MTMG
                 "%c,"   // M
                 "%lf,"  // N_speed
                 "%c,"   // N
                 "%lf,"  // K_speed
                 "%c"    // K
                 ,
                 &(gps_data.TMG),      // double
                 &(gps_data.T),        // char
                 &(gps_data.MTMG),     // double
                 &(gps_data.M),        // char
                 &(gps_data.N_speed),  // double
                 &(gps_data.N),        // char
                 &(gps_data.K_speed),  // double
                 &(gps_data.K)         // char

          );
        } else {
          printf("GPVTG checksum not OK!\n");
        }
      }
    }
  }

  void nmea_parse(const std::string &_str, HEROT &gps_data) {
    std::size_t pos = _str.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      std::string herot_buffer = _str.substr(pos + 1);
      std::size_t rpos = herot_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected xor checksum value
        uint8_t expected_chk =
            (uint8_t)strtol(herot_buffer.substr(rpos + 1).c_str(), NULL, 16);

        // remove string after "*"
        herot_buffer = herot_buffer.substr(0, rpos);

        if (xor_checksum(herot_buffer) == expected_chk) {
          sscanf(herot_buffer.c_str(),
                 "HEROT,"
                 "%lf"  // rateofturning
                 ,
                 &(gps_data.rateofturning)  // double

          );
        } else {
          printf("HEROT checksum not OK!\n");
        }
      }
    }
  }

  void nmea_parse(const std::string &_str, PSAT &gps_data) {
    std::size_t pos = _str.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      std::string psat_buffer = _str.substr(pos + 1);
      std::size_t rpos = psat_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected xor checksum value
        uint8_t expected_chk =
            (uint8_t)strtol(psat_buffer.substr(rpos + 1).c_str(), NULL, 16);

        // remove string after "*"
        psat_buffer = psat_buffer.substr(0, rpos);

        if (xor_checksum(psat_buffer) == expected_chk) {
          sscanf(psat_buffer.c_str(),
                 "PSAT,HPR,"
                 "%lf,"  // UTC
                 "%lf,"  // heading
                 "%lf,"  // pitch
                 "%lf,"  // roll
                 ,
                 &(gps_data.UTC),      // double
                 &(gps_data.heading),  // double
                 &(gps_data.pitch),    // double
                 &(gps_data.roll)      // double

          );
        } else {
          printf("PSAT checksum not OK!\n");
        }
      }
    }
  }

 private:
  // checksum using XOR
  unsigned char xor_checksum(const std::string &_str) {
    // The checksum is simple, just an XOR of all the bytes between the $ and
    // the * (not including the delimiters themselves), and written in
    // hexadecimal.
    unsigned char chk = 0;

    for (std::string::size_type i = 0; i < _str.size(); ++i) {
      chk ^= static_cast<unsigned char>(_str[i]);
    }

    return chk;
  }
};

#endif /*_NMEA_H_*/