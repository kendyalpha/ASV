/*
*******************************************************************************
* testNMEA.cc:
* unit test for NMEA parser
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <cstdio>
#include <iostream>
#include "../include/nmea.h"
using std::setprecision;

void testGPGGA() {
  // real time GPS/IMU data
  GPGGA gps_data{
      0,  // UTC
      0,  // latitude
      0,  // NS
      0,  // longitude
      0,  // EW
      0,  // gps_Q
      0,  // NSV
      0,  // hd
      0,  // altitude
      0,  // GS
      0,  // age
      0   // ds_ID
  };

  std::string str =
      "$GPGGA,044551.80,3101.7197881,N,12126.3598910,E,2,08,1.1,4.316,M,9.725,"
      "M,"
      "6.8,0129*7E";
  nmea _nmea;
  _nmea.nmea_parse(str, gps_data);

  // print GPGGA
  {
    printf("UTC: %lf\n", gps_data.UTC);
    printf("latitude: %lf\n", gps_data.latitude);
    printf("N or S: %c\n", gps_data.NS);
    printf("longitude: %lf\n", gps_data.longitude);
    printf("E or W: %c\n", gps_data.EW);
    printf("GPS Quality: %d\n", gps_data.gps_Q);
    printf("NSV: %d\n", gps_data.NSV);
    printf("dilution: %lf\n", gps_data.hd);
    printf("altitude: %lf\n", gps_data.altitude);
    printf("GS: %lf\n", gps_data.GS);
    printf("age: %lf\n", gps_data.age);
  }
}

void testHEROT() {
  // real time GPS/IMU data
  HEROT gps_data{
      0  // rot
  };

  std::string str = "$HEROT,-1.4,A*03";
  nmea _nmea;
  _nmea.nmea_parse(str, gps_data);

  // print GPGGA
  { printf("ROT: %lf\n", gps_data.rateofturning); }
}

void testGPVTG() {
  // real time GPS/IMU data
  GPVTG gps_data{
      0,  // TMG
      0,  // T
      0,  // MTMG
      0,  // M
      0,  // N_speed
      0,  // N
      0,  // K_speed
      0   // K
  };

  std::string str = "$GPVTG,13.48,T,19.24,M,0.11,N,0.21,K,D*25";
  nmea _nmea;
  _nmea.nmea_parse(str, gps_data);

  // print GPGGA
  {
    printf("TMG: %lf\n", gps_data.TMG);
    printf("MTMG: %lf\n", gps_data.MTMG);
    printf("N_speed: %lf\n", gps_data.N_speed);
    printf("K_speed: %lf\n", gps_data.K_speed);
  }
}

void testGPRMC() {
  // real time GPS/IMU data
  GPRMC gps_data{
      0,  // timestamp
      0,  // status
      0,  // latitude
      0,  // NS
      0,  // longitude
      0,  // EW
      0,  // speed
      0,  // course
      0,  // datestamp
      0   // mvd
  };

  std::string str =
      "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68";
  nmea _nmea;
  _nmea.nmea_parse(str, gps_data);

  // print GPGGA
  {
    printf("timestamp: %d\n", gps_data.timestamp);
    printf("status: %c\n", gps_data.status);
    printf("latitude: %lf\n", gps_data.latitude);
    printf("NS: %c\n", gps_data.NS);
    printf("longitude: %lf\n", gps_data.longitude);
    printf("EW: %c\n", gps_data.EW);
    printf("speed: %lf\n", gps_data.speed);
    printf("course: %lf\n", gps_data.course);
    printf("datestamp: %d\n", gps_data.datestamp);
    printf("mvd : %lf\n", gps_data.mvd);
  }
}

void testPSAT() {
  // real time GPS/IMU data
  PSAT gps_data{
      0,  // timestamp
      0,  // heading
      0,  // roll
      0   // pitch
  };

  std::string str = "$PSAT,HPR,044551.80,161.54,-2.84,1.4,N*3A";
  nmea _nmea;
  _nmea.nmea_parse(str, gps_data);

  // print GPGGA
  {
    printf("timestamp: %lf\n", gps_data.UTC);
    printf("heading: %lf\n", gps_data.heading);
    printf("roll: %lf\n", gps_data.roll);
    printf("pitch: %lf\n", gps_data.pitch);
  }
}

int main() {
  // testGPGGA();
  // testHEROT();
  // testGPVTG();
  // testGPRMC();
  testPSAT();
}