/*
***********************************************************************
* gpsdata.h:
* header file to define the constant and real-time data in
* the GPS/IMU data
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _GPSDATA_H_
#define _GPSDATA_H_

#include <string>

namespace ASV::messages {

// real-time data from the gps/imu sensors
struct gpsRTdata {
  double UTC;       // hhmmss.ss  UTC time
  double latitude;  // Format: dd.dddd; -90 ~ 90 (positive to north, negative to
                    // south)
  double longitude;  // Format: ddd.dddd;-180 ~ 180 (positive to east, negative
                     // to west)
  double heading;    // 0 ~ 359.99, true north
  double pitch;      // -90 ~ 90
  double roll;       // -180 ~ 180
  double altitude;   // (m)
  double Ve;         // velocity in east direction (m/s)
  double Vn;         // velocity in north direction (m/s)
  double roti;       // rate of turn (degree/min)
  int status;

  double UTM_x;          // X in UTM projection (m)
  double UTM_y;          // Y in UTM projection (m)
  std::string UTM_zone;  //
};

// real-time data from the imu sensors
struct imuRTdata {
  double acc_X;  // acceleration in X (m/s^2)
  double acc_Y;  // acceleration in Y (m/s^2)
  double acc_Z;  // acceleration in Z (m/s^2)
  double roll;
  double pitch;
  double yaw;
  double roti;  // rate of turning
};

}  // namespace ASV::messages

#endif /* _GPSDATA_H_ */