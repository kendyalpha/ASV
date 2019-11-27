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
  double UTC;       // hhmmss.ss  UTC时间
  double latitude;  // 纬度 dd.dddd格式, -90 ~ 90 (正为北向，负为南向)
  double longitude;  //经度 ddd.dddd格式，-180 ~ 180 (正为东向，负为西向)
  double heading;   // 航向角 0 ~ 359.99, 以真北为参考基准
  double pitch;     // 俯仰角 -90 ~ 90
  double roll;      // 横滚角 -180 ~ 180
  double altitude;  // 高度 (m)
  double Ve;        // 东向速度 (m/s)
  double Vn;        // 北向速度 (m/s)
  double roti;      // 转向速率 (degree/min)
  int status;       // status
  /**** UTM projection   ****/
  double UTM_x;          // 投影的 x (m)
  double UTM_y;          // 投影的 y (m)
  std::string UTM_zone;  // 
};

}  // namespace ASV::messages


#endif /* _GPSDATA_H_ */