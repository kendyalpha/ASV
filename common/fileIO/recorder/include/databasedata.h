/*
***********************************************************************
* databasedata.h:
* data used in database
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DATABASEDATA_H_
#define _DATABASEDATA_H_

#include <string>

namespace ASV::common {

struct gps_db_data {
  double local_time;
  double UTC;
  double latitude;
  double longitude;
  double heading;
  double pitch;
  double roll;
  double altitude;
  double Ve;
  double Vn;
  double roti;
  int status;
  double UTM_x;
  double UTM_y;
  std::string UTM_zone;
};
}  // namespace ASV::common

#endif /* _DATABASEDATA_H_ */