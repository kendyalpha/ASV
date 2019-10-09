/*
*******************************************************************************
* winddata.h:
* define the data struct used in the wind sensors
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _WINDDATA_H_
#define _WINDDATA_H_

#include <vector>

namespace ASV {
union windsocketmsg {
  double double_msg[2];
  char char_msg[16];
};

struct windRTdata {
  double speed;        // m/s
  double orientation;  // rad
};

}  //  end namespace ASV

#endif /* _WINDDATA_H_ */