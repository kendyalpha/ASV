/*
****************************************************************************
* SpokeProcessingData.h:
* obstacle detection using spoke data from marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _SPOKEPROCESSINGDATA_H_
#define _SPOKEPROCESSINGDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

namespace ASV::perception {

struct AlarmZone {
  uint32_t start_range_m;
  uint32_t end_range_m;
  uint32_t start_bearing_deg;
  uint32_t end_bearing_deg;
  uint8_t sensitivity_threhold;  // min sensitivity
};

struct SpokeProcessRTdata {
  // obstacle in the body-fixed coordinate
  std::vector<double> obstacle_bearing_deg;
  std::vector<double> obstacle_range_m;
  // obstacle in the UTM
  std::vector<double> obstacle_x_m;
  std::vector<double> obstacle_y_m;
};

}  // namespace ASV::perception

#endif /* _SPOKEPROCESSINGDATA_H_ */