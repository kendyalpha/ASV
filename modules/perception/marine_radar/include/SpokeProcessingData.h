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

struct RadarConfig {
  double radar_x;  // x of radar relative to CoG, in the body-fixed coordinate
  double radar_y;  // y of radar relative to CoG, in the body-fixed coordinate
};

struct AlarmZone {
  double start_range_m;
  double end_range_m;
  double center_bearing_rad;
  double width_bearing_rad;
  uint8_t sensitivity_threhold;  // min sensitivity
};

struct SpokeProcessRTdata {
  // surroundings in the body-fixed coordinate
  std::vector<double> surroundings_bearing_rad;
  std::vector<double> surroundings_range_m;
  // surroundings in the marine coordinate
  std::vector<double> surroundings_x_m;
  std::vector<double> surroundings_y_m;
};

}  // namespace ASV::perception

#endif /* _SPOKEPROCESSINGDATA_H_ */