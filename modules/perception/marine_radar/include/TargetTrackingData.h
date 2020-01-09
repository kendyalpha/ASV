/*
****************************************************************************
* TargetTrackingData.h:
* obstacle detection using spoke data from marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _TARGETTRACKINGDATA_H_
#define _TARGETTRACKINGDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

namespace ASV::perception {

enum class SPOKESTATE {
  OUTSIDE_ALARM_ZONE = 0,  // outside the alarm zone
  ENTER_ALARM_ZONE,        // enter the alarm zone
  IN_ALARM_ZONE,           // in the alarm zone
  LEAVE_ALARM_ZONE,        // leave the alarm zone
};

struct SpokeProcessdata {
  double sample_time;
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

// all spoke data in the alarm zone
struct SpokeProcessRTdata {
  // surroundings in the body-fixed coordinate
  std::vector<double> surroundings_bearing_rad;
  std::vector<double> surroundings_range_m;
  // surroundings in the marine coordinate
  std::vector<double> surroundings_x_m;
  std::vector<double> surroundings_y_m;
};

struct AlphaBetaData {
  double sample_time;
  double alpha;
  double beta;
};

struct ClusteringData {
  double p_radius;  // radius of a neighborhood with respect to some point
  std::size_t p_minumum_neighbors;  //
};

struct TargetTrackerRTdata {
  // target position and velocity in the marine coordinate
  std::vector<double> target_x;
  std::vector<double> target_y;
  std::vector<double> target_square_radius;
  std::vector<double> target_vx;
  std::vector<double> target_vy;
};

}  // namespace ASV::perception

#endif /* _TARGETTRACKINGDATA_H_ */