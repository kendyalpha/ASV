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

enum class TARGETSTATE {
  IDLE = 0,   // initial target state, or lost out of range, or acquire fail
  ACQUIRING,  //
  ACQUIRED    // successfully acquired
};

enum class INTENTIONSTATE {
  SAFE = 0,  //
  DANGEROUS
};

struct SpokeProcessdata {
  double sample_time;
  double radar_x;  // x of radar relative to CoG, in the body-fixed coordinate
  double radar_y;  // y of radar relative to CoG, in the body-fixed coordinate
};

struct AlarmZone {
  double start_range_m;          // minimum range of alarm zone
  double end_range_m;            // maximum range of alarm zone
  double center_bearing_rad;     // bearing of center of alarm zone (-Pi ~ Pi)
  double width_bearing_rad;      // bearing width of alarm zone (<2 Pi)
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

struct TrackingTargetData {
  // the minimum squared radius of detected targets (m^2)
  double min_squared_radius;  // too small to be ignore
  double max_squared_radius;  // too big to be removed
  double speed_threhold;      //
  double max_speed;           // the maximum speed of detected targets (m/s)
  double max_acceleration;  // the max acceleration of detected targets (m/s^2)
  double max_roti;          // the maximum ROTI of detected targets (deg/min)
  double safe_distance;

  // loss function
  double K_radius;
  double K_delta_speed;
  double K_delta_yaw;
};

struct ClusteringData {
  double p_radius;  // radius of a neighborhood with respect to some point
  std::size_t p_minumum_neighbors;  //
};

struct TargetDetectionRTdata {
  // detected target position in the marine coordinate
  std::vector<double> target_x;
  std::vector<double> target_y;
  std::vector<double> target_square_radius;
};

template <int max_num_target = 20>
struct TargetTrackerRTdata {
  // spoke state
  SPOKESTATE spoke_state;
  // target state
  Eigen::Matrix<int, max_num_target, 1> targets_state;
  Eigen::Matrix<int, max_num_target, 1> targets_intention;

  // target position and velocity in the marine coordinate
  Eigen::Matrix<double, max_num_target, 1> targets_x;
  Eigen::Matrix<double, max_num_target, 1> targets_y;
  Eigen::Matrix<double, max_num_target, 1> targets_square_radius;
  Eigen::Matrix<double, max_num_target, 1> targets_vx;
  Eigen::Matrix<double, max_num_target, 1> targets_vy;
  // CPA and TCPA (closest point of approach),
  Eigen::Matrix<double, max_num_target, 1> targets_CPA_x;
  Eigen::Matrix<double, max_num_target, 1> targets_CPA_y;
  // when TCPA<0, no collision
  Eigen::Matrix<double, max_num_target, 1> targets_TCPA;
};

}  // namespace ASV::perception

#endif /* _TARGETTRACKINGDATA_H_ */