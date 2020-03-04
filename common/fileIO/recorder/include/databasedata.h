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

#include <common/math/eigen/Eigen/Core>
#include <string>
#include <vector>

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
};  // gps_db_data

struct imu_db_data {
  double local_time;
  double Acc_X;
  double Acc_Y;
  double Acc_Z;
  double Ang_vel_X;
  double Ang_vel_Y;
  double Ang_vel_Z;
  double roll;
  double pitch;
  double yaw;
};  // imu_db_data

struct wind_db_data {
  double local_time;
  double speed;
  double orientation;
};  // wind_db_data

struct stm32_db_data {
  double local_time;
  int stm32_link;
  int stm32_status;
  double command_u1;
  double command_u2;
  double feedback_u1;
  double feedback_u2;
  int feedback_pwm1;
  int feedback_pwm2;
  double RC_X;
  double RC_Y;
  double RC_Mz;
  double voltage_b1;
  double voltage_b2;
  double voltage_b3;
};  // stm32_db_data

struct marineradar_db_data {
  double local_time;
  double azimuth_deg;
  double sample_range;
  std::vector<uint8_t> spokedata;
};  // marineradar_db_data

struct est_measurement_db_data {
  double local_time;
  double meas_x;
  double meas_y;
  double meas_theta;
  double meas_u;
  double meas_v;
  double meas_r;
};  // est_measurement_db_data

struct est_state_db_data {
  double local_time;
  double state_x;
  double state_y;
  double state_theta;
  double state_u;
  double state_v;
  double state_r;
  double curvature;
  double speed;
  double dspeed;
};  // est_state_db_data

struct est_error_db_data {
  double local_time;
  double perror_x;
  double perror_y;
  double perror_mz;
  double verror_x;
  double verror_y;
  double verror_mz;
};  // est_error_db_data

struct plan_route_db_data {
  double local_time;
  double setpoints_X;
  double setpoints_Y;
  double setpoints_heading;
  double setpoints_longitude;
  double setpoints_latitude;
  double speed;
  double captureradius;
  std::string utm_zone;
  std::vector<double> WPX;
  std::vector<double> WPY;
  std::vector<double> WPLONG;
  std::vector<double> WPLAT;
};  // plan_route_db_data

struct plan_lattice_db_data {
  double local_time;
  double lattice_x;
  double lattice_y;
  double lattice_theta;
  double lattice_kappa;
  double lattice_speed;
  double lattice_dspeed;
};  // plan_lattice_db_data

struct control_setpoint_db_data {
  double local_time;
  double set_x;
  double set_y;
  double set_theta;
  double set_u;
  double set_v;
  double set_r;
};  // control_setpoint_db_data

struct control_TA_db_data {
  double local_time;
  double desired_Fx;
  double desired_Fy;
  double desired_Mz;
  double est_Fx;
  double est_Fy;
  double est_Mz;
  std::vector<int> alpha;
  std::vector<int> rpm;
};  // control_TA_db_data

struct perception_spoke_db_data {
  double local_time;
  std::vector<double> surroundings_bearing_rad;
  std::vector<double> surroundings_range_m;
  std::vector<double> surroundings_x_m;
  std::vector<double> surroundings_y_m;
};  // perception_spoke_db_data

struct perception_detection_db_data {
  double local_time;
  std::vector<double> detected_target_x;
  std::vector<double> detected_target_y;
  std::vector<double> detected_target_radius;
};  // perception_detection_db_data

struct perception_trackingtarget_db_data {
  double local_time;
  int spoke_state;
  std::vector<int> targets_state;
  std::vector<int> targets_intention;
  std::vector<double> targets_x;
  std::vector<double> targets_y;
  std::vector<double> targets_square_radius;
  std::vector<double> targets_vx;
  std::vector<double> targets_vy;
  std::vector<double> targets_CPA_x;
  std::vector<double> targets_CPA_y;
  std::vector<double> targets_TCPA;
};  // perception_trackingtarget_db_data

}  // namespace ASV::common

#endif /* _DATABASEDATA_H_ */