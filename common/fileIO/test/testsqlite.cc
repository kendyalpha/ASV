/*
*******************************************************************************
* testsqlite.cc: unit test for modern sqlite wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/database.h"
#include "common/timer/include/timecounter.h"

using namespace ASV;

int main() {
  const int m = 3;
  const int n = 3;
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  LOG(INFO) << "The program has started!";

  common::database<m, n> _sqlitetest("dbtest.db");
  _sqlitetest.initializetables();
  // real time GPS/IMU data
  messages::gpsRTdata gps_data{
      0,    // UTC
      0,    // latitude
      0,    // longitude
      0,    // heading
      0,    // pitch
      0,    // roll
      0,    // altitude
      0,    // Ve
      0,    // Vn
      0,    // roti
      0,    // status
      0,    // UTM_x
      0,    // UTM_y
      "0n"  // UTM_zone
  };
  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      common::STATETOGGLE::IDLE,            // state_toggle
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Cartesian_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };
  control::controllerRTdata<m, n> _controllerRTdata{
      common::STATETOGGLE::IDLE,                                // state_toggle
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),    // tau
      Eigen::Matrix<double, n, 1>::Zero(),                      // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0).finished(),  // command_u
      // vectormi()::Zero(),                    // command_rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3)
          .finished(),                   // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),  // command_alpha_deg
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0).finished(),  // feedback_u
      // vectormi()::Zero(),                    // feedback_rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3)
          .finished(),                  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()  // feedback_alpha_deg
  };

  control::trackerRTdata _trackRTdata{
      control::TRACKERMODE::STARTED,              // trackermode
      (Eigen::Vector3d() << 0, 0, 1).finished(),  // setpoint
      Eigen::Vector3d::Zero()                     // v_setpoint
  };

  planning::RoutePlannerRTdata _plannerRTdata{
      common::STATETOGGLE::IDLE,  // state_toggle
      0,                          // setpoints_X
      0,                          // setpoints_Y;
      0,                          // setpoints_heading;
      0,                          // setpoints_longitude;
      0,                          // setpoints_latitude;
      "0n",                       // UTM zone
      0,                          // speed
      0,                          // los_capture_radius
      Eigen::VectorXd::Zero(2),   // Waypoint_X
      Eigen::VectorXd::Zero(2),   // Waypoint_Y
      Eigen::VectorXd::Zero(2),   // Waypoint_longitude
      Eigen::VectorXd::Zero(2)    // Waypoint_latitude
  };

  windRTdata _windRTdata{
      0,  //  speed
      1   // orientation
  };

  messages::stm32data _stm32data{
      "",                              // UTC_time
      0,                               // command_u1
      -10,                             // command_u2
      0,                               // feedback_u1
      0,                               // feedback_u2
      0,                               // feedback_pwm1
      0,                               // feedback_pwm2
      0,                               // RC_X
      0,                               // RC_Y
      0,                               // RC_Mz
      0,                               // voltage_b1
      0,                               // voltage_b2
      0,                               // voltage_b2
      messages::STM32STATUS::STANDBY,  // stm32status
      messages::STM32STATUS::STANDBY,  // stm32status
      common::LINKSTATUS::CONNECTED    // linkstatus;
  };

  perception::SpokeProcessRTdata Spoke_RTdata{
      std::vector<double>({1, 3, 4}),  // surroundings_bearing_rad
      std::vector<double>({1, 3, 4}),  // surroundings_range_m
      std::vector<double>({1, 3, 4}),  // surroundings_x_m
      std::vector<double>({1, 3, 4})   // surroundings_y_m
  };

  common::timecounter _timer;

  for (int i = 0; i != 3; ++i) {
    _sqlitetest.update_gps_table(gps_data);
    _sqlitetest.update_estimator_table(_estimatorRTdata);
    _sqlitetest.update_controller_table(_controllerRTdata, _trackRTdata);
    _sqlitetest.update_routeplanner_table(_plannerRTdata);
    _sqlitetest.update_wind_table(_windRTdata);
    _sqlitetest.update_stm32_table(_stm32data);
    _sqlitetest.update_surroundings_table(Spoke_RTdata);
  }
  long int et = _timer.timeelapsed();
  std::cout << "time:" << et << std::endl;

  LOG(INFO) << "Shutting down.";
  return 0;
}
