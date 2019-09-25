/*
*******************************************************************************
* testsqlite.cc: unit test for modern sqlite wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "database.h"
#include "timecounter.h"

using namespace ASV;

int main() {
  const int m = 3;
  const int n = 3;
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  LOG(INFO) << "The program has started!";

  database<m, n> _sqlitetest("dbtest.db");
  _sqlitetest.initializetables();
  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,  // UTC
      0,  // latitude
      0,  // longitude
      0,  // heading
      0,  // pitch
      0,  // roll
      0,  // altitude
      0,  // Ve
      0,  // Vn
      0,  // roti
      0,  // status
      0,  // UTM_x
      0   // UTM_y
  };
  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
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
  controllerRTdata<m, n> _controllerRTdata{
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),    // tau
      Eigen::Matrix<double, n, 1>::Zero(),                      // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0).finished(),  // u
      // vectormi()::Zero(),                    // rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3)
          .finished(),                  // alpha
      Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg

  };

  trackerRTdata _trackRTdata{
      (Eigen::Vector3d() << 0, 0, 1).finished(),  // setpoint
      Eigen::Matrix<double, 3, 1>::Zero()         // v_setpoint
  };

  plannerRTdata _plannerRTdata{
      0, 0, Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),        // waypoint1
      Eigen::Vector3d::Zero()         // command
  };

  indicators _indicators{
      0,  // gui_connection
      0,  // joystick_connection
      0,  // indicator_controlmode
      1,  // indicator_windstatus
  };

  windRTdata _windRTdata{
      0,  //  speed
      1   // orientation
  };

  timecounter _timer;

  for (int i = 0; i != 3; ++i) {
    _sqlitetest.update_gps_table(gps_data);
    _sqlitetest.update_estimator_table(_estimatorRTdata);
    _sqlitetest.update_controller_table(_controllerRTdata, _trackRTdata);
    _sqlitetest.update_planner_table(_plannerRTdata);
    _sqlitetest.update_indicators_table(_indicators);
    _sqlitetest.update_wind_table(_windRTdata);
  }
  long int et = _timer.timeelapsed();
  std::cout << "time:" << et << std::endl;

  LOG(INFO) << "Shutting down.";
  return 0;
}
