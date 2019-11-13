/*
***********************************************************************
* threadloop.h: thread-based DP controller and network
* function to run the whole loop on server (including TCP/IP server,
* senser, estimator, controller, planner, database, etc).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _THREADLOOP_H_
#define _THREADLOOP_H_

#include <pthread.h>
#include <chrono>
#include <thread>

#include "common/communication/include/tcpserver.h"
#include "common/fileIO/include/database.h"
#include "common/fileIO/include/jsonparse.h"
#include "common/logging/include/easylogging++.h"
#include "common/property/include/priority.h"
#include "common/timer/include/timecounter.h"
#include "controller/include/controller.h"
#include "controller/include/trajectorytracking.h"
#include "estimator/include/estimator.h"
#include "messages/sensors/gpsimu/include/gps.h"
#include "messages/stm32/include/stm32_link.h"
#include "planner/lanefollow/include/FrenetTrajectoryGenerator.h"
#include "planner/planner.h"

namespace ASV {

constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = USEKALMAN::KALMANOFF;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _gpsimu(gps_data, _jsonparse.getgpsbaudrate(), _jsonparse.getgpsport()),
        _estimator(_estimatorRTdata, _jsonparse.getvessel(),
                   _jsonparse.getestimatordata()),
        _stm32_link(_stm32data, _jsonparse.getstm32baudrate(),
                    _jsonparse.getstm32port()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread gps_thread(&threadloop::gpsloop, this);
    std::thread stm32_thread(&threadloop::stm32loop, this);

    // planner_thread.detach();
    // controller_thread.detach();
    // estimator_thread.detach();
    // sql_thread.detach();
    estimator_thread.join();
    sql_thread.join();
    gps_thread.join();
    stm32_thread.join();
  }

 private:
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Marine_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  // real time GPS/IMU data
  messages::gpsRTdata gps_data{
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

  messages::stm32data _stm32data{
      "",                              // UTC_time
      0,                               // command_n1
      -10,                             // command_n2
      0,                               // feedback_n1
      0,                               // feedback_n2
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

  messages::GPS _gpsimu;

  estimator<indicator_kalman, 1, 1, 1, 1, 1, 1> _estimator;

  messages::stm32_link _stm32_link;

  common::database<num_thruster, dim_controlspace> _sqlite;

  common::timecounter utc_timer;

  void intializethreadloop() { _sqlite.initializetables(); }

  // loop to give real time state estimation
  void estimatorloop() {
    common::timecounter timer_estimator;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _estimator.getsampletime());

    while (1) {
      if (gps_data.status >= 1) {
        _estimator.setvalue(gps_data.UTM_x,     // gps_x
                            gps_data.UTM_y,     // gps_y
                            gps_data.altitude,  // gps_z
                            gps_data.roll,      // gps_roll
                            gps_data.pitch,     // gps_pitch
                            gps_data.heading,   // gps_heading
                            gps_data.Ve,        // gps_Ve
                            gps_data.Vn,        // gps_Vn
                            gps_data.roti       // gps_roti
        );

        CLOG(INFO, "GPS") << "initialation successful!";
        break;
      }
    }
    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      _estimatorRTdata = _estimator
                             .updateestimatedforce(Eigen::Vector3d::Zero(),
                                                   Eigen::Vector3d::Zero())
                             .estimatestate(gps_data.UTM_x,     // gps_x
                                            gps_data.UTM_y,     // gps_y
                                            gps_data.altitude,  // gps_z
                                            gps_data.roll,      // gps_roll
                                            gps_data.pitch,     // gps_pitch
                                            gps_data.heading,   // gps_heading
                                            gps_data.Ve,        // gps_Ve
                                            gps_data.Vn,        // gps_Vn
                                            gps_data.roti,      // gps_roti
                                            0                   //_dheading
                                            )
                             .getEstimatorRTData();

      innerloop_elapsed_time = timer_estimator.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "estimator") << "Too much time!";
    }

  }  // estimatorloop()

  // loop to save real time data using sqlite3 and modern_sqlite3_cpp_wrapper
  void sqlloop() {
    while (1) {
      _sqlite.update_gps_table(gps_data);
      _sqlite.update_stm32_table(_stm32data);
      _sqlite.update_estimator_table(_estimatorRTdata);
    }
  }  // sqlloop()

  // loop to give messages to stm32
  void stm32loop() {
    while (1) {
      _stm32_link.setstm32data(_stm32data).stm32onestep();
      _stm32data = _stm32_link.getstmdata();
    }
  }  // stm32loop()

  // read gps data and convert it to UTM
  void gpsloop() {
    while (1) {
      gps_data = _gpsimu.gpsonestep().getgpsRTdata();
    }
  }  // gpsloop()

};  // end threadloop

}  // end namespace ASV

#endif /* _THREADLOOP_H_ */