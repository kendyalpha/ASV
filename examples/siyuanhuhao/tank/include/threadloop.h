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
constexpr ACTUATION indicator_actuation = ACTUATION::UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _planner(_jsonparse.getplannerdata()),
        _gpsimu(gps_data, _jsonparse.getgpsbaudrate(), _jsonparse.getgpsport()),
        _estimator(_estimatorRTdata, _jsonparse.getvessel(),
                   _jsonparse.getestimatordata()),
        _trajectorytracking(_jsonparse.getcontrollerdata(), _trackerRTdata),
        _controller(_controllerRTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata()),
        _stm32_link(_stm32data, _jsonparse.getstm32baudrate(),
                    _jsonparse.getstm32port()),
        _trajectorygenerator(_jsonparse.getfrenetdata()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread gps_thread(&threadloop::gpsloop, this);
    std::thread stm32_thread(&threadloop::stm32loop, this);

    // planner_thread.detach();
    // controller_thread.detach();
    // estimator_thread.detach();
    // sql_thread.detach();
    planner_thread.join();
    estimator_thread.join();
    controller_thread.join();
    sql_thread.join();
    gps_thread.join();
    stm32_thread.join();
  }

 private:
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  plannerRTdata _plannerRTdata{
      0,                        // curvature
      0,                        // speed
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  trackerRTdata _trackerRTdata{
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero()   // v_setpoint
  };

  controllerRTdata<num_thruster, dim_controlspace> _controllerRTdata{
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // alpha_deg
  };

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
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

  // real time data
  CartesianState Planning_Marine_state{
      0,           // x
      0,           // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
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
      0,                               // RC_X
      0,                               // RC_Y
      0,                               // RC_Mz
      0,                               // voltage_b1
      0,                               // voltage_b2
      0,                               // voltage_b2
      messages::STM32STATUS::STANDBY,  // stm32status
      common::LINKSTATUS::CONNECTED    // linkstatus;
  };

  planner _planner;

  messages::GPS _gpsimu;

  estimator<indicator_kalman, 1, 1, 1, 1, 1, 1> _estimator;
  trajectorytracking _trajectorytracking;
  controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;

  messages::stm32_link _stm32_link;

  FrenetTrajectoryGenerator _trajectorygenerator;
  common::database<num_thruster, dim_controlspace> _sqlite;

  common::timecounter utc_timer;

  void intializethreadloop() {
    _controllerRTdata =
        _controller.initializecontroller().getcontrollerRTdata();
    _sqlite.initializetables();
  }

  void plannerloop() {
    common::timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * _planner.getsampletime());

    // trajectory generator
    Eigen::VectorXd X(5);
    Eigen::VectorXd Y(5);

    X << 0.0, 10.0, 20.5, 35.0, 70.5;
    Y << 0.0, 0, 5.0, 6.5, 0.0;

    _trajectorygenerator.regenerate_target_course(X, Y);

    sqlite::database db(
        "/home/scar1et/Coding/ASV/examples/siyuanhuhao/simulation/data/wp.db");
    std::string str =
        "CREATE TABLE WP"
        "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
        " X           DOUBLE, "
        " Y           DOUBLE); ";
    db << str;

    auto CartRefX = _trajectorygenerator.getCartRefX();
    auto CartRefY = _trajectorygenerator.getCartRefY();

    for (int i = 0; i != CartRefX.size(); i++) {
      // save to sqlite
      std::string str =
          "INSERT INTO WP"
          "(X, Y) VAlUES(" +
          std::to_string(CartRefX(i)) + ", " + std::to_string(-CartRefY(i)) +
          ");";
      db << str;
    }
    CLOG(INFO, "waypoints") << "Waypoints have been generated";

    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();

      auto Plan_cartesianstate =
          _trajectorygenerator
              .trajectoryonestep(_estimatorRTdata.Marine_state(0),
                                 _estimatorRTdata.Marine_state(1),
                                 _estimatorRTdata.Marine_state(2),
                                 _estimatorRTdata.Marine_state(3),
                                 _estimatorRTdata.Marine_state(4),
                                 _estimatorRTdata.Marine_state(5), 2)
              .getnextcartesianstate();
      Planning_Marine_state.x = Plan_cartesianstate.x;
      Planning_Marine_state.speed = Plan_cartesianstate.speed;
      Planning_Marine_state.dspeed = Plan_cartesianstate.dspeed;

      std::tie(Planning_Marine_state.y, Planning_Marine_state.theta,
               Planning_Marine_state.kappa) =
          common::math::Cart2Marine(Plan_cartesianstate.y,
                                    Plan_cartesianstate.theta,
                                    Plan_cartesianstate.kappa);

      innerloop_elapsed_time = timer_planner.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time_ms - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time_ms)
        CLOG(INFO, "planner") << "Too much time!";
    }

  }  // plannerloop

  void controllerloop() {
    common::timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();
      _controller.setcontrolmode(CONTROLMODE::MANEUVERING);
      // trajectory tracking
      _trackerRTdata = _trajectorytracking
                           .CircularArcLOS(Planning_Marine_state.kappa,
                                           Planning_Marine_state.speed,
                                           Planning_Marine_state.theta)
                           .gettrackerRTdata();
      // controller
      _controllerRTdata = _controller
                              .controlleronestep(Eigen::Vector3d::Zero(),
                                                 _estimatorRTdata.p_error,
                                                 _estimatorRTdata.v_error,
                                                 _plannerRTdata.command,
                                                 _trackerRTdata.v_setpoint)
                              .getcontrollerRTdata();
      // std::cout << elapsed_time << std::endl;
      innerloop_elapsed_time = timer_controler.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "controller") << "Too much time!";
    }
  }  // controllerloop

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

      _estimator
          .updateestimatedforce(_controllerRTdata.BalphaU,
                                Eigen::Vector3d::Zero())
          .estimatestate(gps_data.UTM_x,             // gps_x
                         gps_data.UTM_y,             // gps_y
                         gps_data.altitude,          // gps_z
                         gps_data.roll,              // gps_roll
                         gps_data.pitch,             // gps_pitch
                         gps_data.heading,           // gps_heading
                         gps_data.Ve,                // gps_Ve
                         gps_data.Vn,                // gps_Vn
                         gps_data.roti,              // gps_roti
                         _trackerRTdata.setpoint(2)  //_dheading
          );

      _estimatorRTdata =
          _estimator
              .estimateerror(_trackerRTdata.setpoint, _trackerRTdata.v_setpoint)
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
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata, _trackerRTdata);
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