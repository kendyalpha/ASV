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
#include "controller.h"
#include "database.h"
#include "easylogging++.h"
#include "estimator.h"
#include "jsonparse.h"
#include "planner.h"
#include "priority.h"
#include "timecounter.h"

constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = USEKALMAN::KALMANON;
constexpr ACTUATION indicator_actuation = ACTUATION::UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _planner(_jsonparse.getplannerdata()),
        _estimator(_estimatorRTdata, _jsonparse.getvessel(),
                   _jsonparse.getestimatordata()),
        _trajectroytracking(_jsonparse.getcontrollerdata(), _trackerRTdata),
        _controller(_controllerRTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    sched_param sch;
    sch.sched_priority = 99;

    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);

    if (pthread_setschedparam(planner_thread.native_handle(), SCHED_RR, &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    if (pthread_setschedparam(estimator_thread.native_handle(), SCHED_RR,
                              &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    if (pthread_setschedparam(controller_thread.native_handle(), SCHED_RR,
                              &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    if (pthread_setschedparam(sql_thread.native_handle(), SCHED_RR, &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }

    planner_thread.detach();
    controller_thread.detach();
    estimator_thread.detach();
    sql_thread.detach();
  }

 private:
  // json
  jsonparse<num_thruster, dim_controlspace> _jsonparse;

  plannerRTdata _plannerRTdata{
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
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero(),              // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero()   // motiondata_6dof
  };

  indicators _indicators{
      0,  // gui_connection
      0,  // joystick_connection
      2,  // indicator_controlmode
      0,  // indicator_windstatus
  };

  planner _planner;
  estimator<indicator_kalman> _estimator;

  trajectroytracking _trajectroytracking;
  controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;
  database<num_thruster, dim_controlspace> _sqlite;

  void intializethreadloop() {
    _controllerRTdata =
        _controller.initializecontroller().getcontrollerRTdata();
    _sqlite.initializetables();
  }

  void plannerloop() {
    timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _planner.getsampletime());
    Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(2, 4);
    waypoints.col(0) << 3433875, 351046;
    waypoints.col(1) << 3433899, 351060;
    waypoints.col(2) << 3433899, 351075;
    waypoints.col(3) << 3433875, 351075;
    _plannerRTdata.waypoint0 = waypoints.col(0);
    _plannerRTdata.waypoint1 = waypoints.col(1);
    int index_wpt = 2;
    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();
      switch (_indicators.indicator_controlmode) {  // controller mode
        case 2:
          _plannerRTdata.v_setpoint << 1, 0, 0;

          if (_planner.switchwaypoint(_plannerRTdata,
                                      _estimatorRTdata.State.head(2),
                                      waypoints.col(index_wpt))) {
            std::cout << index_wpt << std::endl;
            ++index_wpt;
          }
          if (index_wpt == waypoints.cols()) {
            CLOG(INFO, "waypoints") << "reach the last waypoint!";
            break;
          }
          _planner.pathfollowLOS(_plannerRTdata,
                                 _estimatorRTdata.State.head(2));

          break;
        case 3:
          _plannerRTdata.setpoint << 3433877.3, 351034, M_PI / 4;
          _plannerRTdata.v_setpoint << 0, 0, 0;
          break;
        default:
          break;
      }

      innerloop_elapsed_time = timer_planner.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "planner") << "Too much time!";
    }

  }  // plannerloop

  void controllerloop() {
    timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();
      _controller.setcontrolmode(CONTROLMODE::MANEUVERING);
      // trajectory tracking

      //
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
    timecounter timer_estimator;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _estimator.getsampletime());

    _estimator.setvalue(_estimatorRTdata, 351045.7, 3433883.219, 0, 0, 0, 57, 0,
                        0);
    CLOG(INFO, "GPS") << "initialation successful!";

    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      _estimator.updateestimatedforce(
          _estimatorRTdata, _controllerRTdata.BalphaU,
          _windcompensation.computewindload(0, 0).getwindload());
      _estimator.estimatestate(_estimatorRTdata, _plannerRTdata.setpoint(2));
      _estimator.estimateerror(_estimatorRTdata, _plannerRTdata.setpoint,
                               _plannerRTdata.v_setpoint);

      innerloop_elapsed_time = timer_estimator.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "estimator") << "Too much time!";
    }

  }  // estimatorloop()

  // loop to save real time data using sqlite3
  void sqlloop() {
    while (1) {
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata);
    }
  }  // sqlloop()
};

#endif /* _THREADLOOP_H_ */