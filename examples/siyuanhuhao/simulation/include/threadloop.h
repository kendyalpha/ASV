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
#include "spline.h"
#include "timecounter.h"
#include "trajectorytracking.h"

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
        _trajectorytracking(_jsonparse.getcontrollerdata(), _trackerRTdata),
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
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  indicators _indicators{
      0,  // gui_connection
      0,  // joystick_connection
      2,  // indicator_controlmode
      0,  // indicator_windstatus
  };

  planner _planner;
  estimator<indicator_kalman> _estimator;

  trajectorytracking _trajectorytracking;
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

    // trajectory generator
    Eigen::VectorXd X(5);
    Eigen::VectorXd Y(5);

    X << 0.0, 10.0, 20.5, 35.0, 70.5;
    Y << 0.0, -6.0, 5.0, 6.5, 0.0;
    Spline2D target_Spline2D(X, Y);

    Eigen::VectorXd s = target_Spline2D.getarclength();
    double s_max = s.maxCoeff();

    _plannerRTdata.speed = 1;  // desired speed
    _plannerRTdata.waypoint0 << X(0), Y(0);
    _plannerRTdata.waypoint1 << X(0), Y(0);

    double is = 0.0;
    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();

      is += sample_time * _plannerRTdata.speed / 1000.0;
      if (is <= s_max) {
        _plannerRTdata.waypoint0 = _plannerRTdata.waypoint1;
        _plannerRTdata.waypoint1 = target_Spline2D.compute_position(is);
        _plannerRTdata.curvature = target_Spline2D.compute_curvature(is);
      } else {
        CLOG(INFO, "planner") << "Planner reach the last waypoint";
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
      _trackerRTdata =
          _trajectorytracking
              .CircularArcLOS(_plannerRTdata.curvature, _plannerRTdata.speed,
                              _estimatorRTdata.State.head(2),
                              _plannerRTdata.waypoint0,
                              _plannerRTdata.waypoint1)
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
    timecounter timer_estimator;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _estimator.getsampletime());

    _estimator.setvalue(0, 0, 0, 0, 0, 57, 0, 0, 0);
    CLOG(INFO, "GPS") << "initialation successful!";

    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      _estimator
          .updateestimatedforce(_controllerRTdata.BalphaU,
                                Eigen::Vector3d::Zero())
          .estimatestate(_trackerRTdata.setpoint(2));

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

  // loop to save real time data using sqlite3
  void sqlloop() {
    while (1) {
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata, _trackerRTdata);
    }
  }  // sqlloop()
};

#endif /* _THREADLOOP_H_ */