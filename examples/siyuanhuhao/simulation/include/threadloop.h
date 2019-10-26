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
#include "planner/lanefollow/include/FrenetTrajectoryGenerator.h"
#include "planner/planner.h"
#include "simulator/include/simulator.h"

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
        indicator_socket(0),
        indicator_waypoint(0),
        _planner(_jsonparse.getplannerdata()),
        _estimator(_estimatorRTdata, _jsonparse.getvessel(),
                   _jsonparse.getestimatordata()),
        _simulator(_jsonparse.getsimulatordata(), _jsonparse.getvessel()),
        _trajectorytracking(_jsonparse.getcontrollerdata(), _trackerRTdata),
        _controller(_controllerRTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata()),
        _trajectorygenerator(_jsonparse.getfrenetdata()),
        _sqlite(_jsonparse.getsqlitedata()),
        _tcpserver("9340") {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread socket_thread(&threadloop::socketloop, this);

    // planner_thread.detach();
    // controller_thread.detach();
    // estimator_thread.detach();
    // sql_thread.detach();
    planner_thread.join();
    estimator_thread.join();
    controller_thread.join();
    sql_thread.join();
    socket_thread.join();
  }

 private:
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  planning::plannerRTdata _plannerRTdata{
      0,                        // curvature
      0,                        // speed
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  control::trackerRTdata _trackerRTdata{
      control::TRACKERMODE::STARTED,
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero()   // v_setpoint
  };

  control::controllerRTdata<num_thruster, dim_controlspace> _controllerRTdata{
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
  planning::CartesianState Planning_Marine_state{
      0,           // x
      0,           // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
  };

  int indicator_socket;
  int indicator_waypoint;

  planning::planner _planner;
  estimator<indicator_kalman, 1, 1, 1, 1, 1, 1> _estimator;
  simulator _simulator;
  control::trajectorytracking _trajectorytracking;
  control::controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;

  planning::FrenetTrajectoryGenerator _trajectorygenerator;
  common::database<num_thruster, dim_controlspace> _sqlite;
  tcpserver _tcpserver;

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
    indicator_waypoint = 1;

    while (1) {
      if ((indicator_socket == 1) && (indicator_waypoint == 1)) break;
    }
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

      std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
               Planning_Marine_state.theta, Planning_Marine_state.kappa,
               Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
          common::math::Cart2Marine(
              Plan_cartesianstate.x, Plan_cartesianstate.y,
              Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
              Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);

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
      if ((indicator_socket == 1) && (indicator_waypoint == 1)) break;
    }
    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();
      _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
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

    _estimator.setvalue(0, 0, 0, 0, 0, 0, 0, 1, 0);
    _simulator.setX(_estimatorRTdata.State);

    CLOG(INFO, "GPS") << "initialation successful!";

    while (1) {
      if ((indicator_socket == 1) && (indicator_waypoint == 1)) break;
    }
    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      auto x = _simulator
                   .simulator_onestep(_trackerRTdata.setpoint(2),
                                      _controllerRTdata.BalphaU)
                   .getX();
      _estimator
          .updateestimatedforce(_controllerRTdata.BalphaU,
                                Eigen::Vector3d::Zero())
          .estimatestate(x, _trackerRTdata.setpoint(2));

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
      if ((indicator_socket == 1) && (indicator_waypoint == 1)) break;
    }
    while (1) {
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata, _trackerRTdata);
    }
  }  // sqlloop()

  // socket server
  void socketloop() {
    union socketmsg {
      double double_msg[20];
      char char_msg[160];
    };

    const int recv_size = 10;
    const int send_size = 160;
    char recv_buffer[recv_size];
    socketmsg _sendmsg = {0.0, 0.0, 0.0, 0.0, 0.0};

    common::timecounter timer_socket;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time = 100;

    while (1) {
      outerloop_elapsed_time = timer_socket.timeelapsed();

      for (int i = 0; i != 6; ++i)
        _sendmsg.double_msg[i] = _estimatorRTdata.State(i);  // State

      _sendmsg.double_msg[6] = _plannerRTdata.curvature;      // curvature
      _sendmsg.double_msg[7] = _plannerRTdata.speed;          // speed
      _sendmsg.double_msg[8] = _plannerRTdata.waypoint0(0);   // waypoint0
      _sendmsg.double_msg[9] = _plannerRTdata.waypoint0(1);   // waypoint0
      _sendmsg.double_msg[10] = _plannerRTdata.waypoint1(0);  // waypoint1
      _sendmsg.double_msg[11] = _plannerRTdata.waypoint1(1);  // waypoint1

      for (int i = 0; i != dim_controlspace; ++i) {
        _sendmsg.double_msg[12 + i] = _controllerRTdata.tau(i);  // tau
      }
      for (int i = 0; i != num_thruster; ++i) {
        _sendmsg.double_msg[12 + dim_controlspace + i] =
            _controllerRTdata.rotation(i);  // rotation
      }
      _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                              send_size);

      if (_tcpserver.getconnectioncount() > 0) indicator_socket = 1;

      innerloop_elapsed_time = timer_socket.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "socket") << "Too much time!";
    }
  }  // socketloop()
};   // end threadloop

}  // end namespace ASV

#endif /* _THREADLOOP_H_ */