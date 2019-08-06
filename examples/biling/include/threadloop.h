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
#include "gps.h"
#include "guiserver.h"
#include "jsonparse.h"
#include "motorclient.h"
#include "planner.h"
#include "priority.h"
#include "remotecontrol.h"
#include "tcpclient.h"
#include "tcpserver.h"
#include "timecounter.h"
#include "wind.h"              // wind sensors
#include "windcompensation.h"  // wind compensation

constexpr int num_thruster = 6;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = KALMANOFF;
constexpr ACTUATION indicator_actuation = FULLYACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _planner(_jsonparse.getplannerdata()),
        _estimator(_jsonparse.getvessel(), _jsonparse.getestimatordata()),
        _controller(_jsonparse.getcontrollerdata(), _jsonparse.getvessel(),
                    _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata()),
        _gpsimu(51, true, _jsonparse.getgpsbaudrate(), _jsonparse.getgpsport()),
        _guiserver(_jsonparse.getguibaudrate(), _jsonparse.getguiport()),
        _remotecontrol(_jsonparse.getrcbaudrate(),
                       _jsonparse.getremotecontrolport()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    sched_param sch;
    sch.sched_priority = 99;

    std::thread gps_thread(&threadloop::gpsimuloop, this);
    std::thread wind_thread(&threadloop::windloop, this);
    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread guiserver_thread(&threadloop::guicommunicationloop, this);
    std::thread remotecontrol_thread(&threadloop::remotecontrolloop, this);
    std::thread indicators_thread(&threadloop::indicatorsloop, this);
    std::thread lidar_thread(&threadloop::socketlidarloop, this);

    if (pthread_setschedparam(planner_thread.native_handle(), SCHED_FIFO,
                              &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    if (pthread_setschedparam(estimator_thread.native_handle(), SCHED_FIFO,
                              &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    if (pthread_setschedparam(controller_thread.native_handle(), SCHED_FIFO,
                              &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }

    gps_thread.detach();
    wind_thread.detach();
    planner_thread.detach();
    controller_thread.detach();
    estimator_thread.detach();
    sql_thread.detach();
    guiserver_thread.detach();
    remotecontrol_thread.detach();
    indicators_thread.detach();
    lidar_thread.detach();
  }

 private:
  // json
  jsonparse<num_thruster, dim_controlspace> _jsonparse;

  plannerRTdata _plannerRTdata{
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero(),  // v_setpoint
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  controllerRTdata<num_thruster, dim_controlspace> _controllerRTdata{
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // alpha_deg
  };

  motorRTdata<num_thruster> _motorRTdata;
  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero(),              // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero(),  // motiondata_6dof
      Eigen::Vector3d::Zero()               // windload
  };

  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,                // date
      0,                // time
      0,                // heading
      0,                // pitch
      0,                // roll
      0,                // latitude
      0,                // longitude
      0,                // altitude
      0,                // Ve
      0,                // Vn
      0,                // Vu
      0,                // base_line
      0,                // NSV1
      0,                // NSV2
      'a',              // status
      {'a', 'b', '0'},  // check
      0,                // UTM_x
      0                 // UTM_y
  };
  // real time wind data
  windRTdata _windRTdata{
      0,  // speed
      0   // orientation
  };

  // real time remote control data
  recontrolRTdata _recontrolRTdata{
      0,    // controlmode
      0,    // joystick_connection
      0.0,  // right_joystick_LR
      0.0,  // right_joystick_UD
      0.0,  // left_joystick_UD
      0.0,  // left_joystick_LR
      0.0,  // SA
      0.0,  // SB
      0.0,  // SC
      0.0   // SD
  };

  // real time gui-link data
  guilinkRTdata _guilinkRTdata{
      0,                                   // gui_connection
      0,                                   // indicator_autocontrolmode
      0,                                   // indicator_windstatus
      Eigen::Vector3d::Zero(),             // setpoints
      Eigen::Vector2d::Zero(),             // startingpoint
      Eigen::Vector2d::Zero(),             // endingpoint
      Eigen::Matrix<double, 2, 8>::Zero()  // waypoints
  };

  indicators _indicators{
      0,  // gui_connection
      0,  // joystick_connection
      0,  // indicator_controlmode
      0,  // indicator_windstatus
  };

  planner _planner;
  estimator<indicator_kalman> _estimator;
  controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;
  motorclient _motorclient;

  // sensors
  gpsimu _gpsimu;
  windcompensation _windcompensation;
  guiserver<num_thruster, dim_controlspace> _guiserver;
  remotecontrol _remotecontrol;

  database<num_thruster, dim_controlspace> _sqlite;

  void intializethreadloop() {
    _controller.initializecontroller(_controllerRTdata);
    _sqlite.initializetables();
  }

  void plannerloop() {
    // timecounter timer_planner;
    // long int elapsed_time = 0;
    // long int sample_time =
    //     static_cast<long int>(1000 * _planner.getsampletime());

    // double radius = 2;
    // Eigen::Vector2d startposition = (Eigen::Vector2d() << 2, 0.1).finished();
    // Eigen::Vector2d endposition = (Eigen::Vector2d() << 5, 2).finished();
    // _planner.setconstantspeed(_plannerRTdata, 0.1, 0.06);

    // auto waypoints = _planner.followcircle(startposition, endposition,
    // radius,
    //                                        _estimatorRTdata.State(2),
    //                                        _plannerRTdata.v_setpoint(0));
    // _planner.initializewaypoint(_plannerRTdata, waypoints);

    // int index_wpt = 2;
    // while (1) {
    //   if (_planner.switchwaypoint(_plannerRTdata,
    //                               _estimatorRTdata.State.head(2),
    //                               waypoints.col(index_wpt))) {
    //     std::cout << index_wpt << std::endl;
    //     ++index_wpt;
    //   }
    //   if (index_wpt == waypoints.cols()) {
    //     CLOG(INFO, "waypoints") << "reach the last waypoint!";
    //     break;
    //   }
    //   _planner.pathfollowLOS(_plannerRTdata, _estimatorRTdata.State.head(2));

    //   elapsed_time = timer_planner.timeelapsed();
    //   std::this_thread::sleep_for(
    //       std::chrono::milliseconds(sample_time - elapsed_time));
    // }

    timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _planner.getsampletime());
    // Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(2, 4);
    // waypoints.col(0) << 3433875, 351046;
    // waypoints.col(1) << 3433895, 351058;
    // waypoints.col(2) << 3433892, 351063;

    // _plannerRTdata.waypoint0 = waypoints.col(0);
    // _plannerRTdata.waypoint1 = waypoints.col(1);
    int index_wpt = 2;
    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();

      switch (_indicators.indicator_controlmode) {  // controller mode
        case 2:
          _plannerRTdata.waypoint0 << 3433875, 351046;
          _plannerRTdata.waypoint1 << 3433895, 351058;
          _plannerRTdata.v_setpoint << 1, 0, 0;

          // if (index_wpt == 2) {
          //   _plannerRTdata.waypoint0 = _guilinkRTdata.waypoints.col(0);
          //   _plannerRTdata.waypoint1 = _guilinkRTdata.waypoints.col(1);
          //   _plannerRTdata.v_setpoint << 1, 0, 0;
          // }

          // if (index_wpt < 8) {
          //   if (_planner.switchwaypoint(
          //           _plannerRTdata, _estimatorRTdata.State.head(2),
          //           _guilinkRTdata.waypoints.col(index_wpt)))
          //     ++index_wpt;
          // }
          _planner.pathfollowLOS(_plannerRTdata,
                                 _estimatorRTdata.State.head(2));

          break;
        case 3:
          _plannerRTdata.setpoint = _guilinkRTdata.setpoints;
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

  // GPS/IMU
  void gpsimuloop() {
    try {
      while (1) {
        gps_data = _gpsimu.gpsonestep().getgpsRTdata();
      }

    } catch (std::exception& e) {
      CLOG(ERROR, "GPS serial") << e.what();
    }
  }  // gpsimuloop()

  void controllerloop() {
    timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    _motorclient.startup_socket_client(_motorRTdata);
    CLOG(INFO, "PLC") << "Servo and PLC initialation successful!";

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();
      _controller.setcontrolmode(_indicators.indicator_controlmode);
      _controller.controlleronestep(
          _controllerRTdata, _estimatorRTdata.windload,
          _estimatorRTdata.p_error, _estimatorRTdata.v_error,
          _plannerRTdata.command, _plannerRTdata.v_setpoint);

      _motorclient.commandfromcontroller(
          _motorRTdata.command_alpha, _motorRTdata.command_rotation,
          _controllerRTdata.alpha_deg, _controllerRTdata.rotation);

      _motorclient.PLCcommunication(_motorRTdata);

      std::cout << "controlmode:" << _indicators.indicator_controlmode
                << std::endl;
      std::cout << "tau:" << _controllerRTdata.tau << std::endl;

      std::cout << "p_error " << _estimatorRTdata.p_error << std::endl;
      std::cout << "windstatus: " << _indicators.indicator_windstatus
                << std::endl;
      std::cout << "wind speed: " << _windRTdata.speed << std::endl;
      std::cout << "wind orientation: " << _windRTdata.orientation << std::endl;
      std::cout << "windload " << _estimatorRTdata.windload << std::endl;

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

    while (1) {
      if ((gps_data.status == 'B') || (gps_data.status == '4')) {
        _estimator.setvalue(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                            gps_data.altitude, gps_data.roll, gps_data.pitch,
                            gps_data.heading, gps_data.Ve, gps_data.Vn);
        _windcompensation.setvalue(_windRTdata.speed, _windRTdata.orientation);
        CLOG(INFO, "GPS") << "initialation successful!";
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      _windcompensation.setwindstatus(_indicators.indicator_windstatus);
      _estimatorRTdata.windload =
          _windcompensation
              .computewindload(_windRTdata.speed, _windRTdata.orientation)
              .getwindload();
      _estimator.updateestimatedforce(_estimatorRTdata,
                                      _controllerRTdata.BalphaU,
                                      _estimatorRTdata.windload);
      _estimator.estimatestate(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                               gps_data.altitude, gps_data.roll, gps_data.pitch,
                               gps_data.heading, gps_data.Ve, gps_data.Vn,
                               _plannerRTdata.setpoint(2));
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
      _sqlite.update_gps_table(gps_data);
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata);
      _sqlite.update_indicators_table(_indicators);
      _sqlite.update_wind_table(_windRTdata);
    }
  }  // sqlloop()

  void guicommunicationloop() {
    while (1) {
      _guiserver.guicommunication(_guilinkRTdata, _indicators, _estimatorRTdata,
                                  _plannerRTdata, gps_data, _motorRTdata,
                                  _windRTdata);
    }
  }  // guicommunicationloop()

  void remotecontrolloop() {
    std::vector<pidcontrollerdata> _piddata = _jsonparse.getpiddata();
    Eigen::Matrix<double, 3, 2> command_limit =
        Eigen::Matrix<double, 3, 2>::Zero();
    for (int i = 0; i != 3; ++i) {
      command_limit(i, 0) = _piddata[i].max_output;
      command_limit(i, 1) = _piddata[i].min_output;
    }

    while (1) {
      _recontrolRTdata.joystick_connection = _remotecontrol.checkrcconnection();
      _remotecontrol.readserialdata(_recontrolRTdata);
      _remotecontrol.parsercdata(_plannerRTdata.command, _recontrolRTdata,
                                 command_limit);
    }

  }  // remotecontrolloop()

  void indicatorsloop() {
    while (1) {
      _indicators.joystick_connection = _recontrolRTdata.joystick_connection;
      _indicators.gui_connection = _guilinkRTdata.gui_connection;
      if (_recontrolRTdata.controlmode == 0) {
        _indicators.indicator_controlmode = 0;  // manual mode
        _indicators.indicator_windstatus = 0;   // wind off
      } else {
        if (_indicators.gui_connection == 1) {  // automatic mode
          _indicators.indicator_controlmode =
              _guilinkRTdata.indicator_autocontrolmode;
          _indicators.indicator_windstatus =
              _guilinkRTdata.indicator_windstatus;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }  // indicatorsloop()

  // real time read serial data(speed / orientation)
  void windloop() {
    wind _wind(_jsonparse.getwindbaudrate(), _jsonparse.getwindport());
    while (1) {
      _windRTdata = _wind.readwind().getwindRTdata();
      // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }  // windloop

  // real time socket server for lidar
  void socketlidarloop() {
    union lidarmsg {
      double double_msg[3];
      char char_msg[24];
    };
    tcpserver _tcpserver("9341");
    const int recv_size = 10;
    const int send_size = 24;
    char recv_buffer[recv_size];
    lidarmsg _sendmsg = {0.0, 0.0, 0.0};
    while (1) {
      // State
      _sendmsg.double_msg[0] = gps_data.latitude;
      _sendmsg.double_msg[1] = gps_data.longitude;
      _sendmsg.double_msg[2] = _estimatorRTdata.State(2);

      _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                              send_size);

      // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }  // windloop
};

#endif /* _THREADLOOP_H_ */
