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
#include "messages/GUILink/include/guilink.h"
#include "messages/sensors/gpsimu/include/gps.h"
#include "messages/stm32/include/stm32_link.h"
#include "planner/lanefollow/include/FrenetTrajectoryGenerator.h"
#include "planner/planner.h"
#include "simulator/include/simulator.h"

namespace ASV {

// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_FRENET;
constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_FRENET;

constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = USEKALMAN::KALMANOFF;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _trajectorygenerator(_jsonparse.getfrenetdata()),
        _trajectorytracking(_jsonparse.getcontrollerdata(), tracker_RTdata),
        indicator_waypoint(0),
        _tcpserver("9340") {}
  ~threadloop() {}

  void mainloop() {
    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread gps_thread(&threadloop::gps_loop, this);
    std::thread timer_thread(&threadloop::utc_timer_loop, this);
    std::thread gui_thread(&threadloop::gui_loop, this);
    std::thread stm32_thread(&threadloop::stm32loop, this);
    std::thread socket_thread(&threadloop::socket_loop, this);

    planner_thread.join();
    estimator_thread.join();
    controller_thread.join();
    sql_thread.join();
    gps_thread.join();
    timer_thread.join();
    gui_thread.join();
    stm32_thread.join();
    socket_thread.join();
  }

 private:
  /********************* Real time Data  *********************/
  planning::plannerRTdata planner_RTdata{
      0,                        // curvature
      0,                        // speed
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  // real time data of tracker
  control::trackerRTdata tracker_RTdata{
      control::TRACKERMODE::STARTED,  // trackermode
      Eigen::Vector3d::Zero(),        // setpoint
      Eigen::Vector3d::Zero()         // v_setpoint
  };

  // real time data of controller
  control::controllerRTdata<num_thruster, dim_controlspace> controller_RTdata{
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // alpha_deg
  };

  // realtime parameters of the estimators
  estimatorRTdata estimator_RTdata{
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
  // real time stm32 data
  messages::stm32data stm32_data{
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
      0,                               // voltage_b3
      messages::STM32STATUS::STANDBY,  // feedback_stm32status
      messages::STM32STATUS::STANDBY,  // command_stm32status
      common::LINKSTATUS::CONNECTED    // linkstatus;
  };

  // real time gui-link data
  messages::guilinkRTdata<num_thruster, 3> guilink_RTdata{
      "",                                           // UTC_time
      messages::GUISTATUS::STANDBY,                 // guistutus_PC2gui
      messages::GUISTATUS::STANDBY,                 // guistutus_gui2PC
      common::LINKSTATUS::DISCONNECTED,             // linkstatus
      0,                                            // indicator_autocontrolmode
      0,                                            // indicator_windstatus
      0.0,                                          // latitude
      0.0,                                          // longitude
      Eigen::Matrix<double, 6, 1>::Zero(),          // State
      0.0,                                          // roll
      0.0,                                          // pitch
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_rotation
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<double, 3, 1>::Zero(),          // battery_voltage
      Eigen::Vector3d::Zero(),                      // setpoints
      0.0,                                          // desired_speed
      Eigen::Vector2d::Zero(),                      // startingpoint
      Eigen::Vector2d::Zero(),                      // endingpoint
      Eigen::Matrix<double, 2, 8>::Zero()           // waypoints
  };

  // real time utc
  std::string pt_utc;

  /********************* Modules  *********************/
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  planning::FrenetTrajectoryGenerator _trajectorygenerator;
  control::trajectorytracking _trajectorytracking;

  int indicator_waypoint;

  tcpserver _tcpserver;

  void generate_waypoints() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP: {
        break;
      }
      case common::TESTMODE::SIMULATION_LOS: {
        // trajectory generator
        Eigen::VectorXd WX(5);
        Eigen::VectorXd WY(5);

        WX << 0.0, 10.0, 20.5, 35.0, 70.5;
        WY << 0.0, 0, 5.0, 6.5, 0.0;

        _trajectorytracking.set_grid_points(WX, WY);

        planner_RTdata.speed = 2;

        sqlite::database db("./../../data/wp.db");
        std::string str =
            "CREATE TABLE WP"
            "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
            " X           DOUBLE, "
            " Y           DOUBLE); ";
        db << str;

        for (int i = 0; i != WX.size(); i++) {
          // save to sqlite
          std::string str =
              "INSERT INTO WP"
              "(X, Y) VAlUES(" +
              std::to_string(WX(i)) + ", " + std::to_string(WY(i)) + ");";
          db << str;
        }
        CLOG(INFO, "waypoints") << "Waypoints have been generated";
        indicator_waypoint = 1;

        break;
      }
      case common::TESTMODE::SIMULATION_FRENET: {
        // trajectory generator
        Eigen::VectorXd WX(5);
        Eigen::VectorXd WY(5);

        WX << 0.0, 10.0, 20.5, 35.0, 70.5;
        WY << 0.0, 0, 5.0, 6.5, 0.0;

        _trajectorygenerator.regenerate_target_course(WX, WY);

        sqlite::database db("./../../data/wp.db");
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
              std::to_string(CartRefX(i)) + ", " +
              std::to_string(-CartRefY(i)) + ");";
          db << str;
        }
        CLOG(INFO, "waypoints") << "Waypoints have been generated";
        indicator_waypoint = 1;

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP: {
        break;
      }
      case common::TESTMODE::EXPERIMENT_LOS: {
        // trajectory generator
        Eigen::VectorXd WX(5);
        Eigen::VectorXd WY(5);

        WX << 0.0, 10.0, 20.5, 35.0, 70.5;
        WY << 0.0, 0, 5.0, 6.5, 0.0;

        _trajectorytracking.set_grid_points(WX, WY);

        planner_RTdata.speed = 2;
        indicator_waypoint = 1;

        break;
      }
      case common::TESTMODE::EXPERIMENT_FRENET: {
        break;
      }
      default:
        break;
    }  // end switch
  }

  void plannerloop() {
    generate_waypoints();

    planning::planner _planner(_jsonparse.getplannerdata());

    common::timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * _planner.getsampletime());

    while (1) {
      if (indicator_waypoint == 1) break;
    }
    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP: {
          break;
        }
        case common::TESTMODE::SIMULATION_LOS: {
          break;
        }
        case common::TESTMODE::SIMULATION_FRENET: {
          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5), 2)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP: {
          break;
        }
        case common::TESTMODE::EXPERIMENT_LOS: {
          break;
        }
        case common::TESTMODE::EXPERIMENT_FRENET: {
          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5), 2)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        default:
          break;
      }  // end switch

      innerloop_elapsed_time = timer_planner.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time_ms - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time_ms)
        CLOG(INFO, "planner") << "Too much time!";
    }

  }  // plannerloop

  void controllerloop() {
    control::controller<10, num_thruster, indicator_actuation, dim_controlspace>
        _controller(controller_RTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata());

    common::timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    while (1) {
      if (indicator_waypoint == 1) break;
    }

    controller_RTdata =
        _controller.initializecontroller().getcontrollerRTdata();

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          tracker_RTdata.setpoint = Eigen::Vector3d::Zero();
          tracker_RTdata.v_setpoint = Eigen::Vector3d::Zero();

          break;
        }
        case common::TESTMODE::SIMULATION_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          // trajectory tracking
          _trajectorytracking.Grid_LOS(planner_RTdata.speed,
                                       estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::SIMULATION_FRENET: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .CircularArcLOS(Planning_Marine_state.kappa,
                                               Planning_Marine_state.speed,
                                               Planning_Marine_state.theta)
                               .gettrackerRTdata();
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP: {
          break;
        }
        case common::TESTMODE::EXPERIMENT_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          // trajectory tracking
          _trajectorytracking.Grid_LOS(planner_RTdata.speed,
                                       estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::EXPERIMENT_FRENET: {
          break;
        }
        default:
          break;
      }  // end switch

      // controller
      controller_RTdata = _controller
                              .controlleronestep(Eigen::Vector3d::Zero(),
                                                 estimator_RTdata.p_error,
                                                 estimator_RTdata.v_error,
                                                 planner_RTdata.command,
                                                 tracker_RTdata.v_setpoint)
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
    estimator<indicator_kalman, 1, 1, 1, 1, 1, 1> _estimator(
        estimator_RTdata, _jsonparse.getvessel(),
        _jsonparse.getestimatordata());

    common::timecounter timer_estimator;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _estimator.getsampletime());

    while (1) {
      if (indicator_waypoint == 1) break;
    }

    simulator _simulator(_jsonparse.getsimulatordata(), _jsonparse.getvessel());

    // initialization of estimator
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        // simulation
        _estimator.setvalue(0, 0, 0, 0, 0, 0, 0, 1, 0);
        _simulator.setX(estimator_RTdata.State);
        CLOG(INFO, "GPS") << "initialation successful!";
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // experiment
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
        break;
      }
      default:
        break;
    }

    // real time calculation in estimator
    while (1) {
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::SIMULATION_FRENET: {
          // simulation
          auto x = _simulator
                       .simulator_onestep(tracker_RTdata.setpoint(2),
                                          controller_RTdata.BalphaU)
                       .getX();
          _estimator
              .updateestimatedforce(controller_RTdata.BalphaU,
                                    Eigen::Vector3d::Zero())
              .estimatestate(x, tracker_RTdata.setpoint(2));
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP:
        case common::TESTMODE::EXPERIMENT_LOS:
        case common::TESTMODE::EXPERIMENT_FRENET: {
          // experiment
          _estimator
              .updateestimatedforce(controller_RTdata.BalphaU,
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
                             tracker_RTdata.setpoint(2)  //_dheading
              );
          break;
        }
        default:
          break;
      }  // end switch

      estimator_RTdata =
          _estimator
              .estimateerror(tracker_RTdata.setpoint, tracker_RTdata.v_setpoint)
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
      if (indicator_waypoint == 1) break;
    }

    common::database<num_thruster, dim_controlspace> _sqlite(
        _jsonparse.getsqlitedata());
    _sqlite.initializetables();

    while (1) {
      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::SIMULATION_FRENET: {
          // simulation
          _sqlite.update_planner_table(planner_RTdata);
          _sqlite.update_estimator_table(estimator_RTdata);
          _sqlite.update_controller_table(controller_RTdata, tracker_RTdata);
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP:
        case common::TESTMODE::EXPERIMENT_LOS:
        case common::TESTMODE::EXPERIMENT_FRENET: {
          // experiment
          _sqlite.update_gps_table(gps_data);
          _sqlite.update_stm32_table(stm32_data);
          _sqlite.update_planner_table(planner_RTdata);
          _sqlite.update_estimator_table(estimator_RTdata);
          _sqlite.update_controller_table(controller_RTdata, tracker_RTdata);

          break;
        }
        default:
          break;
      }  // end switch
    }
  }  // sqlloop()

  // loop to give messages to stm32
  void stm32loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // experiment
        messages::stm32_link _stm32_link(stm32_data,
                                         _jsonparse.getstm32baudrate(),
                                         _jsonparse.getstm32port());
        while (1) {
          messages::STM32STATUS _command_stm32 =
              static_cast<messages::STM32STATUS>(
                  guilink_RTdata.guistutus_gui2PC);
          _stm32_link
              .setstm32data(_command_stm32, pt_utc, controller_RTdata.u,
                            controller_RTdata.alpha)
              .stm32onestep();
          stm32_data = _stm32_link.getstmdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // stm32loop()

  // read gps data and convert it to UTM
  void gps_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        messages::GPS _gpsimu(gps_data, _jsonparse.getgpsbaudrate(),
                              _jsonparse.getgpsport());

        // experiment
        while (1) {
          gps_data = _gpsimu.gpsonestep().getgpsRTdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // gps_loop()

  // gui data link
  void gui_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        messages::guilink_serial<num_thruster, 3, dim_controlspace> _gui_link(
            guilink_RTdata, _jsonparse.getguibaudrate(),
            _jsonparse.getguiport());

        // experiment
        while (1) {
          Eigen::Vector3d batteries =
              (Eigen::Vector3d() << stm32_data.voltage_b1,
               stm32_data.voltage_b2, stm32_data.voltage_b3)
                  .finished();
          Eigen::Vector2i feedback_pwm =
              (Eigen::Vector2i() << stm32_data.feedback_pwm1,
               stm32_data.feedback_pwm2)
                  .finished();

          messages::GUISTATUS _guistutus_PC2gui =
              static_cast<messages::GUISTATUS>(stm32_data.feedback_stm32status);
          _gui_link
              .setguilinkRTdata(_guistutus_PC2gui, gps_data.latitude,
                                gps_data.longitude,
                                estimator_RTdata.Measurement_6dof(3),
                                estimator_RTdata.Measurement_6dof(4),
                                estimator_RTdata.State, feedback_pwm, batteries)
              .guicommunication();
          guilink_RTdata = _gui_link.getguilinkRTdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // gui_loop

  // timer
  void utc_timer_loop() {
    common::timecounter utc_timer;

    while (1) {
      pt_utc = utc_timer.getUTCtime();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }  // guiloop

  // socket server
  void socket_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
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
            _sendmsg.double_msg[i] = estimator_RTdata.State(i);  // State

          _sendmsg.double_msg[6] = planner_RTdata.curvature;      // curvature
          _sendmsg.double_msg[7] = planner_RTdata.speed;          // speed
          _sendmsg.double_msg[8] = planner_RTdata.waypoint0(0);   // waypoint0
          _sendmsg.double_msg[9] = planner_RTdata.waypoint0(1);   // waypoint0
          _sendmsg.double_msg[10] = planner_RTdata.waypoint1(0);  // waypoint1
          _sendmsg.double_msg[11] = planner_RTdata.waypoint1(1);  // waypoint1

          for (int i = 0; i != dim_controlspace; ++i) {
            _sendmsg.double_msg[12 + i] = controller_RTdata.tau(i);  // tau
          }
          for (int i = 0; i != num_thruster; ++i) {
            _sendmsg.double_msg[12 + dim_controlspace + i] =
                controller_RTdata.rotation(i);  // rotation
          }
          _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                                  send_size);

          // if (_tcpserver.getconnectioncount() > 0) indicator_socket = 1;

          innerloop_elapsed_time = timer_socket.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "socket") << "Too much time!";
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // experiment: do nothing
        break;
      }
      default:
        break;
    }  // end switch

  }  // socketloop()

};  // end threadloop

}  // end namespace ASV

#endif /* _THREADLOOP_H_ */