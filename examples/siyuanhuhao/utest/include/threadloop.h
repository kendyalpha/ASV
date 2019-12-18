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

#include "config.h"

namespace ASV {

class threadloop : public StateMonitor {
 public:
  threadloop()
      : StateMonitor(),
        _jsonparse("./../../properties/property.json"),
        _sqlite(_jsonparse.getsqlitedata()) {
    _sqlite.initializetables();
    // // write prettified JSON to another file
    // std::ofstream o("pretty.json");
    // o << std::setw(4) << j << std::endl;
  }
  ~threadloop() {}

  void mainloop() {
    std::thread spokeprocess_thread(&threadloop::spoke_process_loop, this);
    std::thread route_planner_thread(&threadloop::route_planner_loop, this);
    std::thread path_planner_thread(&threadloop::path_planner_loop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread gps_thread(&threadloop::gps_loop, this);
    std::thread marine_radar_thread(&threadloop::marine_radar_loop, this);
    std::thread timer_thread(&threadloop::utc_timer_loop, this);
    std::thread gui_thread(&threadloop::gui_loop, this);
    std::thread stm32_thread(&threadloop::stm32loop, this);
    std::thread socket_thread(&threadloop::socket_loop, this);
    std::thread statemonitor_thread(&threadloop::state_monitor_loop, this);

    spokeprocess_thread.join();
    route_planner_thread.join();
    path_planner_thread.join();
    estimator_thread.join();
    controller_thread.join();
    sql_thread.join();
    gps_thread.join();
    marine_radar_thread.join();
    timer_thread.join();
    gui_thread.join();
    stm32_thread.join();
    socket_thread.join();
    statemonitor_thread.join();
  }

 private:
  /********************* Real time Data  *********************/
  planning::RoutePlannerRTdata RoutePlanner_RTdata{
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

  // real time data of tracker
  control::trackerRTdata tracker_RTdata{
      control::TRACKERMODE::STARTED,  // trackermode
      Eigen::Vector3d::Zero(),        // setpoint
      Eigen::Vector3d::Zero()         // v_setpoint
  };

  // real time data of controller
  control::controllerRTdata<num_thruster, dim_controlspace> controller_RTdata{
      common::STATETOGGLE::IDLE,                           // state_toggle
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // command_u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // command_rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // command_alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // command_alpha_deg
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // feedback_u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // feedback_rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // feedback_alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // feedback_alpha_deg
  };

  // realtime parameters of the estimators
  estimatorRTdata estimator_RTdata{
      common::STATETOGGLE::IDLE,            // state_toggle
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
      0,  // UTM_y
      ""  // UTM_zone
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
      Eigen::Matrix<double, 2, 8>::Zero(),          // waypoints
      Eigen::VectorXd::Zero(2),                     // WX
      Eigen::VectorXd::Zero(2)                      // WY
  };

  // real time sourroundings
  perception::SpokeProcessRTdata SpokeProcess_RTdata;

  //
  perception::MarineRadarRTdata MarineRadar_RTdata{
      0,                  // spoke_azimuth_deg
      0,                  // spoke_samplerange_m
      {0x00, 0x00, 0x00}  // spokedata
  };

  // real time utc
  std::string pt_utc;

  /********************* Modules  *********************/
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;
  // sqlite
  common::database<num_thruster, dim_controlspace> _sqlite;

  // spoke processing
  void spoke_process_loop() {
    perception::SpokeProcessing Spoke_Processing(
        _jsonparse.getalarmzonedata(), _jsonparse.getSpokeProcessdata());

    common::timecounter timer_spokeprocess;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * Spoke_Processing.getsampletime());

    std::size_t size_spokedata = sizeof(MarineRadar_RTdata.spokedata) /
                                 sizeof(MarineRadar_RTdata.spokedata[0]);

    while (1) {
      SpokeProcess_RTdata =
          Spoke_Processing
              .DetectionOnSpoke(MarineRadar_RTdata.spokedata, size_spokedata,
                                MarineRadar_RTdata.spoke_azimuth_deg,
                                MarineRadar_RTdata.spoke_samplerange_m,
                                estimator_RTdata.State(0),
                                estimator_RTdata.State(1),
                                estimator_RTdata.State(2))
              .getSpokeProcessRTdata();
    }
  }  // spoke_process_loop

  void route_planner_loop() {
    planning::RoutePlanning _RoutePlanner(RoutePlanner_RTdata,
                                          _jsonparse.getvessel());

    while (1) {
      if (RoutePlanner_RTdata.state_toggle == common::STATETOGGLE::IDLE) {
        double initial_long = 121.4378246;
        double initial_lat = 31.0285510;
        double final_long = 121.4388565;
        double final_lat = 31.0282296;

        Eigen::VectorXd W_long(2);
        Eigen::VectorXd W_lat(2);
        W_long << initial_long, final_long;
        W_lat << initial_lat, final_lat;

        RoutePlanner_RTdata = _RoutePlanner.setCruiseSpeed(1)
                                  .setWaypoints(W_long, W_lat)
                                  .getRoutePlannerRTdata();

        _sqlite.update_routeplanner_table(RoutePlanner_RTdata);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }  // route_planner_loop

  void path_planner_loop() {
    planning::LatticePlanner _trajectorygenerator(
        _jsonparse.getlatticedata(), _jsonparse.getcollisiondata());

    common::timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * _trajectorygenerator.getsampletime());

    StateMonitor::check_planner();

    std::vector<double> ob_x(1);
    std::vector<double> ob_y(1);
    // ob_x << 3433794;
    // ob_y << 350955;

    _trajectorygenerator.regenerate_target_course(
        RoutePlanner_RTdata.Waypoint_X, RoutePlanner_RTdata.Waypoint_Y);

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
          _trajectorygenerator.setup_obstacle(ob_x, ob_y);

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
          _trajectorygenerator.setup_obstacle(ob_x, ob_y);

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

  }  // path_planner_loop

  void controllerloop() {
    control::controller<10, num_thruster, indicator_actuation, dim_controlspace>
        _controller(controller_RTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata());

    control::trajectorytracking _trajectorytracking(
        _jsonparse.getcontrollerdata(), tracker_RTdata);

    common::timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    StateMonitor::check_controller();

    controller_RTdata =
        _controller.initializecontroller().getcontrollerRTdata();

    _trajectorytracking.set_grid_points(
        RoutePlanner_RTdata.Waypoint_X, RoutePlanner_RTdata.Waypoint_Y,
        RoutePlanner_RTdata.speed, RoutePlanner_RTdata.los_capture_radius);

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          tracker_RTdata.setpoint = Eigen::Vector3d::Zero();
          tracker_RTdata.v_setpoint = Eigen::Vector3d::Zero();

          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);

          break;
        }
        case common::TESTMODE::SIMULATION_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          _trajectorytracking.Grid_LOS(estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::SIMULATION_FRENET: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP: {
          _controller.setcontrolmode(control::CONTROLMODE::DYNAMICPOSITION);

          break;
        }
        case common::TESTMODE::EXPERIMENT_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);

          // Eigen::Vector2i feedback_rotation;
          // Eigen::Vector2i feedback_alpha;
          // int rotation1 = stm32_data.feedback_pwm1 - 9250;
          // int rotation2 = stm32_data.feedback_pwm2 - 9250;

          // if (rotation1 > 0) {
          //   feedback_rotation(0) = rotation1;
          //   feedback_alpha(0) = 0;
          // } else {
          //   feedback_rotation(0) = -rotation1;
          //   feedback_alpha(0) = 180;
          // }

          // if (rotation2 > 0) {
          //   feedback_rotation(1) = rotation2;
          //   feedback_alpha(1) = 0;
          // } else {
          //   feedback_rotation(1) = -rotation2;
          //   feedback_alpha(1) = 180;
          // }

          // _controller.set_thruster_feedback(feedback_rotation,
          // feedback_alpha);

          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);

          // trajectory tracking
          _trajectorytracking.Grid_LOS(estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::EXPERIMENT_FRENET: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();

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
                                                 Eigen::Vector3d::Zero(),
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
    // initialization of estimator
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        // simulation
        estimator<indicator_kalman,  // indicator_kalman
                  1,                 // nlp_x
                  1,                 // nlp_y
                  1,                 // nlp_z
                  1,                 // nlp_heading
                  1,                 // nlp_roll
                  1,                 // nlp_pitch
                  1,                 // nlp_u
                  1,                 // nlp_v
                  1                  // nlp_roti
                  >
            _estimator(estimator_RTdata, _jsonparse.getvessel(),
                       _jsonparse.getestimatordata());

        simulation::simulator _simulator(_jsonparse.getsimulatordata(),
                                         _jsonparse.getvessel());

        common::timecounter timer_estimator;
        long int outerloop_elapsed_time = 0;
        long int innerloop_elapsed_time = 0;
        long int sample_time =
            static_cast<long int>(1000 * _estimator.getsampletime());

        // State monitor toggle
        StateMonitor::check_estimator();

        estimator_RTdata =
            _estimator.setvalue(350938.7, 3433823.54, 0, 0, 0, 90, 0, 0, 0)
                .getEstimatorRTData();
        _simulator.setX(estimator_RTdata.State);

        // real time calculation in estimator
        while (1) {
          outerloop_elapsed_time = timer_estimator.timeelapsed();

          auto x = _simulator
                       .simulator_onestep(tracker_RTdata.setpoint(2),
                                          controller_RTdata.BalphaU)
                       .getX();
          _estimator
              .updateestimatedforce(controller_RTdata.BalphaU,
                                    Eigen::Vector3d::Zero())
              .estimatestate(x, tracker_RTdata.setpoint(2));

          estimator_RTdata = _estimator
                                 .estimateerror(tracker_RTdata.setpoint,
                                                tracker_RTdata.v_setpoint)
                                 .getEstimatorRTData();

          innerloop_elapsed_time = timer_estimator.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "estimator") << "Too much time!";
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        //                  experiment                        //

        // initializtion
        estimator<indicator_kalman,  // indicator_kalman
                  1,                 // nlp_x
                  1,                 // nlp_y
                  1,                 // nlp_z
                  1,                 // nlp_heading
                  1,                 // nlp_roll
                  1,                 // nlp_pitch
                  5,                 // nlp_u
                  5,                 // nlp_v
                  1                  // nlp_roti
                  >
            _estimator(estimator_RTdata, _jsonparse.getvessel(),
                       _jsonparse.getestimatordata());

        common::timecounter timer_estimator;
        long int outerloop_elapsed_time = 0;
        long int innerloop_elapsed_time = 0;
        long int sample_time =
            static_cast<long int>(1000 * _estimator.getsampletime());

        // State monitor toggle
        StateMonitor::check_estimator();
        estimator_RTdata = _estimator
                               .setvalue(gps_data.UTM_x,     // gps_x
                                         gps_data.UTM_y,     // gps_y
                                         gps_data.altitude,  // gps_z
                                         gps_data.roll,      // gps_roll
                                         gps_data.pitch,     // gps_pitch
                                         gps_data.heading,   // gps_heading
                                         gps_data.Ve,        // gps_Ve
                                         gps_data.Vn,        // gps_Vn
                                         gps_data.roti       // gps_roti
                                         )
                               .getEstimatorRTData();

        // real time calculation in estimator
        while (1) {
          outerloop_elapsed_time = timer_estimator.timeelapsed();

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

          estimator_RTdata = _estimator
                                 .estimateerror(tracker_RTdata.setpoint,
                                                tracker_RTdata.v_setpoint)
                                 .getEstimatorRTData();

          innerloop_elapsed_time = timer_estimator.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "estimator") << "Too much time!";
        }

        break;
      }
      default:
        break;
    }

  }  // estimatorloop()

  // loop to save real time data using sqlite3 and modern_sqlite3_cpp_wrapper
  void sqlloop() {
    while (1) {
      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::SIMULATION_FRENET: {
          // simulation
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
              .setstm32data(_command_stm32, pt_utc, controller_RTdata.command_u,
                            controller_RTdata.command_alpha)
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
        messages::GPS _gpsimu(_jsonparse.getgpsbaudrate(),
                              _jsonparse.getgpsport());

        // experiment
        while (1) {
          gps_data =
              _gpsimu.parseGPS(RoutePlanner_RTdata.utm_zone).getgpsRTdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // gps_loop()

  // marine radar giving spoke data
  void marine_radar_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        perception::MarineRadar Marine_Radar;

        Marine_Radar.StartMarineRadar();
        // experiment
        while (1) {
          MarineRadar_RTdata = Marine_Radar.getMarineRadarRTdata();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // marine_radar_loop

  // gui data link
  void gui_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS: {
        guilink_RTdata.WX = Eigen::VectorXd::Zero(5);
        guilink_RTdata.WY = Eigen::VectorXd::Zero(5);
        guilink_RTdata.WX << 3433823.54, 3433803, 3433769, 3433790.37,
            3433823.54;
        guilink_RTdata.WY << 350938.7, 350928.66, 350987, 351004, 350938.7;
        guilink_RTdata.desired_speed = 2;

        break;
      }
      case common::TESTMODE::SIMULATION_FRENET: {
        guilink_RTdata.desired_speed = 2;

        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS: {
        guilink_RTdata.WX = Eigen::VectorXd::Zero(5);
        guilink_RTdata.WY = Eigen::VectorXd::Zero(5);
        guilink_RTdata.WX << 3433823.54, 3433803, 3433769, 3433790.37,
            3433823.54;
        guilink_RTdata.WY << 350938.7, 350928.66, 350987, 351004, 350938.7;
        guilink_RTdata.desired_speed = 1;

        break;
      }
      case common::TESTMODE::EXPERIMENT_FRENET: {
        messages::guilink_serial<num_thruster, 3, dim_controlspace> _gui_link(
            guilink_RTdata, _jsonparse.getguibaudrate(),
            _jsonparse.getguiport());

        guilink_RTdata.desired_speed = 2;

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
  }  // utc_timer_loop

  void state_monitor_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET: {
        while (1) {
          if (StateMonitor::indicator_estimator == common::STATETOGGLE::IDLE) {
            StateMonitor::indicator_estimator = common::STATETOGGLE::READY;
            CLOG(INFO, "estimator") << "initialation successful!";
          }

          if (StateMonitor::indicator_planner == common::STATETOGGLE::IDLE &&
              estimator_RTdata.state_toggle == common::STATETOGGLE::READY) {
            StateMonitor::indicator_planner = common::STATETOGGLE::READY;
            CLOG(INFO, "planner") << "initialation successful!";
          }

          if (StateMonitor::indicator_estimator == common::STATETOGGLE::READY &&
              StateMonitor::indicator_planner == common::STATETOGGLE::READY &&
              StateMonitor::indicator_controller == common::STATETOGGLE::IDLE) {
            StateMonitor::indicator_controller = common::STATETOGGLE::READY;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // experiment

        while (1) {
          if (StateMonitor::indicator_planner == common::STATETOGGLE::IDLE &&
              RoutePlanner_RTdata.state_toggle == common::STATETOGGLE::READY) {
            StateMonitor::indicator_planner = common::STATETOGGLE::READY;
            CLOG(INFO, "planner") << "initialation successful!";
          }

          if (gps_data.status >= 1 &&
              StateMonitor::indicator_gps == common::STATETOGGLE::IDLE &&
              StateMonitor::indicator_planner == common::STATETOGGLE::READY) {
            StateMonitor::indicator_gps = common::STATETOGGLE::READY;
            CLOG(INFO, "GPS") << "initialation successful!";
          }

          if (StateMonitor::indicator_gps == common::STATETOGGLE::READY &&
              StateMonitor::indicator_estimator == common::STATETOGGLE::IDLE) {
            StateMonitor::indicator_estimator = common::STATETOGGLE::READY;
            CLOG(INFO, "estimator") << "initialation successful!";
          }

          if (StateMonitor::indicator_estimator == common::STATETOGGLE::READY &&
              StateMonitor::indicator_planner == common::STATETOGGLE::READY &&
              StateMonitor::indicator_controller == common::STATETOGGLE::IDLE) {
            StateMonitor::indicator_controller = common::STATETOGGLE::READY;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        break;
      }
    }

  }  // state_monitor_loop

  // socket server
  void socket_loop() {
    tcpserver _tcpserver("9340");

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

          _sendmsg.double_msg[6] =
              RoutePlanner_RTdata.los_capture_radius;          // curvature
          _sendmsg.double_msg[7] = RoutePlanner_RTdata.speed;  // speed
          _sendmsg.double_msg[8] =
              RoutePlanner_RTdata.setpoints_X;  // waypoint0
          _sendmsg.double_msg[9] =
              RoutePlanner_RTdata.setpoints_Y;  // waypoint0
          _sendmsg.double_msg[10] =
              RoutePlanner_RTdata.setpoints_heading;  // waypoint1
          _sendmsg.double_msg[11] =
              RoutePlanner_RTdata.setpoints_longitude;  // waypoint1

          for (int i = 0; i != dim_controlspace; ++i) {
            _sendmsg.double_msg[12 + i] = controller_RTdata.tau(i);  // tau
          }
          for (int i = 0; i != num_thruster; ++i) {
            _sendmsg.double_msg[12 + dim_controlspace + i] =
                controller_RTdata.command_rotation(i);  // rotation
          }
          _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                                  send_size);

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

};  // namespace ASV

}  // end namespace ASV

#endif /* _THREADLOOP_H_ */