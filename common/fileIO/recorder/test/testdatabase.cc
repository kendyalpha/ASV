/*
***********************************************************************
* testdatabase.cc:
* uint test for database
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <boost/test/included/unit_test.hpp>
#include "../include/dataparser.h"
#include "../include/datarecorder.h"

const std::string folderp = "../../data/";
const std::string config_path = "../../config/dbconfig.json";
const double starting_time = 0;
const double end_time = 10;

BOOST_AUTO_TEST_CASE(GPS) {
  // record
  ASV::common::gps_db_data gps_db_data{
      0,              // local_time
      1.11111,        // UTC
      21.22222233,    // latitude
      121.444441112,  // longitude
      4,              // heading
      5.12,           // pitch
      6.11,           // roll
      7.9999,         // altitude
      8.012,          // Ve
      9.111,          // Vn
      10.000001,      // roti
      11,             // status
      12.232323,      // UTM_x
      13.5454,        // UTM_y
      "0n"            // UTM_zone
  };

  ASV::common::imu_db_data imu_db_data{
      0,            // local_time
      0,            // Acc_X
      1.11111,      // Acc_Y
      21.2223,      // Acc_Z
      121.4444412,  // Ang_vel_X
      4,            // Ang_vel_Y
      4,            // Ang_vel_Z
      232.1,        // roll
      232.1,        // pitch
      232.1         // yaw
  };

  ASV::common::gps_db gps_db(folderp, config_path);
  gps_db.create_table();
  for (int i = 0; i != 10; ++i) {
    gps_db.update_gps_table(gps_db_data);
    gps_db.update_imu_table(imu_db_data);
  }

  // parse
  ASV::common::GPS_parser GPS_parser(folderp, config_path);
  auto read_gps = GPS_parser.parse_gps_table(starting_time, end_time);
  auto read_imu = GPS_parser.parse_imu_table(starting_time, end_time);

  // TEST
  {
    BOOST_CHECK_CLOSE(read_gps[0].UTC, gps_db_data.UTC, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].latitude, gps_db_data.latitude, 1e-6);
    BOOST_CHECK_CLOSE(read_gps[0].longitude, gps_db_data.longitude, 1e-6);
    BOOST_CHECK_CLOSE(read_gps[0].heading, gps_db_data.heading, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].pitch, gps_db_data.pitch, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].roll, gps_db_data.roll, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].altitude, gps_db_data.altitude, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].Ve, gps_db_data.Ve, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].Vn, gps_db_data.Vn, 1e-7);
    BOOST_CHECK_CLOSE(read_gps[0].roti, gps_db_data.roti, 1e-7);
    BOOST_TEST(read_gps[0].status == gps_db_data.status);
    BOOST_CHECK_CLOSE(read_gps[0].UTM_x, gps_db_data.UTM_x, 1e-2);
    BOOST_CHECK_CLOSE(read_gps[0].UTM_y, gps_db_data.UTM_y, 1e-2);
    BOOST_TEST(read_gps[0].UTM_zone == gps_db_data.UTM_zone);
  }

  {
    BOOST_CHECK_CLOSE(read_imu[0].Acc_X, imu_db_data.Acc_X, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].Acc_Y, imu_db_data.Acc_Y, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].Acc_Z, imu_db_data.Acc_Z, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].Ang_vel_X, imu_db_data.Ang_vel_X, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].Ang_vel_Y, imu_db_data.Ang_vel_Y, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].Ang_vel_Z, imu_db_data.Ang_vel_Z, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].roll, imu_db_data.roll, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].pitch, imu_db_data.pitch, 1e-6);
    BOOST_CHECK_CLOSE(read_imu[0].yaw, imu_db_data.yaw, 1e-6);
  }
}

BOOST_AUTO_TEST_CASE(wind) {
  // record
  ASV::common::wind_db_data wind_db_data{
      0,       // local_time
      1.22,    // speed
      2.33433  // orientation
  };
  ASV::common::wind_db wind_db(folderp, config_path);
  wind_db.create_table();
  wind_db.update_table(wind_db_data);

  // parse
  ASV::common::wind_parser wind_parser(folderp, config_path);
  auto read_wind = wind_parser.parse_table(starting_time, end_time);

  // TEST
  BOOST_CHECK_CLOSE(read_wind[0].speed, wind_db_data.speed, 1e-7);
  BOOST_CHECK_CLOSE(read_wind[0].orientation, wind_db_data.orientation, 1e-7);
}

BOOST_AUTO_TEST_CASE(stm32) {
  // record
  ASV::common::stm32_db_data stm32_db_data{
      0,   // local_time
      1,   // stm32_link
      2,   // stm32_status
      3,   // command_u1
      4,   // command_u2
      5,   // feedback_u1
      6,   // feedback_u2
      7,   // feedback_pwm1
      8,   // feedback_pwm2
      9,   // RC_X
      10,  // RC_Y
      11,  // RC_Mz
      12,  // voltage_b1
      13,  // voltage_b2
      14   // voltage_b3
  };

  ASV::common::stm32_db stm32_db(folderp, config_path);
  stm32_db.create_table();
  stm32_db.update_table(stm32_db_data);
  // parse
  ASV::common::stm32_parser stm32_parser(folderp, config_path);
  auto read_stm32 = stm32_parser.parse_table(starting_time, end_time);
}

BOOST_AUTO_TEST_CASE(marineradar) {
  // parse
  constexpr int size_array = 512;
  uint8_t spokedata[size_array] = {0xc0, 0x00, 0x23, 0x23, 0x23, 0xff, 0xff,
                                   0xac, 0x55, 0xc0, 0xc0, 0xac, 0xfa, 0xff,
                                   0xac, 0x55, 0xc0, 0x55, 0xac, 0xac, 0xac,
                                   0x55, 0x55, 0xc0, 0xca, 0x01, 0x30, 0x45};
  ASV::common::marineradar_db_data marineradar_db_data{
      0,        // local_time
      10.1111,  // azimuth_deg
      0.9999,   // sample_range
      std::vector<uint8_t>(&spokedata[0],
                           &spokedata[size_array])  // spokedata
  };

  ASV::common::marineradar_db marineradar_db(folderp, config_path);
  marineradar_db.create_table();
  marineradar_db.update_table(marineradar_db_data);

  // parse
  ASV::common::marineradar_parser marineradar_parser(folderp, config_path);
  auto read_marineradar =
      marineradar_parser.parse_table(starting_time, end_time);

  // TEST
  BOOST_CHECK_CLOSE(read_marineradar[0].azimuth_deg,
                    marineradar_db_data.azimuth_deg, 1e-7);
  BOOST_CHECK_CLOSE(read_marineradar[0].sample_range,
                    marineradar_db_data.sample_range, 1e-7);
  for (int i = 0; i != size_array; ++i)
    BOOST_TEST(read_marineradar[0].spokedata[i] ==
               marineradar_db_data.spokedata[i]);
}

BOOST_AUTO_TEST_CASE(estimator) {
  // record
  ASV::common::est_measurement_db_data est_measurement_db_data{
      -1,  // local_time
      1,   // meas_x
      2,   // meas_y
      3,   // meas_theta
      4,   // meas_u
      5,   // meas_v
      6    // meas_r
  };
  ASV::common::est_state_db_data est_state_db_data{
      -1,  // local_time
      1,   // state_x
      2,   // state_y
      3,   // state_theta
      4,   // state_u
      5,   // state_v
      6,   // state_r
      7,   // curvature
      8,   // speed
      9    // dspeed
  };
  ASV::common::est_error_db_data est_error_db_data{
      -1,  // local_time
      6,   // perror_x
      5,   // perror_y
      4,   // perror_mz
      3,   // verror_x
      2,   // verror_y
      1    // verror_mz
  };
  ASV::common::estimator_db estimator_db(folderp, config_path);
  estimator_db.create_table();
  estimator_db.update_measurement_table(est_measurement_db_data);
  estimator_db.update_state_table(est_state_db_data);
  estimator_db.update_error_table(est_error_db_data);

  // parse
  ASV::common::estimator_parser estimator_parser(folderp, config_path);
  auto read_measurement =
      estimator_parser.parse_measurement_table(starting_time, end_time);
  auto read_error = estimator_parser.parse_error_table(starting_time, end_time);
  auto read_state = estimator_parser.parse_state_table(starting_time, end_time);

  // TEST
  BOOST_CHECK_CLOSE(read_measurement[0].meas_x, est_measurement_db_data.meas_x,
                    1e-3);
  BOOST_CHECK_CLOSE(read_measurement[0].meas_y, est_measurement_db_data.meas_y,
                    1e-3);
  BOOST_CHECK_CLOSE(read_measurement[0].meas_theta,
                    est_measurement_db_data.meas_theta, 1e-4);
  BOOST_CHECK_CLOSE(read_measurement[0].meas_u, est_measurement_db_data.meas_u,
                    1e-3);
  BOOST_CHECK_CLOSE(read_measurement[0].meas_v, est_measurement_db_data.meas_v,
                    1e-3);
  BOOST_CHECK_CLOSE(read_measurement[0].meas_r, est_measurement_db_data.meas_r,
                    1e-4);
  BOOST_CHECK_CLOSE(read_state[0].state_x, est_state_db_data.state_x, 1e-3);
  BOOST_CHECK_CLOSE(read_state[0].state_y, est_state_db_data.state_y, 1e-3);
  BOOST_CHECK_CLOSE(read_state[0].state_theta, est_state_db_data.state_theta,
                    1e-4);
  BOOST_CHECK_CLOSE(read_state[0].state_u, est_state_db_data.state_u, 1e-3);
  BOOST_CHECK_CLOSE(read_state[0].state_v, est_state_db_data.state_v, 1e-3);
  BOOST_CHECK_CLOSE(read_state[0].state_r, est_state_db_data.state_r, 1e-4);
  BOOST_CHECK_CLOSE(read_state[0].curvature, est_state_db_data.curvature, 1e-4);
  BOOST_CHECK_CLOSE(read_state[0].speed, est_state_db_data.speed, 1e-4);
  BOOST_CHECK_CLOSE(read_state[0].dspeed, est_state_db_data.dspeed, 1e-4);
  BOOST_CHECK_CLOSE(read_error[0].perror_x, est_error_db_data.perror_x, 1e-3);
  BOOST_CHECK_CLOSE(read_error[0].perror_y, est_error_db_data.perror_y, 1e-3);
  BOOST_CHECK_CLOSE(read_error[0].perror_mz, est_error_db_data.perror_mz, 1e-4);
  BOOST_CHECK_CLOSE(read_error[0].verror_x, est_error_db_data.verror_x, 1e-3);
  BOOST_CHECK_CLOSE(read_error[0].verror_y, est_error_db_data.verror_y, 1e-3);
  BOOST_CHECK_CLOSE(read_error[0].verror_mz, est_error_db_data.verror_mz, 1e-4);
}

BOOST_AUTO_TEST_CASE(planner) {
  // record
  ASV::common::plan_route_db_data plan_route_db_data{
      -1,            // local_time
      1,             // setpoints_X
      2,             // setpoints_Y
      3,             // setpoints_heading
      4,             // setpoints_longitude
      5,             // setpoints_latitude
      1,             // speed
      2,             // captureradius
      "3434",        // utm_zone
      {2, 3, 5, 1},  // WPX
      {4, 4, 4, 4},  // WPY
      {5},           // WPLONG
      {6}            // WPLAT
  };

  ASV::common::plan_lattice_db_data plan_lattice_db_data{
      -1,  // local_time
      1,   // lattice_x
      2,   // lattice_y
      3,   // lattice_theta
      4,   // lattice_kappa
      5,   // lattice_speed
      6    // lattice_dspeed
  };

  ASV::common::planner_db planner_db(folderp, config_path);
  planner_db.create_table();
  planner_db.update_routeplanner_table(plan_route_db_data);
  planner_db.update_latticeplanner_table(plan_lattice_db_data);

  // parse
  ASV::common::planner_parser planner_parser(folderp, config_path);
  auto read_route = planner_parser.parse_route_table(starting_time, end_time);
  auto read_lattice =
      planner_parser.parse_lattice_table(starting_time, end_time);
}

BOOST_AUTO_TEST_CASE(control) {
  // record
  constexpr int m = 4;
  Eigen::Matrix<int, m, 1> azimuth;
  Eigen::Matrix<int, m, 1> rotation;
  azimuth << 100, 200, 300, 400;
  rotation << 900, 800, 700, 600;
  ASV::common::control_setpoint_db_data control_setpoint_db_data{
      -1,  // local_time
      1,   // set_x
      0,   // set_y
      2,   // set_theta
      3,   // set_u
      4,   // set_v
      5    // set_r
  };

  ASV::common::control_TA_db_data control_TA_db_data{
      -1,                                                     // local_time
      1,                                                      // desired_Fx
      0,                                                      // desired_Fy
      2,                                                      // desired_Mz
      3,                                                      // est_Fx
      4,                                                      // est_Fy
      5,                                                      // est_Mz
      std::vector<int>(azimuth.data(), azimuth.data() + m),   // alpha
      std::vector<int>(rotation.data(), rotation.data() + m)  // rpm
  };

  ASV::common::controller_db controller_db(folderp, config_path);
  controller_db.create_table();
  controller_db.update_setpoint_table(control_setpoint_db_data);
  controller_db.update_TA_table(control_TA_db_data);

  // parse
  ASV::common::control_parser control_parser(folderp, config_path);
  auto read_setpoint =
      control_parser.parse_setpoint_table(starting_time, end_time);
  auto read_TA = control_parser.parse_TA_table(starting_time, end_time);
}

BOOST_AUTO_TEST_CASE(perception) {
  // record
  ASV::common::perception_db perception_db(folderp, config_path);
  perception_db.create_table();
  perception_db.update_spoke_table(ASV::common::perception_spoke_db_data{
      -1,                                 // local_time
      std::vector<double>({1, 2, 3, 4}),  // surroundings_bearing_rad
      std::vector<double>({1, 2, 3, 4}),  // surroundings_range_m
      std::vector<double>({1, 2, 3, 4}),  // surroundings_x_m
      std::vector<double>({1, 2, 3, 4})   // surroundings_y_m
  });
  perception_db.update_detection_table(
      ASV::common::perception_detection_db_data{
          -1,                                 // local_time
          std::vector<double>({1, 2, 3, 4}),  // detected_target_x
          std::vector<double>({1, 2, 3, 4}),  // detected_target_y
          std::vector<double>({1, 2, 3, 4})   // detected_target_radius
      });

  constexpr int num_target = 10;
  Eigen::Matrix<int, num_target, 1> targets_state =
      Eigen::Matrix<int, num_target, 1>::Random();
  Eigen::Matrix<int, num_target, 1> targets_intention =
      Eigen::Matrix<int, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_x =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_y =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_square_radius =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_vx =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_vy =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_CPA_x =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_CPA_y =
      Eigen::Matrix<double, num_target, 1>::Random();
  Eigen::Matrix<double, num_target, 1> targets_TCPA =
      Eigen::Matrix<double, num_target, 1>::Random();
  perception_db.update_trackingtarget_table(
      ASV::common::perception_trackingtarget_db_data{
          -1,  // local_time
          20,  // spoke_state
          std::vector<int>(targets_state.data(),
                           targets_state.data() + num_target),  // targets_state
          std::vector<int>(
              targets_intention.data(),
              targets_intention.data() + num_target),  // targets_intention
          std::vector<double>(targets_x.data(),
                              targets_x.data() + num_target),  // targets_x
          std::vector<double>(targets_y.data(),
                              targets_y.data() + num_target),  // targets_y
          std::vector<double>(targets_square_radius.data(),
                              targets_square_radius.data() +
                                  num_target),  // targets_square_radius
          std::vector<double>(targets_vx.data(),
                              targets_vx.data() + num_target),  // targets_vx
          std::vector<double>(targets_vy.data(),
                              targets_vy.data() + num_target),  // targets_vy
          std::vector<double>(
              targets_CPA_x.data(),
              targets_CPA_x.data() + num_target),  // targets_CPA_x
          std::vector<double>(
              targets_CPA_y.data(),
              targets_CPA_y.data() + num_target),  // targets_CPA_y
          std::vector<double>(targets_TCPA.data(),
                              targets_TCPA.data() + num_target)  // targets_TCPA
      });
  // parse
  ASV::common::perception_parser perception_parser(folderp, config_path);
  auto read_spoke =
      perception_parser.parse_spoke_table(starting_time, end_time);
  auto read_detection =
      perception_parser.parse_detection_table(starting_time, end_time);
  auto read_TT = perception_parser.parse_TT_table(starting_time, end_time);
  for (auto const &value : read_TT) {
    std::cout << value.local_time << std::endl;
  }
}
