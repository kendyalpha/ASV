/*
***********************************************************************
* testrecorder.cc:
* uint test for data recorder
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/datarecorder.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  LOG(INFO) << "The program has started!";

  const std::string folderp = "../../data/";
  const std::string config_path = "../../config/dbconfig.json";

  // GPS
  ASV::common::gps_db gps_db(folderp, config_path, "1");
  gps_db.create_table();
  for (int i = 0; i != 10; ++i)
    gps_db.update_table(ASV::common::gps_db_data{
        0,    // local_time
        1,    // UTC
        2,    // latitude
        3,    // longitude
        4,    // heading
        5,    // pitch
        6,    // roll
        7,    // altitude
        8,    // Ve
        9,    // Vn
        10,   // roti
        11,   // status
        12,   // UTM_x
        13,   // UTM_y
        "0n"  // UTM_zone
    });

  // wind
  ASV::common::wind_db wind_db(folderp, config_path);
  wind_db.create_table();
  wind_db.update_table(
      ASV::common::wind_db_data{
          0,  // local_time
          1,  // speed
          2   // orientation
      },
      "3333");

  // stm32
  ASV::common::stm32_db stm32_db(folderp, config_path);
  stm32_db.create_table();
  stm32_db.update_table(ASV::common::stm32_db_data{
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
  });

  // marine radar
  constexpr int size_array = 512;
  uint8_t _value = 0x23;
  uint8_t spokedata[size_array] = {
      0xc0,   0x00,   _value, _value, _value, _value, _value,
      _value, _value, _value, _value, _value, _value, _value,
      _value, _value, _value, _value, _value, _value, _value,
      _value, _value, _value, 0xca,   0x01,   0x30,   0x45};
  ASV::common::marineradar_db marineradar_db(folderp, config_path);
  marineradar_db.create_table();

  marineradar_db.update_table(ASV::common::marineradar_db_data{
      0,   // local_time
      10,  // azimuth_deg
      20,  // sample_range
      std::vector<uint8_t>(&spokedata[0],
                           &spokedata[size_array])  // spokedata
  });

  // estimator
  ASV::common::estimator_db estimator_db(folderp, config_path);
  estimator_db.create_table();
  estimator_db.update_measurement_table(ASV::common::est_measurement_db_data{
      -1,  // local_time
      1,   // meas_x
      2,   // meas_y
      3,   // meas_theta
      4,   // meas_u
      5,   // meas_v
      6    // meas_r
  });
  estimator_db.update_state_table(ASV::common::est_state_db_data{
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
  });
  estimator_db.update_error_table(ASV::common::est_error_db_data{
      -1,  // local_time
      6,   // perror_x
      5,   // perror_y
      4,   // perror_mz
      3,   // verror_x
      2,   // verror_y
      1    // verror_mz
  });

  // planner
  ASV::common::planner_db planner_db(folderp, config_path);
  planner_db.create_table();
  planner_db.update_routeplanner_table(ASV::common::plan_route_db_data{
      -1,  // local_time
      1,   // speed
      2,   // captureradius
      3,   // WPX
      4,   // WPY
      5,   // WPLONG
      6    // WPLAT
  });
  planner_db.update_latticeplanner_table(ASV::common::plan_lattice_db_data{
      -1,  // local_time
      1,   // lattice_x
      2,   // lattice_y
      3,   // lattice_theta
      4,   // lattice_kappa
      5,   // lattice_speed
      6    // lattice_dspeed
  });

  // controller
  constexpr int m = 4;
  Eigen::Matrix<int, m, 1> azimuth;
  Eigen::Matrix<int, m, 1> rotation;
  azimuth << 100, 200, 300, 400;
  rotation << 900, 800, 700, 600;

  ASV::common::controller_db controller_db(folderp, config_path);
  controller_db.create_table();
  controller_db.update_setpoint_table(ASV::common::control_setpoint_db_data{
      -1,  // local_time
      1,   // set_x
      0,   // set_y
      2,   // set_theta
      3,   // set_u
      4,   // set_v
      5    // set_r
  });

  controller_db.update_TA_table(ASV::common::control_TA_db_data{
      -1,                                                     // local_time
      1,                                                      // desired_Fx
      0,                                                      // desired_Fy
      2,                                                      // desired_Mz
      3,                                                      // est_Fx
      4,                                                      // est_Fy
      5,                                                      // est_Mz
      std::vector<int>(azimuth.data(), azimuth.data() + m),   // alpha
      std::vector<int>(rotation.data(), rotation.data() + m)  // rpm
  });

  // perception
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

  LOG(INFO) << "Shutting down.";
}