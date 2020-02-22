/*
***********************************************************************
* datarecorder.h:
* database using sqlite3 and sqlite modern cpp wrapper
* This header file can be read by C++ compilers
*
* db_config.json is used to construct the tables in database
* databasedata.h is used to update the data in database
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <sqlite_modern_cpp.h>
#include "common/fileIO/recorder/include/datarecorder.h"

void parsedb(const std::string &parse_dbpath, const std::string &new_dbpath,
             const std::string &config_path) {
  sqlite::database db(parse_dbpath);

  std::string datatime0;
  db << "select DATETIME from GPS where ID=1;" >> datatime0;

  ASV::common::gps_db gps_db(new_dbpath, config_path, datatime0);
  gps_db.create_table();
  std::string parse_string =
      "select DATETIME, UTC, latitude, longitude, heading, pitch, roll, "
      "altitude, Ve, Vn, roti, status, UTM_x, UTM_y, UTM_zone from GPS where "
      "ID=?;";
  // loop through the database
  int max_id = 0;
  db << "select MAX(ID) from GPS;" >> max_id;
  for (int i = 0; i != max_id; i++) {
    db << parse_string << i + 1 >>
        [&](std::string local_time, double UTC, double latitude,
            double longitude, double heading, double pitch, double roll,
            double altitude, double Ve, double Vn, double roti, int status,
            double UTM_x, double UTM_y, std::string UTM_zone) {
          gps_db.update_table(
              ASV::common::gps_db_data{
                  0,          // local_time
                  UTC,        // UTC
                  latitude,   // latitude
                  longitude,  // longitude
                  heading,    // heading
                  pitch,      // pitch
                  roll,       // roll
                  altitude,   // altitude
                  Ve,         // Ve
                  Vn,         // Vn
                  roti,       // roti
                  status,     // status
                  UTM_x,      // UTM_x
                  UTM_y,      // UTM_y
                  UTM_zone    // UTM_zone
              },
              local_time);
        };
  }

  ASV::common::controller_db controller_db(new_dbpath, config_path);
  controller_db.create_table();
  parse_string =
      "select DATETIME, set_x, set_y, set_theta, set_u, set_v, set_r, tau1, "
      "tau2, tau3, alpha1, alpha2, rpm1, rpm2, est1, est2, est3 from "
      "controller where ID=?;";
  // loop through the database
  db << "select MAX(ID) from controller;" >> max_id;
  for (int i = 0; i != max_id; i++) {
    db << parse_string << i + 1 >>
        [&](std::string local_time, double set_x, double set_y,
            double set_theta, double set_u, double set_v, double set_r,
            double tau1, double tau2, double tau3, int alpha1, int alpha2,
            int rpm1, int rpm2, double est1, double est2, double est3) {
          controller_db.update_setpoint_table(
              ASV::common::control_setpoint_db_data{
                  0,          // local_time
                  set_x,      // set_x
                  set_y,      // set_y
                  set_theta,  // set_theta
                  set_u,      // set_u
                  set_v,      // set_v
                  set_r       // set_r
              },
              local_time);
          controller_db.update_TA_table(
              ASV::common::control_TA_db_data{
                  -1,                                  // local_time
                  tau1,                                // desired_Fx
                  tau2,                                // desired_Fy
                  tau3,                                // desired_Mz
                  est1,                                // est_Fx
                  est2,                                // est_Fy
                  est3,                                // est_Mz
                  std::vector<int>({alpha1, alpha2}),  // alpha
                  std::vector<int>({rpm1, rpm2})       // rpm
              },
              local_time);
        };
  }

  ASV::common::planner_db planner_db(new_dbpath, config_path);
  planner_db.create_table();
  parse_string =
      "select DATETIME, speed, captureradius, WPX, WPY, WPLONG, WPLAT from "
      "route_planning where ID=?;";
  // loop through the database
  db << "select MAX(ID) from route_planning;" >> max_id;
  for (int i = 0; i != max_id; i++) {
    db << parse_string << i + 1 >>
        [&](std::string local_time, double speed, double captureradius,
            double WPX, double WPY, double WPLONG, double WPLAT) {
          planner_db.update_routeplanner_table(
              ASV::common::plan_route_db_data{
                  -1,             // local_time
                  speed,          // speed
                  captureradius,  // captureradius
                  WPX,            // WPX
                  WPY,            // WPY
                  WPLONG,         // WPLONG
                  WPLAT           // WPLAT
              },
              local_time);
        };
  }

  ASV::common::estimator_db estimator_db(new_dbpath, config_path);
  estimator_db.create_table();
  parse_string =
      "select DATETIME, meas_x, meas_y, meas_theta, meas_u, meas_v, meas_r, "
      "state_x, state_y, state_theta, state_u, state_v, state_r, perror_x, "
      "perror_y, perror_mz, verror_x, verror_y, verror_mz, curvature, "
      "speed, dspeed from estimator where ID=?;";
  // loop through the database
  db << "select MAX(ID) from estimator;" >> max_id;
  for (int i = 0; i != max_id; i++) {
    db << parse_string << i + 1 >>
        [&](std::string local_time, double meas_x, double meas_y,
            double meas_theta, double meas_u, double meas_v, double meas_r,
            double state_x, double state_y, double state_theta, double state_u,
            double state_v, double state_r, double perror_x, double perror_y,
            double perror_mz, double verror_x, double verror_y,
            double verror_mz, double curvature, double speed, double dspeed) {
          estimator_db.update_measurement_table(
              ASV::common::est_measurement_db_data{
                  -1,          // local_time
                  meas_x,      // meas_x
                  meas_y,      // meas_y
                  meas_theta,  // meas_theta
                  meas_u,      // meas_u
                  meas_v,      // meas_v
                  meas_r       // meas_r
              },
              local_time);
          estimator_db.update_state_table(
              ASV::common::est_state_db_data{
                  -1,           // local_time
                  state_x,      // state_x
                  state_y,      // state_y
                  state_theta,  // state_theta
                  state_u,      // state_u
                  state_v,      // state_v
                  state_r,      // state_r
                  curvature,    // curvature
                  speed,        // speed
                  dspeed        // dspeed
              },
              local_time);
          estimator_db.update_error_table(
              ASV::common::est_error_db_data{
                  -1,         // local_time
                  perror_x,   // perror_x
                  perror_y,   // perror_y
                  perror_mz,  // perror_mz
                  verror_x,   // verror_x
                  verror_y,   // verror_y
                  verror_mz   // verror_mz
              },
              local_time);
        };
  }

  ASV::common::stm32_db stm32_db(new_dbpath, config_path);
  stm32_db.create_table();
  parse_string =
      "select DATETIME, stm32_link, stm32_status, command_u1, command_u2, "
      "feedback_u1, feedback_u2, feedback_pwm1, feedback_pwm2, RC_X, RC_Y, "
      "RC_Mz, voltage_b1, voltage_b2, voltage_b3 from stm32 where ID=?;";
  // loop through the database
  db << "select MAX(ID) from stm32;" >> max_id;
  for (int i = 0; i != max_id; i++) {
    db << parse_string << i + 1 >>
        [&](std::string local_time, int stm32_link, int stm32_status,
            double command_u1, double command_u2, double feedback_u1,
            double feedback_u2, int feedback_pwm1, int feedback_pwm2,
            double RC_X, double RC_Y, double RC_Mz, double voltage_b1,
            double voltage_b2, double voltage_b3) {
          stm32_db.update_table(
              ASV::common::stm32_db_data{
                  0,              // local_time
                  stm32_link,     // stm32_link
                  stm32_status,   // stm32_status
                  command_u1,     // command_u1
                  command_u2,     // command_u2
                  feedback_u1,    // feedback_u1
                  feedback_u2,    // feedback_u2
                  feedback_pwm1,  // feedback_pwm1
                  feedback_pwm2,  // feedback_pwm2
                  RC_X,           // RC_X
                  RC_Y,           // RC_Y
                  RC_Mz,          // RC_Mz
                  voltage_b1,     // voltage_b1
                  voltage_b2,     // voltage_b2
                  voltage_b3      // voltage_b3
              },
              local_time);
        };
  }
}

int main() {
  const std::string parse_dbpath = "../test.db";
  const std::string new_dbpath = "../data/";
  const std::string config_path =
      "../../../common/fileIO/recorder/config/dbconfig.json";
  parsedb(parse_dbpath, new_dbpath, config_path);
}
