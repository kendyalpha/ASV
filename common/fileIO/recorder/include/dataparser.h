/*
***********************************************************************
* dataparser.h:
* parse data from sqlite3
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DATAPARSER_H_
#define _DATAPARSER_H_

#include <sqlite_modern_cpp.h>
#include <stdlib.h>
#include <string>

#include "common/fileIO/include/json.hpp"
#include "common/logging/include/easylogging++.h"

#include "databasedata.h"

namespace ASV::common {

class master_parser {
 public:
  explicit master_parser(const std::string &_DB_folder_path) : timestamp0(0) {
    std::string dbpath = _DB_folder_path + "master.db";
    sqlite::database db(dbpath);
    db << "select DATETIME from info where ID = 1;" >>
        [&](std::string _datetime) { timestamp0 = atof(_datetime.c_str()); };
  }
  virtual ~master_parser() = default;

 protected:
  double timestamp0;
};  // end class master_parser

class GPS_parser : public master_parser {
 public:
  explicit GPS_parser(const std::string &_DB_folder_path,
                      const std::string &_config_name)
      : master_parser(_DB_folder_path),
        config_name(_config_name),
        db(_DB_folder_path + "gps.db") {}

  ~GPS_parser() {}

  std::vector<gps_db_data> parse_table(const double start_time,
                                       const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["GPS"].get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from GPS where ID= ?;";

    //
    std::vector<gps_db_data> v_gps_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from GPS;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, double UTC, double latitude,
              double longitude, double heading, double pitch, double roll,
              double altitude, double Ve, double Vn, double roti, int status,
              double UTM_x, double UTM_y, std::string UTM_zone) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_gps_db_data.push_back(gps_db_data{
                  _local_time_s,  // local_time
                  UTC,            // UTC
                  latitude,       // latitude
                  longitude,      // longitude
                  heading,        // heading
                  pitch,          // pitch
                  roll,           // roll
                  altitude,       // altitude
                  Ve,             // Ve
                  Vn,             // Vn
                  roti,           // roti
                  status,         // status
                  UTM_x,          // UTM_x
                  UTM_y,          // UTM_y
                  UTM_zone        // UTM_zone
              });
            }
          };
    }
    return v_gps_db_data;
  }

 private:
  std::string config_name;

  sqlite::database db;

};  // end class GPS_parser

class wind_parser : public master_parser {
 public:
  explicit wind_parser(const std::string &_DB_folder_path,
                       const std::string &_config_name)
      : master_parser(_DB_folder_path),
        config_name(_config_name),
        db(_DB_folder_path + "wind.db") {}

  ~wind_parser() {}

  std::vector<wind_db_data> parse_table(const double start_time,
                                        const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["wind"].get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from wind where ID= ?;";

    //
    std::vector<wind_db_data> v_wind_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from wind;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, double speed, double orientation) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_wind_db_data.push_back(wind_db_data{
                  _local_time_s,  // local_time
                  speed,          // speed
                  orientation     // orientation
              });
            }
          };
    }
    return v_wind_db_data;
  }

 private:
  std::string config_name;
  sqlite::database db;

};  // end class wind_parser

class stm32_parser : public master_parser {
 public:
  explicit stm32_parser(const std::string &_DB_folder_path,
                        const std::string &_config_name)
      : master_parser(_DB_folder_path),
        config_name(_config_name),
        db(_DB_folder_path + "stm32.db") {}

  ~stm32_parser() {}

  std::vector<stm32_db_data> parse_table(const double start_time,
                                         const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["stm32"].get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from stm32 where ID= ?;";

    //
    std::vector<stm32_db_data> v_stm32_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from stm32;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, int stm32_link, int stm32_status,
              double command_u1, double command_u2, double feedback_u1,
              double feedback_u2, int feedback_pwm1, int feedback_pwm2,
              double RC_X, double RC_Y, double RC_Mz, double voltage_b1,
              double voltage_b2, double voltage_b3) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_stm32_db_data.push_back(stm32_db_data{
                  _local_time_s,  // local_time
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
              });
            }
          };
    }
    return v_stm32_db_data;
  }

 private:
  std::string config_name;
  sqlite::database db;

};  // end class stm32_parser

class marineradar_parser : public master_parser {
 public:
  explicit marineradar_parser(const std::string &_DB_folder_path,
                              const std::string &_config_name)
      : master_parser(_DB_folder_path),
        config_name(_config_name),
        db(_DB_folder_path + "marineradar.db") {}

  ~marineradar_parser() {}

  std::vector<marineradar_db_data> parse_table(const double start_time,
                                               const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["marineradar"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from radar where ID= ?;";

    //
    std::vector<marineradar_db_data> v_marineradar_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from radar;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, double azimuth_deg, double sample_range,
              std::vector<uint8_t> spokedata) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_marineradar_db_data.push_back(marineradar_db_data{
                  _local_time_s,  // local_time
                  azimuth_deg,    // azimuth_deg
                  sample_range,   // sample_range
                  spokedata       // spokedata
              });
            }
          };
    }
    return v_marineradar_db_data;
  }

 private:
  std::string config_name;
  sqlite::database db;

};  // end class marineradar_parser

class estimator_parser : public master_parser {
 public:
  explicit estimator_parser(const std::string &_DB_folder_path,
                            const std::string &_config_name)
      : master_parser(_DB_folder_path),
        config_name(_config_name),
        db(_DB_folder_path + "estimator.db") {}

  ~estimator_parser() {}

  std::vector<est_measurement_db_data> parse_measurement_table(
      const double start_time, const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["estimator"]["measurement"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from measurement where ID= ?;";

    //
    std::vector<est_measurement_db_data> v_est_measurement_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from measurement;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, double meas_x, double meas_y,
              double meas_theta, double meas_u, double meas_v, double meas_r) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_est_measurement_db_data.push_back(est_measurement_db_data{
                  _local_time_s,  // local_time
                  meas_x,         // meas_x
                  meas_y,         // meas_y
                  meas_theta,     // meas_theta
                  meas_u,         // meas_u
                  meas_v,         // meas_v
                  meas_r          // meas_r
              });
            }
          };
    }
    return v_est_measurement_db_data;
  }  // parse_measurement_table

  std::vector<est_state_db_data> parse_state_table(const double start_time,
                                                   const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["estimator"]["state"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from state where ID= ?;";

    //
    std::vector<est_state_db_data> v_est_state_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from state;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >>
          [&](std::string local_time, double state_x, double state_y,
              double state_theta, double state_u, double state_v,
              double state_r, double curvature, double speed, double dspeed) {
            double _local_time_s =
                86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
            if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
              v_est_state_db_data.push_back(est_state_db_data{
                  _local_time_s,  // local_time
                  state_x,        // state_x
                  state_y,        // state_y
                  state_theta,    // state_theta
                  state_u,        // state_u
                  state_v,        // state_v
                  state_r,        // state_r
                  curvature,      // curvature
                  speed,          // speed
                  dspeed          // dspeed
              });
            }
          };
    }
    return v_est_state_db_data;
  }  // parse_state_table

  std::vector<est_error_db_data> parse_error_table(const double start_time,
                                                   const double end_time) {
    // parse config file
    std::string parse_string = "select DATETIME";
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["estimator"]["error"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    for (auto const &value : db_config) parse_string += ", " + value.first;
    parse_string += " from error where ID= ?;";

    //
    std::vector<est_error_db_data> v_est_error_db_data;

    // loop through the database
    int max_id = 0;
    db << "select MAX(ID) from error;" >> max_id;
    for (int i = 0; i != max_id; i++) {
      db << parse_string << i + 1 >> [&](std::string local_time,
                                         double perror_x, double perror_y,
                                         double perror_mz, double verror_x,
                                         double verror_y, double verror_mz) {
        double _local_time_s =
            86400 * (atof(local_time.c_str()) - master_parser::timestamp0);
        if ((start_time <= _local_time_s) && (_local_time_s <= end_time)) {
          v_est_error_db_data.push_back(est_error_db_data{
              _local_time_s,  // local_time
              perror_x,       // perror_x
              perror_y,       // perror_y
              perror_mz,      // perror_mz
              verror_x,       // verror_x
              verror_y,       // verror_y
              verror_mz       // verror_mz
          });
        }
      };
    }
    return v_est_error_db_data;
  }  // parse_error_table

 private:
  std::string config_name;
  sqlite::database db;

};  // end class estimator_parser

}  // namespace ASV::common

#endif /* _DATAPARSER_H_ */