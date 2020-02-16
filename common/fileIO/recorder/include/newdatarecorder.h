/*
***********************************************************************
* datarecorder.h:
* database using sqlite3 and sqlite modern cpp wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DATARECORDER_H_
#define _DATARECORDER_H_

#include <sqlite_modern_cpp.h>
#include <common/math/eigen/Eigen/Core>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include "common/fileIO/include/json.hpp"
#include "common/logging/include/easylogging++.h"

namespace ASV::common {

/********************************* messages **********************************/
/*sensors(GPS, IMU, marine radar, etc)          */
class gps_db {
 public:
  explicit gps_db(const std::string &_DB_folder_path,
                  const std::string &_config_name)
      : dbpath(_DB_folder_path + "gps.db"),
        config_name(_config_name),
        insert_string(""),
        db(dbpath) {}
  ~gps_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["GPS"].get<std::vector<std::pair<std::string, std::string>>>();
    try {
      std::string str =
          "CREATE TABLE GPS"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string = "(DATETIME";

      for (auto const &[name, type] : db_config) {
        str += ", " + name + " " + type;
        insert_string += ", " + name;
      }
      str += ");";
      insert_string += ") ";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-GPS") << e.what();
    }
  }  // create_table

  void update_table(double UTC = 0, double latitude = 0, double longitude = 0,
                    double heading = 0, double pitch = 0, double roll = 0,
                    double altitude = 0, double Ve = 0, double Vn = 0,
                    double roti = 0, int status = 0, double UTM_x = 0,
                    double UTM_y = 0, const std::string &UTM_zone = "") {
    try {
      std::string str = "INSERT INTO GPS";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(UTC);
      str += ", ";
      str += std::to_string(latitude);
      str += ", ";
      str += std::to_string(longitude);
      str += ", ";
      str += std::to_string(heading);
      str += ", ";
      str += std::to_string(pitch);
      str += ", ";
      str += std::to_string(roll);
      str += ", ";
      str += std::to_string(altitude);
      str += ", ";
      str += std::to_string(Ve);
      str += ", ";
      str += std::to_string(Vn);
      str += ", ";
      str += std::to_string(roti);
      str += ", ";
      str += std::to_string(status);
      str += ", ";
      str += std::to_string(UTM_x);
      str += ", ";
      str += std::to_string(UTM_y);
      str += ", '";
      str += UTM_zone;
      str += "');";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-GPS") << e.what();
    }
  }  // update_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string;
  sqlite::database db;

};  // end class gps_db

class wind_db {
 public:
  explicit wind_db(const std::string &_DB_folder_path,
                   const std::string &_config_name)
      : dbpath(_DB_folder_path + "wind.db"),
        config_name(_config_name),
        insert_string(""),
        db(dbpath) {}
  ~wind_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["wind"].get<std::vector<std::pair<std::string, std::string>>>();
    try {
      std::string str =
          "CREATE TABLE wind"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string = "(DATETIME";

      for (auto const &[name, type] : db_config) {
        str += ", " + name + " " + type;
        insert_string += ", " + name;
      }
      str += ");";
      insert_string += ") ";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-wind") << e.what();
    }
  }  // create_table

  void update_table(double speed = 0, double orientation = 0) {
    try {
      std::string str = "INSERT INTO wind";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(speed);
      str += ", ";
      str += std::to_string(orientation);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-wind") << e.what();
    }
  }  // update_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string;
  sqlite::database db;

};  // end class wind_db

class stm32_db {
 public:
  explicit stm32_db(const std::string &_DB_folder_path,
                    const std::string &_config_name)
      : dbpath(_DB_folder_path + "stm32.db"),
        config_name(_config_name),
        insert_string(""),
        db(dbpath) {}
  ~stm32_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["stm32"].get<std::vector<std::pair<std::string, std::string>>>();
    try {
      std::string str =
          "CREATE TABLE stm32"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string = "(DATETIME";

      for (auto const &[name, type] : db_config) {
        str += ", " + name + " " + type;
        insert_string += ", " + name;
      }
      str += ");";
      insert_string += ") ";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-stm32") << e.what();
    }
  }  // create_table

  void update_table(int stm32_link = 0, int stm32_status = 0,
                    double command_u1 = 0, double command_u2 = 0,
                    double feedback_u1 = 0, double feedback_u2 = 0,
                    int feedback_pwm1 = 0, int feedback_pwm2 = 0,
                    double RC_X = 0, double RC_Y = 0, double RC_Mz = 0,
                    double voltage_b1 = 0, double voltage_b2 = 0,
                    double voltage_b3 = 0) {
    try {
      std::string str = "INSERT INTO stm32";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(stm32_link);
      str += ", ";
      str += std::to_string(stm32_status);
      str += ", ";
      str += std::to_string(command_u1);
      str += ", ";
      str += std::to_string(command_u2);
      str += ", ";
      str += std::to_string(feedback_u1);
      str += ", ";
      str += std::to_string(feedback_u2);
      str += ", ";
      str += std::to_string(feedback_pwm1);
      str += ", ";
      str += std::to_string(feedback_pwm2);
      str += ", ";
      str += std::to_string(RC_X);
      str += ", ";
      str += std::to_string(RC_Y);
      str += ", ";
      str += std::to_string(RC_Mz);
      str += ", ";
      str += std::to_string(voltage_b1);
      str += ", ";
      str += std::to_string(voltage_b2);
      str += ", ";
      str += std::to_string(voltage_b3);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-stm32") << e.what();
    }
  }  // update_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string;
  sqlite::database db;

};  // end class stm32_db

class marineradar_db {
 public:
  explicit marineradar_db(const std::string &_DB_folder_path,
                          const std::string &_config_name)
      : dbpath(_DB_folder_path + "marineradar.db"),
        config_name(_config_name),
        insert_string(""),
        db(dbpath) {}
  ~marineradar_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["marineradar"].get<std::map<std::string, std::string>>();
    try {
      std::string str =
          "CREATE TABLE radar"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string = "(DATETIME";

      for (auto const &[name, type] : db_config) {
        str += ", " + name + " " + type;
        insert_string += ", " + name;
      }
      str += ");";
      insert_string += ") ";
      std::cout << str << std::endl;

      std::cout << insert_string << std::endl;

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-marineradar") << e.what();
    }
  }  // create_table

  void update_table(double azimuth_deg, double sample_range,
                    const std::size_t size_spokedata,
                    const uint8_t *spokedata) {
    try {
      std::string str = "INSERT INTO radar";
      str += insert_string;
      str += "VALUES(julianday('now'),? ,? ,? )";

      std::vector<uint8_t> data(&spokedata[0], &spokedata[size_spokedata]);
      db << str << azimuth_deg << sample_range << data;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-marineradar") << e.what();
    }
  }  // update_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string;
  sqlite::database db;

};  // end class marineradar_db

/********************************* Modules **********************************/
/* perception, planner, controller, estimator(GPS, IMU, marine radar, etc) */
class estimator_db {
 public:
  explicit estimator_db(const std::string &_DB_folder_path,
                        const std::string &_config_name)
      : dbpath(_DB_folder_path + "estimator.db"),
        config_name(_config_name),
        insert_string_measurement(""),
        insert_string_state(""),
        insert_string_error(""),
        db(dbpath) {}
  ~estimator_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config_measurement = file["estimator"]["measurement"]
                                     .get<std::map<std::string, std::string>>();
    auto db_config_state =
        file["estimator"]["state"].get<std::map<std::string, std::string>>();
    auto db_config_error =
        file["estimator"]["error"].get<std::map<std::string, std::string>>();
    try {
      // measurement
      std::string str =
          "CREATE TABLE measurement"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_measurement = "(DATETIME";

      for (auto const &[name, type] : db_config_measurement) {
        str += ", " + name + " " + type;
        insert_string_measurement += ", " + name;
      }
      str += ");";
      insert_string_measurement += ") ";

      db << str;

      // state
      str =
          "CREATE TABLE state"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";

      insert_string_state = "(DATETIME";

      for (auto const &[name, type] : db_config_state) {
        str += ", " + name + " " + type;
        insert_string_state += ", " + name;
      }
      str += ");";
      insert_string_state += ") ";
      db << str;

      // error
      str =
          "CREATE TABLE error"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      insert_string_error = "(DATETIME";

      for (auto const &[name, type] : db_config_error) {
        str += ", " + name + " " + type;
        insert_string_error += ", " + name;
      }
      str += ");";
      insert_string_error += ") ";

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // create_table

  void update_measurement_table(double meas_x = 0, double meas_y = 0,
                                double meas_theta = 0, double meas_u = 0,
                                double meas_v = 0, double meas_r = 0) {
    try {
      std::string str = "INSERT INTO measurement";
      str += insert_string_measurement;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(meas_x);
      str += ", ";
      str += std::to_string(meas_y);
      str += ", ";
      str += std::to_string(meas_theta);
      str += ", ";
      str += std::to_string(meas_u);
      str += ", ";
      str += std::to_string(meas_v);
      str += ", ";
      str += std::to_string(meas_r);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_measurement_table

  void update_state_table(double state_x = 0, double state_y = 0,
                          double state_theta = 0, double state_u = 0,
                          double state_v = 0, double state_r = 0,
                          double curvature = 0, double speed = 0,
                          double dspeed = 0) {
    try {
      std::string str = "INSERT INTO state";
      str += insert_string_state;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(state_x);
      str += ", ";
      str += std::to_string(state_y);
      str += ", ";
      str += std::to_string(state_theta);
      str += ", ";
      str += std::to_string(state_u);
      str += ", ";
      str += std::to_string(state_v);
      str += ", ";
      str += std::to_string(state_r);
      str += ", ";
      str += std::to_string(curvature);
      str += ", ";
      str += std::to_string(speed);
      str += ", ";
      str += std::to_string(dspeed);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_state_table

  void update_error_table(double perror_x = 0, double perror_y = 0,
                          double perror_mz = 0, double verror_x = 0,
                          double verror_y = 0, double verror_mz = 0) {
    try {
      std::string str = "INSERT INTO error";
      str += insert_string_error;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(perror_x);
      str += ", ";
      str += std::to_string(perror_y);
      str += ", ";
      str += std::to_string(perror_mz);
      str += ", ";
      str += std::to_string(verror_x);
      str += ", ";
      str += std::to_string(verror_y);
      str += ", ";
      str += std::to_string(verror_mz);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_error_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string_measurement;
  std::string insert_string_state;
  std::string insert_string_error;
  sqlite::database db;

};  // end class estimator_db

}  // namespace ASV::common

#endif /* _DATARECORDER_H_ */