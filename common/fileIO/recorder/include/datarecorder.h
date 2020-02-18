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

#ifndef _DATARECORDER_H_
#define _DATARECORDER_H_

#include <sqlite_modern_cpp.h>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include "common/fileIO/include/json.hpp"
#include "common/logging/include/easylogging++.h"
#include "databasedata.h"

namespace ASV::common {

class master_db {
 public:
  explicit master_db(const std::string &_DB_folder_path)
      : dbpath(_DB_folder_path + "master.db") {
    sqlite::database db(dbpath);

    db << "CREATE TABLE IF NOT EXISTS info "
          "(ID INTEGER PRIMARY KEY, DATETIME TEXT NOT NULL);";
    db << "INSERT OR IGNORE INTO info (ID, DATETIME)"
          " VALUES(1, julianday('now'));";
  }
  virtual ~master_db() = default;

 private:
  std::string dbpath;

};  // end class master_db

/********************************* messages **********************************/
/*sensors(GPS, IMU, marine radar, etc)          */
class gps_db : public master_db {
 public:
  explicit gps_db(const std::string &_DB_folder_path,
                  const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "gps.db"),
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

  void update_table(const gps_db_data &update_data) {
    try {
      std::string str = "INSERT INTO GPS";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.UTC);
      str += ", ";
      str += std::to_string(update_data.latitude);
      str += ", ";
      str += std::to_string(update_data.longitude);
      str += ", ";
      str += std::to_string(update_data.heading);
      str += ", ";
      str += std::to_string(update_data.pitch);
      str += ", ";
      str += std::to_string(update_data.roll);
      str += ", ";
      str += std::to_string(update_data.altitude);
      str += ", ";
      str += std::to_string(update_data.Ve);
      str += ", ";
      str += std::to_string(update_data.Vn);
      str += ", ";
      str += std::to_string(update_data.roti);
      str += ", ";
      str += std::to_string(update_data.status);
      str += ", ";
      str += std::to_string(update_data.UTM_x);
      str += ", ";
      str += std::to_string(update_data.UTM_y);
      str += ", '";
      str += update_data.UTM_zone;
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

class wind_db : public master_db {
 public:
  explicit wind_db(const std::string &_DB_folder_path,
                   const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "wind.db"),
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

  void update_table(const wind_db_data &update_data) {
    try {
      std::string str = "INSERT INTO wind";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.speed);
      str += ", ";
      str += std::to_string(update_data.orientation);
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

class stm32_db : public master_db {
 public:
  explicit stm32_db(const std::string &_DB_folder_path,
                    const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "stm32.db"),
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

  void update_table(const stm32_db_data &update_data) {
    try {
      std::string str = "INSERT INTO stm32";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.stm32_link);
      str += ", ";
      str += std::to_string(update_data.stm32_status);
      str += ", ";
      str += std::to_string(update_data.command_u1);
      str += ", ";
      str += std::to_string(update_data.command_u2);
      str += ", ";
      str += std::to_string(update_data.feedback_u1);
      str += ", ";
      str += std::to_string(update_data.feedback_u2);
      str += ", ";
      str += std::to_string(update_data.feedback_pwm1);
      str += ", ";
      str += std::to_string(update_data.feedback_pwm2);
      str += ", ";
      str += std::to_string(update_data.RC_X);
      str += ", ";
      str += std::to_string(update_data.RC_Y);
      str += ", ";
      str += std::to_string(update_data.RC_Mz);
      str += ", ";
      str += std::to_string(update_data.voltage_b1);
      str += ", ";
      str += std::to_string(update_data.voltage_b2);
      str += ", ";
      str += std::to_string(update_data.voltage_b3);
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

class marineradar_db : public master_db {
 public:
  explicit marineradar_db(const std::string &_DB_folder_path,
                          const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "marineradar.db"),
        config_name(_config_name),
        insert_string(""),
        db(dbpath) {}
  ~marineradar_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config =
        file["marineradar"]
            .get<std::vector<std::pair<std::string, std::string>>>();
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

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-marineradar") << e.what();
    }
  }  // create_table

  void update_table(const marineradar_db_data &update_data) {
    try {
      std::string str = "INSERT INTO radar";
      str += insert_string;
      str += "VALUES(julianday('now'),? ,? ,? )";

      db << str << update_data.azimuth_deg << update_data.sample_range
         << update_data.spokedata;

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
class estimator_db : public master_db {
 public:
  explicit estimator_db(const std::string &_DB_folder_path,
                        const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "estimator.db"),
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
    auto db_config_measurement =
        file["estimator"]["measurement"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_state =
        file["estimator"]["state"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_error =
        file["estimator"]["error"]
            .get<std::vector<std::pair<std::string, std::string>>>();
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
          " DATETIME    TEXT       NOT NULL";
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
          " DATETIME    TEXT       NOT NULL";
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

  void update_measurement_table(const est_measurement_db_data &update_data) {
    try {
      std::string str = "INSERT INTO measurement";
      str += insert_string_measurement;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.meas_x);
      str += ", ";
      str += std::to_string(update_data.meas_y);
      str += ", ";
      str += std::to_string(update_data.meas_theta);
      str += ", ";
      str += std::to_string(update_data.meas_u);
      str += ", ";
      str += std::to_string(update_data.meas_v);
      str += ", ";
      str += std::to_string(update_data.meas_r);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_measurement_table

  void update_state_table(const est_state_db_data &update_data) {
    try {
      std::string str = "INSERT INTO state";
      str += insert_string_state;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.state_x);
      str += ", ";
      str += std::to_string(update_data.state_y);
      str += ", ";
      str += std::to_string(update_data.state_theta);
      str += ", ";
      str += std::to_string(update_data.state_u);
      str += ", ";
      str += std::to_string(update_data.state_v);
      str += ", ";
      str += std::to_string(update_data.state_r);
      str += ", ";
      str += std::to_string(update_data.curvature);
      str += ", ";
      str += std::to_string(update_data.speed);
      str += ", ";
      str += std::to_string(update_data.dspeed);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_state_table

  void update_error_table(const est_error_db_data &update_data) {
    try {
      std::string str = "INSERT INTO error";
      str += insert_string_error;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.perror_x);
      str += ", ";
      str += std::to_string(update_data.perror_y);
      str += ", ";
      str += std::to_string(update_data.perror_mz);
      str += ", ";
      str += std::to_string(update_data.verror_x);
      str += ", ";
      str += std::to_string(update_data.verror_y);
      str += ", ";
      str += std::to_string(update_data.verror_mz);
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

class planner_db : public master_db {
 public:
  explicit planner_db(const std::string &_DB_folder_path,
                      const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "planner.db"),
        config_name(_config_name),
        insert_string_routeplanner(""),
        insert_string_latticeplanner(""),
        db(dbpath) {}
  ~planner_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config_routeplanner =
        file["planner"]["routeplanner"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_latticeplanner =
        file["planner"]["latticeplanner"]
            .get<std::vector<std::pair<std::string, std::string>>>();

    try {
      // routeplanner
      std::string str =
          "CREATE TABLE routeplanner"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_routeplanner = "(DATETIME";

      for (auto const &[name, type] : db_config_routeplanner) {
        str += ", " + name + " " + type;
        insert_string_routeplanner += ", " + name;
      }
      str += ");";
      insert_string_routeplanner += ") ";
      db << str;

      // latticeplanner
      str =
          "CREATE TABLE latticeplanner"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_latticeplanner = "(DATETIME";

      for (auto const &[name, type] : db_config_latticeplanner) {
        str += ", " + name + " " + type;
        insert_string_latticeplanner += ", " + name;
      }
      str += ");";
      insert_string_latticeplanner += ") ";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-planner") << e.what();
    }
  }  // create_table

  void update_routeplanner_table(const plan_route_db_data &update_data) {
    try {
      std::string str = "INSERT INTO routeplanner";
      str += insert_string_routeplanner;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.speed);
      str += ", ";
      str += std::to_string(update_data.captureradius);
      str += ", ";
      str += std::to_string(update_data.WPX);
      str += ", ";
      str += std::to_string(update_data.WPY);
      str += ", ";
      str += std::to_string(update_data.WPLONG);
      str += ", ";
      str += std::to_string(update_data.WPLAT);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-planner") << e.what();
    }
  }  // update_routeplanner_table

  void update_latticeplanner_table(const plan_lattice_db_data &update_data) {
    try {
      std::string str = "INSERT INTO latticeplanner";
      str += insert_string_latticeplanner;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.lattice_x);
      str += ", ";
      str += std::to_string(update_data.lattice_y);
      str += ", ";
      str += std::to_string(update_data.lattice_theta);
      str += ", ";
      str += std::to_string(update_data.lattice_kappa);
      str += ", ";
      str += std::to_string(update_data.lattice_speed);
      str += ", ";
      str += std::to_string(update_data.lattice_dspeed);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-planner") << e.what();
    }
  }  // update_latticeplanner_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string_routeplanner;
  std::string insert_string_latticeplanner;

  sqlite::database db;

};  // end class planner_db

class controller_db : public master_db {
 public:
  explicit controller_db(const std::string &_DB_folder_path,
                         const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "controller.db"),
        config_name(_config_name),
        insert_string_setpoint(""),
        insert_string_TA(""),
        db(dbpath) {}
  ~controller_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config_setpoint =
        file["controller"]["setpoint"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_TA =
        file["controller"]["TA"]
            .get<std::vector<std::pair<std::string, std::string>>>();

    try {
      // setpoints
      std::string str =
          "CREATE TABLE setpoint"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_setpoint = "(DATETIME";

      for (auto const &[name, type] : db_config_setpoint) {
        str += ", " + name + " " + type;
        insert_string_setpoint += ", " + name;
      }
      str += ");";
      insert_string_setpoint += ") ";
      db << str;

      // thrust allocation
      str =
          "CREATE TABLE TA"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_TA = "(DATETIME";

      for (auto const &[name, type] : db_config_TA) {
        str += ", " + name + " " + type;
        insert_string_TA += ", " + name;
      }
      str += ");";
      insert_string_TA += ") ";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }  // create_table

  void update_setpoint_table(const control_setpoint_db_data &update_data) {
    try {
      std::string str = "INSERT INTO setpoint";
      str += insert_string_setpoint;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(update_data.set_x);
      str += ", ";
      str += std::to_string(update_data.set_y);
      str += ", ";
      str += std::to_string(update_data.set_theta);
      str += ", ";
      str += std::to_string(update_data.set_u);
      str += ", ";
      str += std::to_string(update_data.set_v);
      str += ", ";
      str += std::to_string(update_data.set_r);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }  // update_setpoint_table

  void update_TA_table(const control_TA_db_data &update_data) {
    try {
      std::string str = "INSERT INTO TA";
      str += insert_string_TA;
      str += "VALUES(julianday('now'),? ,? ,? ,? ,? ,? ,? ,?)";
      db << str << update_data.desired_Fx << update_data.desired_Fy
         << update_data.desired_Mz << update_data.est_Fx << update_data.est_Fy
         << update_data.est_Mz << update_data.alpha << update_data.rpm;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }  // update_TA_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string_setpoint;
  std::string insert_string_TA;
  sqlite::database db;

};  // end class controller_db

class perception_db : public master_db {
 public:
  explicit perception_db(const std::string &_DB_folder_path,
                         const std::string &_config_name)
      : master_db(_DB_folder_path),
        dbpath(_DB_folder_path + "perception.db"),
        config_name(_config_name),
        insert_string_spoke(""),
        insert_string_detectedtarget(""),
        insert_string_trackingtarget(""),
        db(dbpath) {}
  ~perception_db() {}

  void create_table() {
    std::ifstream in(config_name);
    nlohmann::json file;
    in >> file;
    auto db_config_SpokeProcess =
        file["perception"]["SpokeProcess"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_DetectedTarget =
        file["perception"]["DetectedTarget"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    auto db_config_TrackingTarget =
        file["perception"]["TrackingTarget"]
            .get<std::vector<std::pair<std::string, std::string>>>();
    try {
      // spoke process
      std::string str =
          "CREATE TABLE SpokeProcess"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_spoke = "(DATETIME";

      for (auto const &[name, type] : db_config_SpokeProcess) {
        str += ", " + name + " " + type;
        insert_string_spoke += ", " + name;
      }
      str += ");";
      insert_string_spoke += ") ";
      db << str;

      // Detected target
      str =
          "CREATE TABLE DetectedTarget"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_detectedtarget = "(DATETIME";

      for (auto const &[name, type] : db_config_DetectedTarget) {
        str += ", " + name + " " + type;
        insert_string_detectedtarget += ", " + name;
      }
      str += ");";
      insert_string_detectedtarget += ") ";
      db << str;

      // Tracking targets
      str =
          "CREATE TABLE TrackingTarget"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";
      insert_string_trackingtarget = "(DATETIME";

      for (auto const &[name, type] : db_config_TrackingTarget) {
        str += ", " + name + " " + type;
        insert_string_trackingtarget += ", " + name;
      }
      str += ");";
      insert_string_trackingtarget += ") ";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // create_table

  void update_spoke_table(const perception_spoke_db_data &update_data) {
    try {
      std::string str = "INSERT INTO SpokeProcess";
      str += insert_string_spoke;
      str += "VALUES(julianday('now'), ?, ?, ?, ?)";

      db << str << update_data.surroundings_bearing_rad
         << update_data.surroundings_range_m << update_data.surroundings_x_m
         << update_data.surroundings_y_m;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // update_spoke_table

  void update_detection_table(const perception_detection_db_data &update_data) {
    try {
      std::string str = "INSERT INTO DetectedTarget";
      str += insert_string_detectedtarget;
      str += "VALUES(julianday('now'), ?, ?, ?)";

      db << str << update_data.detected_target_x
         << update_data.detected_target_y << update_data.detected_target_radius;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // update_detection_table

  void update_trackingtarget_table(
      const perception_trackingtarget_db_data &update_data) {
    try {
      std::string str = "INSERT INTO TrackingTarget";
      str += insert_string_trackingtarget;
      str += "VALUES(julianday('now'),? ,? ,? ,? ,? ,? ,? ,?, ?, ?, ?)";

      db << str << update_data.spoke_state << update_data.targets_state
         << update_data.targets_intention << update_data.targets_x
         << update_data.targets_y << update_data.targets_square_radius
         << update_data.targets_vx << update_data.targets_vy
         << update_data.targets_CPA_x << update_data.targets_CPA_y
         << update_data.targets_TCPA;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // update_TA_table

 private:
  std::string dbpath;
  std::string config_name;
  std::string insert_string_spoke;
  std::string insert_string_detectedtarget;
  std::string insert_string_trackingtarget;

  sqlite::database db;

};  // end class perception_db

}  // namespace ASV::common

#endif /* _DATARECORDER_H_ */