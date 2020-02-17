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

  void update_routeplanner_table(double speed = 0, double captureradius = 0,
                                 double WPX = 0, double WPY = 0,
                                 double WPLONG = 0, double WPLAT = 0) {
    try {
      std::string str = "INSERT INTO routeplanner";
      str += insert_string_routeplanner;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(speed);
      str += ", ";
      str += std::to_string(captureradius);
      str += ", ";
      str += std::to_string(WPX);
      str += ", ";
      str += std::to_string(WPY);
      str += ", ";
      str += std::to_string(WPLONG);
      str += ", ";
      str += std::to_string(WPLAT);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-planner") << e.what();
    }
  }  // update_routeplanner_table

  void update_latticeplanner_table(double lattice_x = 0, double lattice_y = 0,
                                   double lattice_theta = 0,
                                   double lattice_kappa = 0,
                                   double lattice_speed = 0,
                                   double lattice_dspeed = 0) {
    try {
      std::string str = "INSERT INTO latticeplanner";
      str += insert_string_latticeplanner;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(lattice_x);
      str += ", ";
      str += std::to_string(lattice_y);
      str += ", ";
      str += std::to_string(lattice_theta);
      str += ", ";
      str += std::to_string(lattice_kappa);
      str += ", ";
      str += std::to_string(lattice_speed);
      str += ", ";
      str += std::to_string(lattice_dspeed);
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

  void update_setpoint_table(double set_x = 0, double set_y = 0,
                             double set_theta = 0, double set_u = 0,
                             double set_v = 0, double set_r = 0) {
    try {
      std::string str = "INSERT INTO setpoint";
      str += insert_string_setpoint;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(set_x);
      str += ", ";
      str += std::to_string(set_y);
      str += ", ";
      str += std::to_string(set_theta);
      str += ", ";
      str += std::to_string(set_u);
      str += ", ";
      str += std::to_string(set_v);
      str += ", ";
      str += std::to_string(set_r);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }  // update_setpoint_table

  template <int num_thruster>
  void update_TA_table(double desired_Fx, double desired_Fy, double desired_Mz,
                       double est_Fx, double est_Fy, double est_Mz,
                       const Eigen::Matrix<int, num_thruster, 1> &_alpha,
                       const Eigen::Matrix<int, num_thruster, 1> &_rpm) {
    try {
      std::string str = "INSERT INTO TA";
      str += insert_string_TA;
      str += "VALUES(julianday('now'),? ,? ,? ,? ,? ,? ,? ,?)";

      std::vector<int> alpha(_alpha.data(), _alpha.data() + num_thruster);
      std::vector<int> rpm(_rpm.data(), _rpm.data() + num_thruster);
      db << str << desired_Fx << desired_Fy << desired_Mz << est_Fx << est_Fy
         << est_Mz << alpha << rpm;

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

  void update_spoke_table(const std::vector<double> &surroundings_bearing_rad,
                          const std::vector<double> &surroundings_range_m,
                          const std::vector<double> &surroundings_x_m,
                          const std::vector<double> &surroundings_y_m) {
    try {
      std::string str = "INSERT INTO SpokeProcess";
      str += insert_string_spoke;
      str += "VALUES(julianday('now'), ?, ?, ?, ?)";

      db << str << surroundings_bearing_rad << surroundings_range_m
         << surroundings_x_m << surroundings_y_m;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // update_spoke_table

  void update_detection_table(
      const std::vector<double> &detected_target_x,
      const std::vector<double> &detected_target_y,
      const std::vector<double> &detected_target_radius) {
    try {
      std::string str = "INSERT INTO DetectedTarget";
      str += insert_string_detectedtarget;
      str += "VALUES(julianday('now'), ?, ?, ?)";

      db << str << detected_target_x << detected_target_y
         << detected_target_radius;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-perception") << e.what();
    }
  }  // update_detection_table

  template <int num_target>
  void update_trackingtarget_table(
      const int spoke_state,
      const Eigen::Matrix<int, num_target, 1> &targets_state,
      const Eigen::Matrix<int, num_target, 1> &targets_intention,
      const Eigen::Matrix<double, num_target, 1> &targets_x,
      const Eigen::Matrix<double, num_target, 1> &targets_y,
      const Eigen::Matrix<double, num_target, 1> &targets_square_radius,
      const Eigen::Matrix<double, num_target, 1> &targets_vx,
      const Eigen::Matrix<double, num_target, 1> &targets_vy,
      const Eigen::Matrix<double, num_target, 1> &targets_CPA_x,
      const Eigen::Matrix<double, num_target, 1> &targets_CPA_y,
      const Eigen::Matrix<double, num_target, 1> &targets_TCPA) {
    try {
      std::string str = "INSERT INTO TrackingTarget";
      str += insert_string_trackingtarget;
      str += "VALUES(julianday('now'),? ,? ,? ,? ,? ,? ,? ,?, ?, ?, ?)";

      db << str << spoke_state
         << std::vector<int>(targets_state.data(),
                             targets_state.data() + num_target)
         << std::vector<int>(targets_intention.data(),
                             targets_intention.data() + num_target)
         << std::vector<double>(targets_x.data(), targets_x.data() + num_target)
         << std::vector<double>(targets_y.data(), targets_y.data() + num_target)
         << std::vector<double>(targets_square_radius.data(),
                                targets_square_radius.data() + num_target)
         << std::vector<double>(targets_vx.data(),
                                targets_vx.data() + num_target)
         << std::vector<double>(targets_vy.data(),
                                targets_vy.data() + num_target)
         << std::vector<double>(targets_CPA_x.data(),
                                targets_CPA_x.data() + num_target)
         << std::vector<double>(targets_CPA_y.data(),
                                targets_CPA_y.data() + num_target)
         << std::vector<double>(targets_TCPA.data(),
                                targets_TCPA.data() + num_target);

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