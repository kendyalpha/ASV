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
#include "common/logging/include/easylogging++.h"

#define VARIABLENAME4DB(variable) #variable

namespace ASV::common {

class dbinfo {
 public:
  explicit dbinfo(const std::string &_folder_path)
      : folder_path(_folder_path),
        type_names({{std::type_index(typeid(int)), "INT"},
                    {std::type_index(typeid(double)), "DOUBLE"},
                    {std::type_index(typeid(std::string)), "TEXT"},
                    {std::type_index(typeid(std::vector<double>)), "BLOB"},
                    {std::type_index(typeid(std::vector<int>)), "BLOB"}}) {}
  virtual ~dbinfo() = default;

 protected:
  std::string folder_path;
  std::unordered_map<std::type_index, std::string> type_names;
};  // end class database

/********************************* messages **********************************/
/*sensors(GPS, IMU, marine radar, etc)          */
class gps_db : public dbinfo {
 public:
  explicit gps_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "gps.db"),
        insert_string(""),
        db(dbpath) {}
  ~gps_db() {}

  void create_table(double UTC = 0, double latitude = 0, double longitude = 0,
                    double heading = 0, double pitch = 0, double roll = 0,
                    double altitude = 0, double Ve = 0, double Vn = 0,
                    double roti = 0, int status = 0, double UTM_x = 0,
                    double UTM_y = 0, const std::string &UTM_zone = "") {
    try {
      std::string s_UTC = VARIABLENAME4DB(UTC);
      std::string s_latitude = VARIABLENAME4DB(latitude);
      std::string s_longitude = VARIABLENAME4DB(longitude);
      std::string s_heading = VARIABLENAME4DB(heading);
      std::string s_pitch = VARIABLENAME4DB(pitch);
      std::string s_roll = VARIABLENAME4DB(roll);
      std::string s_altitude = VARIABLENAME4DB(altitude);
      std::string s_Ve = VARIABLENAME4DB(Ve);
      std::string s_Vn = VARIABLENAME4DB(Vn);
      std::string s_roti = VARIABLENAME4DB(roti);
      std::string s_status = VARIABLENAME4DB(status);
      std::string s_UTM_x = VARIABLENAME4DB(UTM_x);
      std::string s_UTM_y = VARIABLENAME4DB(UTM_y);
      std::string s_UTM_zone = VARIABLENAME4DB(UTM_zone);

      std::string str =
          "CREATE TABLE GPS"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // UTC
      str +=
          s_UTC + " " + dbinfo::type_names[std::type_index(typeid(UTC))] + ",";
      // latitude;
      str += s_latitude + " " +
             dbinfo::type_names[std::type_index(typeid(latitude))] + ",";
      // longitude;
      str += s_longitude + " " +
             dbinfo::type_names[std::type_index(typeid(longitude))] + ",";
      // heading
      str += s_heading + " " +
             dbinfo::type_names[std::type_index(typeid(heading))] + ",";
      // pitch
      str += s_pitch + " " +
             dbinfo::type_names[std::type_index(typeid(pitch))] + ",";
      // roll
      str += s_roll + " " + dbinfo::type_names[std::type_index(typeid(roll))] +
             ",";
      // altitude
      str += s_altitude + " " +
             dbinfo::type_names[std::type_index(typeid(altitude))] + ",";
      // Ve
      str += s_Ve + " " + dbinfo::type_names[std::type_index(typeid(Ve))] + ",";
      // Vn
      str += s_Vn + " " + dbinfo::type_names[std::type_index(typeid(Vn))] + ",";
      // roti
      str += s_roti + " " + dbinfo::type_names[std::type_index(typeid(roti))] +
             ",";
      // status
      str += s_status + " " +
             dbinfo::type_names[std::type_index(typeid(status))] + ",";
      // UTM_x
      str += s_UTM_x + " " +
             dbinfo::type_names[std::type_index(typeid(UTM_x))] + ",";
      // UTM_y
      str += s_UTM_y + " " +
             dbinfo::type_names[std::type_index(typeid(UTM_y))] + ",";
      // UTM_zone
      str += s_UTM_zone + " " +
             dbinfo::type_names[std::type_index(typeid(UTM_zone))];
      str += ");";

      // insert_string
      insert_string = "(DATETIME, ";
      insert_string += s_UTC + ", ";
      insert_string += s_latitude + ", ";
      insert_string += s_longitude + ", ";
      insert_string += s_heading + ", ";
      insert_string += s_pitch + ", ";
      insert_string += s_roll + ", ";
      insert_string += s_altitude + ", ";
      insert_string += s_Ve + ", ";
      insert_string += s_Vn + ", ";
      insert_string += s_roti + ", ";
      insert_string += s_status + ", ";
      insert_string += s_UTM_x + ", ";
      insert_string += s_UTM_y + ", ";
      insert_string += s_UTM_zone;
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
  std::string insert_string;
  sqlite::database db;

};  // end class gps_db

class wind_db : public dbinfo {
 public:
  explicit wind_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "wind.db"),
        insert_string(""),
        db(dbpath) {}
  ~wind_db() {}

  void create_table(double speed = 0, double orientation = 0) {
    try {
      std::string s_speed = VARIABLENAME4DB(speed);
      std::string s_orientation = VARIABLENAME4DB(orientation);

      std::string str =
          "CREATE TABLE wind"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // speed
      str += s_speed + " " +
             dbinfo::type_names[std::type_index(typeid(speed))] + ",";
      // orientation;
      str += s_orientation + " " +
             dbinfo::type_names[std::type_index(typeid(orientation))];
      str += ");";

      db << str;
      // insert_string
      insert_string = "(DATETIME, ";
      insert_string += s_speed + ", ";
      insert_string += s_orientation + ")";

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
  std::string insert_string;
  sqlite::database db;

};  // end class wind_db

class stm32_db : public dbinfo {
 public:
  explicit stm32_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "stm32.db"),
        insert_string(""),
        db(dbpath) {}
  ~stm32_db() {}

  void create_table(int stm32_link = 0, int stm32_status = 0,
                    double command_u1 = 0, double command_u2 = 0,
                    double feedback_u1 = 0, double feedback_u2 = 0,
                    int feedback_pwm1 = 0, int feedback_pwm2 = 0,
                    double RC_X = 0, double RC_Y = 0, double RC_Mz = 0,
                    double voltage_b1 = 0, double voltage_b2 = 0,
                    double voltage_b3 = 0) {
    try {
      std::string s_stm32_link = VARIABLENAME4DB(stm32_link);
      std::string s_stm32_status = VARIABLENAME4DB(stm32_status);
      std::string s_command_u1 = VARIABLENAME4DB(command_u1);
      std::string s_command_u2 = VARIABLENAME4DB(command_u2);
      std::string s_feedback_u1 = VARIABLENAME4DB(feedback_u1);
      std::string s_feedback_u2 = VARIABLENAME4DB(feedback_u2);
      std::string s_feedback_pwm1 = VARIABLENAME4DB(feedback_pwm1);
      std::string s_feedback_pwm2 = VARIABLENAME4DB(feedback_pwm2);
      std::string s_RC_X = VARIABLENAME4DB(RC_X);
      std::string s_RC_Y = VARIABLENAME4DB(RC_Y);
      std::string s_RC_Mz = VARIABLENAME4DB(RC_Mz);
      std::string s_voltage_b1 = VARIABLENAME4DB(voltage_b1);
      std::string s_voltage_b2 = VARIABLENAME4DB(voltage_b2);
      std::string s_voltage_b3 = VARIABLENAME4DB(voltage_b3);

      std::string str =
          "CREATE TABLE stm32"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // stm32_link
      str += s_stm32_link + " " +
             dbinfo::type_names[std::type_index(typeid(stm32_link))] + ",";
      // stm32_status
      str += s_stm32_status + " " +
             dbinfo::type_names[std::type_index(typeid(stm32_status))] + ",";
      // command_u1
      str += s_command_u1 + " " +
             dbinfo::type_names[std::type_index(typeid(command_u1))] + ",";
      // command_u2
      str += s_command_u2 + " " +
             dbinfo::type_names[std::type_index(typeid(command_u2))] + ",";
      // feedback_u1
      str += s_feedback_u1 + " " +
             dbinfo::type_names[std::type_index(typeid(feedback_u1))] + ",";
      // feedback_u2
      str += s_feedback_u2 + " " +
             dbinfo::type_names[std::type_index(typeid(feedback_u2))] + ",";
      // feedback_pwm1
      str += s_feedback_pwm1 + " " +
             dbinfo::type_names[std::type_index(typeid(feedback_pwm1))] + ",";
      // feedback_pwm2
      str += s_feedback_pwm2 + " " +
             dbinfo::type_names[std::type_index(typeid(feedback_pwm2))] + ",";
      // RC_X
      str += s_RC_X + " " + dbinfo::type_names[std::type_index(typeid(RC_X))] +
             ",";
      // RC_Y
      str += s_RC_Y + " " + dbinfo::type_names[std::type_index(typeid(RC_Y))] +
             ",";
      // RC_Mz
      str += s_RC_Mz + " " +
             dbinfo::type_names[std::type_index(typeid(RC_Mz))] + ",";
      // voltage_b1
      str += s_voltage_b1 + " " +
             dbinfo::type_names[std::type_index(typeid(voltage_b1))] + ",";
      // voltage_b2
      str += s_voltage_b2 + " " +
             dbinfo::type_names[std::type_index(typeid(voltage_b2))] + ",";
      // voltage_b3
      str += s_voltage_b3 + " " +
             dbinfo::type_names[std::type_index(typeid(voltage_b3))];
      str += ");";

      db << str;
      // insert_string
      insert_string = "(DATETIME, ";
      insert_string += s_stm32_link + ", ";
      insert_string += s_stm32_status + ", ";
      insert_string += s_command_u1 + ", ";
      insert_string += s_command_u2 + ", ";
      insert_string += s_feedback_u1 + ", ";
      insert_string += s_feedback_u2 + ", ";
      insert_string += s_feedback_pwm1 + ", ";
      insert_string += s_feedback_pwm2 + ", ";
      insert_string += s_RC_X + ", ";
      insert_string += s_RC_Y + ", ";
      insert_string += s_RC_Mz + ", ";
      insert_string += s_voltage_b1 + ", ";
      insert_string += s_voltage_b2 + ", ";
      insert_string += s_voltage_b3 + ")";

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
  std::string insert_string;
  sqlite::database db;

};  // end class stm32_db

class marineradar_db : public dbinfo {
 public:
  marineradar_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "marineradar.db"),
        insert_string(""),
        db(dbpath) {}
  ~marineradar_db() {}

  void create_table(double azimuth_deg = 0, double sample_range = 0) {
    try {
      // setpoints
      std::string s_azimuth_deg = VARIABLENAME4DB(azimuth_deg);
      std::string s_sample_range = VARIABLENAME4DB(sample_range);

      std::string str =
          "CREATE TABLE radar"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // azimuth_deg
      str += s_azimuth_deg + " " +
             dbinfo::type_names[std::type_index(typeid(azimuth_deg))] + ",";
      // sample_range
      str += s_sample_range + " " +
             dbinfo::type_names[std::type_index(typeid(sample_range))] + ",";
      // spoke data
      str += "SpokeData BLOB);";
      db << str;

      insert_string = "(DATETIME, ";
      insert_string += s_azimuth_deg;
      insert_string += ", ";
      insert_string += s_sample_range;
      insert_string += ", ";
      insert_string += "SpokeData)";

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-marineradar") << e.what();
    }
  }  // create_table

  template <int datasize>
  void update_table(double azimuth_deg, double sample_range,
                    const uint8_t *spokedata) {
    try {
      std::string str = "INSERT INTO radar";
      str += insert_string;
      str += "VALUES(julianday('now'),? ,? ,? )";

      std::vector<uint8_t> data(&spokedata[0], &spokedata[datasize]);
      db << str << azimuth_deg << sample_range << data;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-marineradar") << e.what();
    }
  }  // update_table

 private:
  std::string dbpath;
  std::string insert_string;
  sqlite::database db;

};  // end class marineradar_db

/********************************* Modules **********************************/
/* perception, planner, controller, estimator(GPS, IMU, marine radar, etc) */
class estimator_db : public dbinfo {
 public:
  explicit estimator_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "estimator.db"),
        insert_string_measurement(""),
        insert_string_state(""),
        insert_string_error(""),
        db(dbpath) {}
  ~estimator_db() {}

  void create_table(double meas_x = 0, double meas_y = 0, double meas_theta = 0,
                    double meas_u = 0, double meas_v = 0, double meas_r = 0,
                    double state_x = 0, double state_y = 0,
                    double state_theta = 0, double state_u = 0,
                    double state_v = 0, double state_r = 0, double perror_x = 0,
                    double perror_y = 0, double perror_mz = 0,
                    double verror_x = 0, double verror_y = 0,
                    double verror_mz = 0, double curvature = 0,
                    double speed = 0, double dspeed = 0) {
    try {
      std::string s_meas_x = VARIABLENAME4DB(meas_x);
      std::string s_meas_y = VARIABLENAME4DB(meas_y);
      std::string s_meas_theta = VARIABLENAME4DB(meas_theta);
      std::string s_meas_u = VARIABLENAME4DB(meas_u);
      std::string s_meas_v = VARIABLENAME4DB(meas_v);
      std::string s_meas_r = VARIABLENAME4DB(meas_r);

      std::string s_state_x = VARIABLENAME4DB(state_x);
      std::string s_state_y = VARIABLENAME4DB(state_y);
      std::string s_state_theta = VARIABLENAME4DB(state_theta);
      std::string s_state_u = VARIABLENAME4DB(state_u);
      std::string s_state_v = VARIABLENAME4DB(state_v);
      std::string s_state_r = VARIABLENAME4DB(state_r);
      std::string s_curvature = VARIABLENAME4DB(curvature);
      std::string s_speed = VARIABLENAME4DB(speed);
      std::string s_dspeed = VARIABLENAME4DB(dspeed);

      std::string s_perror_x = VARIABLENAME4DB(perror_x);
      std::string s_perror_y = VARIABLENAME4DB(perror_y);
      std::string s_perror_mz = VARIABLENAME4DB(perror_mz);
      std::string s_verror_x = VARIABLENAME4DB(verror_x);
      std::string s_verror_y = VARIABLENAME4DB(verror_y);
      std::string s_verror_mz = VARIABLENAME4DB(verror_mz);

      // measurement
      std::string str =
          "CREATE TABLE measurement"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // meas_x
      str += s_meas_x + " " +
             dbinfo::type_names[std::type_index(typeid(meas_x))] + ",";
      // meas_y
      str += s_meas_y + " " +
             dbinfo::type_names[std::type_index(typeid(meas_y))] + ",";
      // meas_theta
      str += s_meas_theta + " " +
             dbinfo::type_names[std::type_index(typeid(meas_theta))] + ",";
      // s_meas_u
      str += s_meas_u + " " +
             dbinfo::type_names[std::type_index(typeid(meas_u))] + ",";
      // meas_v
      str += s_meas_v + " " +
             dbinfo::type_names[std::type_index(typeid(meas_v))] + ",";
      // meas_r
      str += s_meas_r + " " +
             dbinfo::type_names[std::type_index(typeid(meas_r))] + ");";

      db << str;

      // insert_string
      insert_string_measurement = "(DATETIME, ";
      insert_string_measurement += s_meas_x + ", ";
      insert_string_measurement += s_meas_y + ", ";
      insert_string_measurement += s_meas_theta + ", ";
      insert_string_measurement += s_meas_u + ", ";
      insert_string_measurement += s_meas_v + ", ";
      insert_string_measurement += s_meas_r + ") ";

      // state
      str =
          "CREATE TABLE state"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // state_x
      str += s_state_x + " " +
             dbinfo::type_names[std::type_index(typeid(state_x))] + ",";
      // state_y
      str += s_state_y + " " +
             dbinfo::type_names[std::type_index(typeid(state_y))] + ",";
      // state_theta
      str += s_state_theta + " " +
             dbinfo::type_names[std::type_index(typeid(state_theta))] + ",";
      // state_u
      str += s_state_u + " " +
             dbinfo::type_names[std::type_index(typeid(state_u))] + ",";
      // state_v
      str += s_state_v + " " +
             dbinfo::type_names[std::type_index(typeid(state_v))] + ",";
      // state_r
      str += s_state_r + " " +
             dbinfo::type_names[std::type_index(typeid(state_r))] + ",";
      // curvature
      str += s_curvature + " " +
             dbinfo::type_names[std::type_index(typeid(curvature))] + ",";
      // speed
      str += s_speed + " " +
             dbinfo::type_names[std::type_index(typeid(speed))] + ",";
      // dspeed
      str +=
          s_dspeed + " " + dbinfo::type_names[std::type_index(typeid(dspeed))];
      str += ");";

      db << str;

      // insert_string_state
      insert_string_state = "(DATETIME, ";
      insert_string_state += s_state_x + ", ";
      insert_string_state += s_state_y + ", ";
      insert_string_state += s_state_theta + ", ";
      insert_string_state += s_state_u + ", ";
      insert_string_state += s_state_v + ", ";
      insert_string_state += s_state_r + ", ";
      insert_string_state += s_curvature + ", ";
      insert_string_state += s_speed + ", ";
      insert_string_state += s_dspeed + ") ";

      // error
      str =
          "CREATE TABLE error"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // perror_x
      str += s_perror_x + " " +
             dbinfo::type_names[std::type_index(typeid(perror_x))] + ",";
      // perror_y
      str += s_perror_y + " " +
             dbinfo::type_names[std::type_index(typeid(perror_y))] + ",";
      // perror_mz
      str += s_perror_mz + " " +
             dbinfo::type_names[std::type_index(typeid(perror_mz))] + ",";
      // verror_x
      str += s_verror_x + " " +
             dbinfo::type_names[std::type_index(typeid(verror_x))] + ",";
      // verror_y
      str += s_verror_y + " " +
             dbinfo::type_names[std::type_index(typeid(verror_y))] + ",";
      // verror_mz
      str += s_verror_mz + " " +
             dbinfo::type_names[std::type_index(typeid(verror_mz))] + ");";

      db << str;

      // insert_string_error
      insert_string_error = "(DATETIME, ";
      insert_string_error += s_perror_x + ", ";
      insert_string_error += s_perror_y + ", ";
      insert_string_error += s_perror_mz + ", ";
      insert_string_error += s_verror_x + ", ";
      insert_string_error += s_verror_y + ", ";
      insert_string_error += s_verror_mz + ") ";

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
  std::string insert_string_measurement;
  std::string insert_string_state;
  std::string insert_string_error;

  sqlite::database db;

};  // end class estimator_db

class planner_db : public dbinfo {
 public:
  explicit planner_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "planner.db"),
        insert_string_routeplanner(""),
        insert_string_latticeplanner(""),
        db(dbpath) {}
  ~planner_db() {}

  void create_table(double speed = 0, double captureradius = 0, double WPX = 0,
                    double WPY = 0, double WPLONG = 0, double WPLAT = 0,
                    double lattice_x = 0, double lattice_y = 0,
                    double lattice_theta = 0, double lattice_kappa = 0,
                    double lattice_speed = 0, double lattice_dspeed = 0) {
    try {
      std::string s_speed = VARIABLENAME4DB(speed);
      std::string s_captureradius = VARIABLENAME4DB(captureradius);
      std::string s_WPX = VARIABLENAME4DB(WPX);
      std::string s_WPY = VARIABLENAME4DB(WPY);
      std::string s_WPLONG = VARIABLENAME4DB(WPLONG);
      std::string s_WPLAT = VARIABLENAME4DB(WPLAT);

      std::string str =
          "CREATE TABLE routeplanner"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // speed
      str += s_speed + " " +
             dbinfo::type_names[std::type_index(typeid(speed))] + ",";
      // captureradius;
      str += s_captureradius + " " +
             dbinfo::type_names[std::type_index(typeid(captureradius))] + ",";
      // WPX;
      str +=
          s_WPX + " " + dbinfo::type_names[std::type_index(typeid(WPX))] + ",";
      // WPY
      str +=
          s_WPY + " " + dbinfo::type_names[std::type_index(typeid(WPY))] + ",";
      // WPLONG
      str += s_WPLONG + " " +
             dbinfo::type_names[std::type_index(typeid(WPLONG))] + ",";
      // WPLAT
      str += s_WPLAT + " " + dbinfo::type_names[std::type_index(typeid(WPLAT))];
      str += ");";

      db << str;
      // insert_string_routeplanner
      insert_string_routeplanner = "(DATETIME, ";
      insert_string_routeplanner += s_speed + ", ";
      insert_string_routeplanner += s_captureradius + ", ";
      insert_string_routeplanner += s_WPX + ", ";
      insert_string_routeplanner += s_WPY + ", ";
      insert_string_routeplanner += s_WPLONG + ", ";
      insert_string_routeplanner += s_WPLAT + ")";

      // planning marine state
      std::string s_lattice_x = VARIABLENAME4DB(lattice_x);
      std::string s_lattice_y = VARIABLENAME4DB(lattice_y);
      std::string s_lattice_theta = VARIABLENAME4DB(lattice_theta);
      std::string s_lattice_kappa = VARIABLENAME4DB(lattice_kappa);
      std::string s_lattice_speed = VARIABLENAME4DB(lattice_speed);
      std::string s_lattice_dspeed = VARIABLENAME4DB(lattice_dspeed);

      str =
          "CREATE TABLE latticeplanner"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // lattice_x
      str += s_lattice_x + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_x))] + ",";
      // lattice_y;
      str += s_lattice_y + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_y))] + ",";
      // lattice_theta
      str += s_lattice_theta + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_theta))] + ",";
      // lattice_kappa
      str += s_lattice_kappa + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_kappa))] + ",";
      // lattice_speed
      str += s_lattice_speed + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_speed))] + ",";
      // lattice_dspeed
      str += s_lattice_dspeed + " " +
             dbinfo::type_names[std::type_index(typeid(lattice_dspeed))];
      str += ");";

      db << str;
      // insert_string_latticeplanner
      insert_string_latticeplanner = "(DATETIME, ";
      insert_string_latticeplanner += s_lattice_x + ", ";
      insert_string_latticeplanner += s_lattice_y + ", ";
      insert_string_latticeplanner += s_lattice_theta + ", ";
      insert_string_latticeplanner += s_lattice_kappa + ", ";
      insert_string_latticeplanner += s_lattice_speed + ", ";
      insert_string_latticeplanner += s_lattice_dspeed + ")";

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
  std::string insert_string_routeplanner;
  std::string insert_string_latticeplanner;

  sqlite::database db;

};  // end class planner_db

class controller_db : public dbinfo {
 public:
  controller_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "controller.db"),
        insert_string_setpoint(""),
        insert_string_TA(""),
        db(dbpath) {}
  ~controller_db() {}

  void create_table(double set_x = 0, double set_y = 0, double set_theta = 0,
                    double set_u = 0, double set_v = 0, double set_r = 0,
                    double desired_Fx = 0, double desired_Fy = 0,
                    double desired_Mz = 0, double est_Fx = 0, double est_Fy = 0,
                    double est_Mz = 0) {
    try {
      // setpoints
      std::string s_set_x = VARIABLENAME4DB(set_x);
      std::string s_set_y = VARIABLENAME4DB(set_y);
      std::string s_set_theta = VARIABLENAME4DB(set_theta);
      std::string s_set_u = VARIABLENAME4DB(set_u);
      std::string s_set_v = VARIABLENAME4DB(set_v);
      std::string s_set_r = VARIABLENAME4DB(set_r);

      std::string str =
          "CREATE TABLE setpoint"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // set_x
      str += s_set_x + " " +
             dbinfo::type_names[std::type_index(typeid(set_x))] + ",";
      // set_y
      str += s_set_y + " " +
             dbinfo::type_names[std::type_index(typeid(set_y))] + ",";
      // set_theta
      str += s_set_theta + " " +
             dbinfo::type_names[std::type_index(typeid(set_theta))] + ",";
      // set_u
      str += s_set_u + " " +
             dbinfo::type_names[std::type_index(typeid(set_u))] + ",";
      // set_v
      str += s_set_v + " " +
             dbinfo::type_names[std::type_index(typeid(set_v))] + ",";
      // set_r
      str += s_set_r + " " + dbinfo::type_names[std::type_index(typeid(set_r))];
      str += ");";
      db << str;

      insert_string_setpoint = "(DATETIME, ";
      insert_string_setpoint += s_set_x;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_set_y;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_set_theta;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_set_u;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_set_v;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_set_r;
      insert_string_setpoint += ") ";

      // thrust allocation
      std::string s_desired_Fx = VARIABLENAME4DB(desired_Fx);
      std::string s_desired_Fy = VARIABLENAME4DB(desired_Fy);
      std::string s_desired_Mz = VARIABLENAME4DB(desired_Mz);
      std::string s_est_Fx = VARIABLENAME4DB(est_Fx);
      std::string s_est_Fy = VARIABLENAME4DB(est_Fy);
      std::string s_est_Mz = VARIABLENAME4DB(est_Mz);

      str =
          "CREATE TABLE TA"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // desired_Fx
      str += s_desired_Fx + " " +
             dbinfo::type_names[std::type_index(typeid(desired_Fx))] + ",";
      // desired_Fy
      str += s_desired_Fy + " " +
             dbinfo::type_names[std::type_index(typeid(desired_Fy))] + ",";
      // desired_Mz
      str += s_desired_Mz + " " +
             dbinfo::type_names[std::type_index(typeid(desired_Mz))] + ",";
      // est_Fx
      str += s_est_Fx + " " +
             dbinfo::type_names[std::type_index(typeid(est_Fx))] + ",";
      // est_Fy
      str += s_est_Fy + " " +
             dbinfo::type_names[std::type_index(typeid(est_Fy))] + ",";
      // est_Mz
      str += s_est_Mz + " " +
             dbinfo::type_names[std::type_index(typeid(est_Mz))] + ",";
      // Azimuth
      str += "Azimuth BLOB,";
      // Rotation
      str += "Rotation BLOB);";

      db << str;

      insert_string_TA = "(DATETIME, ";
      insert_string_TA += s_desired_Fx;
      insert_string_TA += ", ";
      insert_string_TA += s_desired_Fy;
      insert_string_TA += ", ";
      insert_string_TA += s_desired_Mz;
      insert_string_TA += ", ";
      insert_string_TA += s_est_Fx;
      insert_string_TA += ", ";
      insert_string_TA += s_est_Fy;
      insert_string_TA += ", ";
      insert_string_TA += s_est_Mz;
      insert_string_TA += ", Azimuth, Rotation)";

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
  std::string insert_string_setpoint;
  std::string insert_string_TA;

  sqlite::database db;

};  // end class controller_db

class perception_db : public dbinfo {
 public:
  perception_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "perception.db"),
        insert_string_spoke(""),
        insert_string_detectedtarget(""),
        insert_string_trackingtarget(""),
        db(dbpath) {}
  ~perception_db() {}

  void create_table(const std::vector<double> &surroundings_bearing_rad = {},
                    const std::vector<double> &surroundings_range_m = {},
                    const std::vector<double> &surroundings_x_m = {},
                    const std::vector<double> &surroundings_y_m = {},
                    const std::vector<double> &detected_target_x = {},
                    const std::vector<double> &detected_target_y = {},
                    const std::vector<double> &detected_target_radius = {},
                    int spoke_state = 0,
                    const std::vector<int> &targets_state = {},
                    const std::vector<int> &targets_intention = {},
                    const std::vector<double> &targets_x = {},
                    const std::vector<double> &targets_y = {},
                    const std::vector<double> &targets_square_radius = {},
                    const std::vector<double> &targets_vx = {},
                    const std::vector<double> &targets_vy = {},
                    const std::vector<double> &targets_CPA_x = {},
                    const std::vector<double> &targets_CPA_y = {},
                    const std::vector<double> &targets_TCPA = {}) {
    try {
      // spoke process
      std::string s_surroundings_bearing_rad =
          VARIABLENAME4DB(surroundings_bearing_rad);
      std::string s_surroundings_range_m =
          VARIABLENAME4DB(surroundings_range_m);
      std::string s_surroundings_x_m = VARIABLENAME4DB(surroundings_x_m);
      std::string s_surroundings_y_m = VARIABLENAME4DB(surroundings_y_m);

      std::string str =
          "CREATE TABLE SpokeProcess"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // surroundings_bearing_rad
      str += s_surroundings_bearing_rad + " " +
             dbinfo::type_names[std::type_index(
                 typeid(surroundings_bearing_rad))] +
             ",";
      // surroundings_range_m
      str += s_surroundings_range_m + " " +
             dbinfo::type_names[std::type_index(typeid(surroundings_range_m))] +
             ",";
      // surroundings_x_m
      str += s_surroundings_x_m + " " +
             dbinfo::type_names[std::type_index(typeid(surroundings_x_m))] +
             ",";
      // surroundings_y_m
      str += s_surroundings_y_m + " " +
             dbinfo::type_names[std::type_index(typeid(surroundings_y_m))];
      str += ");";
      db << str;

      insert_string_spoke = "(DATETIME, ";
      insert_string_spoke += s_surroundings_bearing_rad;
      insert_string_spoke += ", ";
      insert_string_spoke += s_surroundings_range_m;
      insert_string_spoke += ", ";
      insert_string_spoke += s_surroundings_x_m;
      insert_string_spoke += ", ";
      insert_string_spoke += s_surroundings_y_m;
      insert_string_spoke += ") ";

      // Detected target
      std::string s_detected_target_x = VARIABLENAME4DB(detected_target_x);
      std::string s_detected_target_y = VARIABLENAME4DB(detected_target_y);
      std::string s_detected_target_radius =
          VARIABLENAME4DB(detected_target_radius);

      str =
          "CREATE TABLE DetectedTarget"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // detected_target_x
      str += s_detected_target_x + " " +
             dbinfo::type_names[std::type_index(typeid(detected_target_x))] +
             ",";
      // detected_target_y
      str += s_detected_target_y + " " +
             dbinfo::type_names[std::type_index(typeid(detected_target_y))] +
             ",";
      // detected_target_radius
      str +=
          s_detected_target_radius + " " +
          dbinfo::type_names[std::type_index(typeid(detected_target_radius))] +
          ");";
      db << str;

      insert_string_detectedtarget = "(DATETIME, ";
      insert_string_detectedtarget += s_detected_target_x;
      insert_string_detectedtarget += ", ";
      insert_string_detectedtarget += s_detected_target_y;
      insert_string_detectedtarget += ", ";
      insert_string_detectedtarget += s_detected_target_radius;
      insert_string_detectedtarget += ")";

      // Tracking targets
      std::string s_spoke_state = VARIABLENAME4DB(spoke_state);
      std::string s_targets_state = VARIABLENAME4DB(targets_state);
      std::string s_targets_intention = VARIABLENAME4DB(targets_intention);
      std::string s_targets_x = VARIABLENAME4DB(targets_x);
      std::string s_targets_y = VARIABLENAME4DB(targets_y);
      std::string s_targets_square_radius =
          VARIABLENAME4DB(targets_square_radius);
      std::string s_targets_vx = VARIABLENAME4DB(targets_vx);
      std::string s_targets_vy = VARIABLENAME4DB(targets_vy);
      std::string s_targets_CPA_x = VARIABLENAME4DB(targets_CPA_x);
      std::string s_targets_CPA_y = VARIABLENAME4DB(targets_CPA_y);
      std::string s_targets_TCPA = VARIABLENAME4DB(targets_TCPA);

      str =
          "CREATE TABLE TrackingTarget"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,";
      // spoke_state
      str += s_spoke_state + " " +
             dbinfo::type_names[std::type_index(typeid(spoke_state))] + ",";
      // targets_state
      str += s_targets_state + " " +
             dbinfo::type_names[std::type_index(typeid(targets_state))] + ",";
      // targets_intention
      str += s_targets_intention + " " +
             dbinfo::type_names[std::type_index(typeid(targets_intention))] +
             ",";
      // targets_x
      str += s_targets_x + " " +
             dbinfo::type_names[std::type_index(typeid(targets_x))] + ",";
      // targets_y
      str += s_targets_y + " " +
             dbinfo::type_names[std::type_index(typeid(targets_y))] + ",";
      // targets_square_radius
      str +=
          s_targets_square_radius + " " +
          dbinfo::type_names[std::type_index(typeid(targets_square_radius))] +
          ",";
      // targets_vx
      str += s_targets_vx + " " +
             dbinfo::type_names[std::type_index(typeid(targets_vx))] + ",";
      // targets_vy
      str += s_targets_vy + " " +
             dbinfo::type_names[std::type_index(typeid(targets_vy))] + ",";
      // targets_CPA_x
      str += s_targets_CPA_x + " " +
             dbinfo::type_names[std::type_index(typeid(targets_CPA_x))] + ",";
      // targets_CPA_y
      str += s_targets_CPA_y + " " +
             dbinfo::type_names[std::type_index(typeid(targets_CPA_y))] + ",";
      // targets_TCPA
      str += s_targets_TCPA + " " +
             dbinfo::type_names[std::type_index(typeid(targets_TCPA))] + ");";

      db << str;

      insert_string_trackingtarget = "(DATETIME, ";
      insert_string_trackingtarget += s_spoke_state;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_state;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_intention;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_x;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_y;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_square_radius;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_vx;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_vy;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_CPA_x;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_CPA_y;
      insert_string_trackingtarget += ", ";
      insert_string_trackingtarget += s_targets_TCPA;
      insert_string_trackingtarget += ")";

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
  std::string insert_string_spoke;
  std::string insert_string_detectedtarget;
  std::string insert_string_trackingtarget;

  sqlite::database db;

};  // end class perception_db

}  // namespace ASV::common

#endif /* _DATARECORDER_H_ */