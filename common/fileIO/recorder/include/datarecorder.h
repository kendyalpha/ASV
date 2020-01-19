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
                    {std::type_index(typeid(std::string)), "TEXT"}}) {}
  virtual ~dbinfo() = default;

 protected:
  std::string folder_path;
  std::unordered_map<std::type_index, std::string> type_names;
};  // end class database

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

      std::cout << str << std::endl;

      // insert_string
      insert_string_measurement = "(DATETIME, ";
      insert_string_measurement += s_meas_x + ", ";
      insert_string_measurement += s_meas_y + ", ";
      insert_string_measurement += s_meas_theta + ", ";
      insert_string_measurement += s_meas_u + ", ";
      insert_string_measurement += s_meas_v + ", ";
      insert_string_measurement += s_meas_r + ") ";

      std::cout << insert_string_measurement << std::endl;

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

      std::cout << str << std::endl;

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

      std::cout << insert_string_state << std::endl;

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

      std::cout << str << std::endl;

      // insert_string_error
      insert_string_error = "(DATETIME, ";
      insert_string_error += s_perror_x + ", ";
      insert_string_error += s_perror_y + ", ";
      insert_string_error += s_perror_mz + ", ";
      insert_string_error += s_verror_x + ", ";
      insert_string_error += s_verror_y + ", ";
      insert_string_error += s_verror_mz + ") ";

      std::cout << insert_string_error << std::endl;

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
        db(dbpath) {}
  ~planner_db() {}

  void create_table(double speed = 0, double captureradius = 0, double WPX = 0,
                    double WPY = 0, double WPLONG = 0, double WPLAT = 0) {
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

 private:
  std::string dbpath;
  std::string insert_string_routeplanner;
  sqlite::database db;

};  // end class planner_db

class controller_db : public dbinfo {
 public:
  controller_db(const std::string &_folder_path)
      : dbinfo(_folder_path),
        dbpath(dbinfo::folder_path + "controller.db"),
        insert_string_setpoint(""),
        db(dbpath) {
    std::cout << dbpath << std::endl;
  }
  ~controller_db() {}

  void create_table(double set_x = 0, double set_y = 0, double set_theta = 0,
                    double set_u = 0, double set_v = 0, double set_r = 0,
                    double desired_Fx = 0, double desired_Fy = 0,
                    double desired_Mz = 0, double est_Fx = 0, double est_Fy = 0,
                    double est_Mz = 0) {
    try {
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
      // set_x
      str += s_set_x + " " +
             dbinfo::type_names[std::type_index(typeid(set_x))] + ",";
      // set_x
      str += s_set_x + " " +
             dbinfo::type_names[std::type_index(typeid(set_x))] + ",";
      // set_x
      str += s_set_x + " " +
             dbinfo::type_names[std::type_index(typeid(set_x))] + ",";
      str += ");";
      db << str;

      insert_string_setpoint = "(DATETIME, ";
      insert_string_setpoint += s_intvalue;
      insert_string_setpoint += ", ";
      insert_string_setpoint += s_doublevalue;
      insert_string_setpoint += ") ";

      std::cout << str << std::endl << insert_string << std::endl;

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_table
  void update_table(int intvalue, double doublevalue) {
    try {
      std::string str = "INSERT INTO controller";
      str += insert_string;
      str += "VALUES(julianday('now')";
      str += ", ";
      str += std::to_string(intvalue);
      str += ", ";
      str += std::to_string(doublevalue);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-GPS") << e.what();
    }
  }

 private:
  std::string dbpath;
  std::string insert_string_setpoint;
  sqlite::database db;

};  // end class controller_db

}  // namespace ASV::common

#endif /* _DATARECORDER_H_ */