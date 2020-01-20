/*
***********************************************************************
* database.h:
* database using sqlite3 and sqlite modern cpp wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DATABASE_H_
#define _DATABASE_H_

#include <sqlite_modern_cpp.h>
#include <common/math/eigen/Eigen/Core>
#include <sstream>
#include <string>
#include <vector>
#include "common/fileIO/include/utilityio.h"
#include "common/logging/include/easylogging++.h"
#include "common/property/include/priority.h"
#include "modules/controller/include/controllerdata.h"
#include "modules/estimator/include/estimatordata.h"
#include "modules/messages/sensors/gpsimu/include/gpsdata.h"
#include "modules/messages/sensors/wind/include/winddata.h"
#include "modules/messages/stm32/include/stm32data.h"
#include "modules/perception/marine_radar/include/TargetTrackingData.h"
#include "modules/planner/route_planning/include/RoutePlannerData.h"

namespace ASV::common {
template <int m, int n = 3>
class database {
 public:
  explicit database(const std::string &_savepath)
      : db(_savepath),
        clientset({"controller", "estimator", "route_planning", "GPS", "wind",
                   "motor", "stm32", "surroundings"}) {}

  ~database() = default;

  void initializetables() {
    create_mastertable();
    create_GPS_table();           //
    create_wind_table();          //
    create_controller_table();    //
    create_estimator_table();     //
    create_routeplanner_table();  //
    create_stm32_table();
    create_surrounding_table();
  }  // initializetables

  // insert a bow into gps table
  void update_gps_table(const messages::gpsRTdata &_gpsRTdata) {
    try {
      std::string str =
          "INSERT INTO GPS"
          "(DATETIME, UTC, latitude, longitude, heading, pitch, roll, "
          " altitude, Ve, Vn, roti, status,UTM_x, UTM_y, UTM_zone) "
          " VALUES(julianday('now')";
      convert2string(_gpsRTdata, str);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-GPS") << e.what();
    }
  }  // update_gps_table

  // insert a bow into gps table
  void update_surroundings_table(
      const perception::SpokeProcessRTdata &_spokeRTdata) {
    try {
      db << "INSERT INTO surroundings"
            "(bearing_rad, range_m, x_m, y_m) VALUES (?,?,?,?)"
         << _spokeRTdata.surroundings_bearing_rad
         << _spokeRTdata.surroundings_range_m << _spokeRTdata.surroundings_x_m
         << _spokeRTdata.surroundings_y_m;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-surroundings") << e.what();
    }
  }  // update_surroundings_table

  // insert a bow into controller table
  void update_controller_table(const control::controllerRTdata<m, n> &_RTdata,
                               const control::trackerRTdata &_trackRTdata) {
    try {
      std::string str =
          "INSERT INTO controller"
          "(DATETIME, set_x, set_y, set_theta, set_u, set_v, set_r";
      // tau: desired force
      for (int i = 0; i != n; ++i) {
        str += ", tau" + std::to_string(i + 1);
      }
      // the angle of each propeller
      for (int i = 0; i != m; ++i) {
        str += ", alpha" + std::to_string(i + 1);
      }
      // the speed of each propeller
      for (int i = 0; i != m; ++i) {
        str += ", rpm" + std::to_string(i + 1);
      }
      // BalphaU: estimated force
      for (int i = 0; i != n; ++i) {
        str += ", est" + std::to_string(i + 1);
      }

      str += ") VALUES(julianday('now')";
      convert2string(_RTdata, _trackRTdata, str);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }  // update_controller_table

  // insert a bow into estimator table
  void update_estimator_table(const estimatorRTdata &_RTdata) {
    try {
      std::string str =
          "INSERT INTO estimator"
          "(DATETIME, meas_x, meas_y, meas_theta, meas_u, meas_v, meas_r,"
          "state_x, state_y, state_theta, state_u, state_v, state_r, "
          "perror_x, perror_y, perror_mz, verror_x, verror_y, verror_mz, "
          "curvature, speed, dspeed ) VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";
      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }  // update_estimator_table

  // insert a bow into route planner table
  void update_routeplanner_table(const planning::RoutePlannerRTdata &_RTdata) {
    for (int i = 0; i != _RTdata.Waypoint_X.size(); ++i) {
      try {
        std::string str =
            "INSERT INTO route_planning"
            "(DATETIME, speed, captureradius, WPX, WPY, "
            "WPLONG, WPLAT) VALUES(julianday('now')";

        str += ", ";
        str += std::to_string(_RTdata.speed);
        str += ", ";
        str += std::to_string(_RTdata.los_capture_radius);
        str += ", ";
        str += std::to_string(_RTdata.Waypoint_X(i));
        str += ", ";
        str += std::to_string(_RTdata.Waypoint_Y(i));
        str += ", ";
        str += std::to_string(_RTdata.Waypoint_longitude(i));
        str += ", ";
        str += std::to_string(_RTdata.Waypoint_latitude(i));
        str += ");";
        db << str;
      } catch (sqlite::sqlite_exception &e) {
        CLOG(ERROR, "sql-routeplanner") << e.what();
      }
    }

  }  // update_routeplanner_table

  void update_stm32_table(const messages::stm32data &_RTdata) {
    try {
      std::string str =
          "INSERT INTO stm32"
          "(DATETIME, stm32_link, stm32_status, command_u1, command_u2, "
          "feedback_u1, feedback_u2, feedback_pwm1, feedback_pwm2, "
          "RC_X, RC_Y, RC_Mz, voltage_b1, voltage_b2, voltage_b3 ) "
          "VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-stm32") << e.what();
    }
  }  // update_stm32_table

  // insert a row into wind table
  void update_wind_table(const windRTdata &_RTdata) {
    try {
      std::string str =
          "INSERT INTO wind"
          "(DATETIME, speed, orientation) "
          "VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";
      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-wind") << e.what();
    }
  }  // update_wind_table

  // // read sql data from database
  // void readdatafromdb(gpsRTdata &_gpsRTdata, controllerRTdata<m, n> &_RTdata,
  //                     estimatorRTdata &_RTdata, plannerRTdata &_RTdata,
  //                     indicators &_RTdata) {
  //   try {
  //     std::string str =
  //         "INSERT INTO indicators"
  //         "(DATETIME, gui_link, joystick_link, controlmode, windstatus) "
  //         "VALUES(julianday('now')";
  //     convert2string(_RTdata, str);
  //     str += ");";
  //     db << str;
  //   } catch (sqlite::sqlite_exception &e) {
  //     CLOG(ERROR, "sql-read") << e.what();
  //   }
  // }

 private:
  sqlite::database db;
  // the set of table names
  // 0 --> controller
  // 1 --> estimator
  // 2 --> planner
  // 3 --> GPS
  std::vector<std::string> clientset;

  // create master table about the info of each tables
  void create_mastertable() {
    try {
      db << "CREATE TABLE IF NOT EXISTS MASTER("
            "CLIENT_ID    INT    PRIMARY KEY    NOT NULL,"
            "TABLE_NAME   TEXT                          ,"
            "DATETIME     TEXT                         );";

      for (unsigned int i = 0; i != clientset.size(); ++i) {
        // update clientset
        std::string s_id = std::to_string(i);
        // create string for sqlite
        std::string str(
            "INSERT INTO MASTER (CLIENT_ID,TABLE_NAME,DATETIME) VALUES(");
        str += s_id;
        str += ", '";
        str += clientset[i];
        str += "' , DATETIME('now'));";
        db << str;
      }
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_mastertable

  // create surrounding table
  void create_surrounding_table() {
    try {
      std::string str =
          "CREATE TABLE surroundings"
          "(ID           INTEGER PRIMARY KEY AUTOINCREMENT,"
          " bearing_rad  BLOB, "
          " range_m      BLOB, "
          " x_m          BLOB, "
          " y_m          BLOB); ";

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_surrounding_table

  // create GPS table
  void create_GPS_table() {
    try {
      // GPS/IMU sensor GPFPD format
      std::string str =
          "CREATE TABLE GPS"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,"
          " UTC         DOUBLE, "
          " latitude    DOUBLE, "
          " longitude   DOUBLE, "
          " heading     DOUBLE, "
          " pitch       DOUBLE, "
          " roll        DOUBLE, "
          " altitude    DOUBLE, "
          " Ve          DOUBLE, "
          " Vn          DOUBLE, "
          " roti        DOUBLE, "
          " status      INT,    "
          " UTM_x       DOUBLE, "
          " UTM_y       DOUBLE, "
          " UTM_zone    TEXT);";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_GPS_table

  // create each controller table (TODO: controller for underactuated USV)
  void create_controller_table() {
    try {
      // real-time data in the controller
      std::string str =
          "CREATE TABLE controller ("
          "  ID            INTEGER PRIMARY KEY AUTOINCREMENT"
          ", DATETIME      TEXT       NOT NULL "
          ", set_x         DOUBLE "
          ", set_y         DOUBLE "
          ", set_theta     DOUBLE " /* setpoint */
          ", set_u         DOUBLE "
          ", set_v         DOUBLE "
          ", set_r         DOUBLE "; /* v_setpoint */
      // tau: desired force
      for (int i = 0; i != n; ++i) {
        str += " ,tau" + std::to_string(i + 1) + " DOUBLE";
      }
      // the angle of each propeller
      for (int i = 0; i != m; ++i) {
        str += " ,alpha" + std::to_string(i + 1) + " INT";
      }
      // the speed of each propeller
      for (int i = 0; i != m; ++i) {
        str += " ,rpm" + std::to_string(i + 1) + " INT";
      }
      // BalphaU: estimated force
      for (int i = 0; i != n; ++i) {
        str += " ,est" + std::to_string(i + 1) + " DOUBLE";
      }
      str += ");";

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_controller_table

  // create each estimator table
  void create_estimator_table() {
    try {
      // real-time data in the estimator
      std::string str =
          "CREATE TABLE estimator"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,"
          " meas_x      DOUBLE, "
          " meas_y      DOUBLE, "
          " meas_theta  DOUBLE, "
          " meas_u      DOUBLE, "
          " meas_v      DOUBLE, "
          " meas_r      DOUBLE, " /* Measurement */
          " state_x     DOUBLE, "
          " state_y     DOUBLE, "
          " state_theta DOUBLE, "
          " state_u     DOUBLE, "
          " state_v     DOUBLE, "
          " state_r     DOUBLE, " /* state */
          " perror_x    DOUBLE, "
          " perror_y    DOUBLE, "
          " perror_mz   DOUBLE, " /* perror */
          " verror_x    DOUBLE, "
          " verror_y    DOUBLE, "
          " verror_mz   DOUBLE, " /* verror */
          " curvature   DOUBLE, "
          " speed       DOUBLE, "
          " dspeed      DOUBLE); "; /* Marine state*/
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_estimator_table

  // create_routeplanner_table
  void create_routeplanner_table() {
    try {
      // real-time data in the route planner
      std::string str =
          "CREATE TABLE route_planning"
          "(ID            INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME      TEXT       NOT NULL,"
          " speed         DOUBLE, "
          " captureradius DOUBLE, "
          " WPX           DOUBLE, "
          " WPY           DOUBLE, "
          " WPLONG        DOUBLE, "
          " WPLAT         DOUBLE);";

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_routeplanner_table

  // create wind table
  void create_wind_table() {
    try {
      std::string str =
          "CREATE TABLE wind"
          "(ID               INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME         TEXT       NOT NULL,"
          " speed            DOUBLE, " /* setpoint */
          " orientation      DOUBLE); ";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_wind_table

  // create stm32 table
  void create_stm32_table() {
    try {
      std::string str =
          "CREATE TABLE stm32"
          "(ID               INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME         TEXT       NOT NULL,"
          " stm32_link       INT, "
          " stm32_status     INT, "
          " command_u1       DOUBLE, " /* motor */
          " command_u2       DOUBLE, "
          " feedback_u1      DOUBLE, "
          " feedback_u2      DOUBLE, "
          " feedback_pwm1    INT, "
          " feedback_pwm2    INT, "
          " RC_X             DOUBLE, " /* remote control */
          " RC_Y             DOUBLE, "
          " RC_Mz            DOUBLE, "
          " voltage_b1       DOUBLE, " /* battery */
          " voltage_b2       DOUBLE, "
          " voltage_b3       DOUBLE); ";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }  // create_stm32_table

  // convert real time GPS data to sql string
  void convert2string(const messages::gpsRTdata &_gpsRTdata,
                      std::string &_str) {
    _str += ", ";
    _str += to_string_with_precision<double>(_gpsRTdata.UTC, 2);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.latitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.longitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.heading);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.pitch);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.roll);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.altitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Ve);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Vn);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.roti);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.status);
    _str += ", ";
    _str += to_string_with_precision<double>(_gpsRTdata.UTM_x, 3);
    _str += ", ";
    _str += to_string_with_precision<double>(_gpsRTdata.UTM_y, 3);
    _str += ", '";
    _str += _gpsRTdata.UTM_zone;
    _str += "'";
  }

  void convert2string(const control::controllerRTdata<m, n> &_RTdata,
                      const control::trackerRTdata &_trackRTdata,
                      std::string &_str) {
    // setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += to_string_with_precision<double>(_trackRTdata.setpoint(i), 3);
    }
    // v_setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_trackRTdata.v_setpoint(i));
    }
    // tau: desired force
    for (int i = 0; i != n; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.tau(i));
    }
    // the angle of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.command_alpha_deg(i));
    }
    // the speed of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.command_rotation(i));
    }
    // BalphaU: estimated force
    for (int i = 0; i != n; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.BalphaU(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // Measurement
    for (int i = 0; i != 5; ++i) {
      _str += ", ";
      _str += to_string_with_precision<double>(_RTdata.Measurement(i), 3);
    }
    // r is special
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.Measurement(5), 4);

    // State
    for (int i = 0; i != 5; ++i) {
      _str += ", ";
      _str += to_string_with_precision<double>(_RTdata.State(i), 3);
    }
    // r is special
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.State(5), 4);

    // Perror
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.p_error(i));
    }
    // verror
    for (int i = 0; i != 2; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.v_error(i));
    }
    // r is special
    _str += ", ";
    _str += std::to_string(_RTdata.v_error(2));

    // marine state
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += to_string_with_precision<double>(_RTdata.Marine_state(i + 3), 2);
    }
  }

  void convert2string(const messages::stm32data &_RTdata, std::string &_str) {
    _str += ",";
    _str += std::to_string(static_cast<int>(_RTdata.linkstatus));
    _str += ",";
    _str += std::to_string(static_cast<int>(_RTdata.feedback_stm32status));

    // motor
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.command_u1, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.command_u2, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.feedback_u1, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.feedback_u2, 1);
    _str += ", ";
    _str += std::to_string(_RTdata.feedback_pwm1);
    _str += ", ";
    _str += std::to_string(_RTdata.feedback_pwm2);
    // remote control
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.RC_X, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.RC_Y, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.RC_Mz, 1);

    // battery
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.voltage_b1, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.voltage_b2, 1);
    _str += ", ";
    _str += to_string_with_precision<double>(_RTdata.voltage_b3, 1);
  }

  void convert2string(const windRTdata &_RTdata, std::string &_str) {
    _str += ", ";
    _str += std::to_string(_RTdata.speed);
    _str += ", ";
    _str += std::to_string(_RTdata.orientation);
  }

};  // end class database
}  // namespace ASV::common

#endif /* _DATABASE_H_ */