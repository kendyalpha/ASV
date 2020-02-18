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
        dbpath(_DB_folder_path + "gps.db"),
        config_name(_config_name),
        db(dbpath) {}

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
    for (auto const &[name, type] : db_config) parse_string += ", " + name;
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
  std::string dbpath;
  std::string config_name;

  sqlite::database db;

};  // end class GPS_parser

}  // namespace ASV::common

#endif /* _DATAPARSER_H_ */