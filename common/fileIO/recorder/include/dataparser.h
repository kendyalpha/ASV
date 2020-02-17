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

  void parse_table(const double start_time, const double end_time,
                   std::vector<double> &local_time,    //
                   std::vector<double> &UTC,           //
                   std::vector<double> &latitude,      //
                   std::vector<double> &longitude,     //
                   std::vector<double> &heading,       //
                   std::vector<double> &pitch,         //
                   std::vector<double> &roll,          //
                   std::vector<double> &altitude,      //
                   std::vector<double> &Ve,            //
                   std::vector<double> &Vn,            //
                   std::vector<double> &roti,          //
                   std::vector<int> &status,           //
                   std::vector<double> &UTM_x,         //
                   std::vector<double> &UTM_y,         //
                   std::vector<std::string> &UTM_zone  //
  ) {
    int max_id = 0;
    // master_parser::timestamp0;
    db << "select MAX(ID) from GPS;" >> max_id;

    for (int i = 0; i != max_id; i++) {
      db << "select age,name,weight from user where ID = ? ;" << i + 1 >>
          [&](std::string _local_time, string name, double weight) {
            cout << age << ' ' << name << ' ' << weight << endl;
          };
    }
  }

 private:
  std::string dbpath;
  std::string config_name;

  sqlite::database db;
}
};  // namespace ASV::common

}  // namespace ASV::common

#endif /* _DATAPARSER_H_ */