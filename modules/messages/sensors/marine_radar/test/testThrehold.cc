/*
****************************************************************************
* testThrehold.cc:
* example for marine radar and write spoke data to sqlite3
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <sqlite_modern_cpp.h>
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"
#include "modules/perception/marine_radar/include/SpokeProcessing.h"

using namespace ASV;
using namespace sqlite;

void readsqlitedata() {
  database db("radar.db");
  std::vector<uint8_t> numbers_test;
  db << "SELECT numbers from person where id = ?;" << 1 >> numbers_test;
}

int main() {
  // radar
  messages::MarineRadar _MarineRadar;
  _MarineRadar.StartMarineRadar();
  // real time data from marine radar
  messages::MarineRadarRTdata MarineRadar_RTdata{
      common::STATETOGGLE::IDLE,  // state_toggle
      0,                          // spoke_azimuth_deg
      0,                          // spoke_samplerange_m
      {0x00, 0x00, 0x00}          // spokedata
  };

  // Spoke Processing
  perception::SpokeProcessdata SpokeProcess_data{
      0.1,   // sample_time
      -1.0,  // radar_x
      0.0    // radar_y
  };

  perception::AlarmZone Alarm_Zone{
      10,        // start_range_m
      20,        // end_range_m
      0,         // center_bearing_rad
      M_PI / 2,  // width_bearing_rad
      0xac       // sensitivity_threhold
  };
  perception::SpokeProcessing _SpokeProcessing(Alarm_Zone, SpokeProcess_data);

  perception::SpokeProcessRTdata SpokeProcess_RTdata;

  // sqlite3
  database db("radar.db");
  db << "CREATE TABLE person (id integer primary key autoincrement not "
        "null, bearing BLOB, range BLOB, x BLOB, y BLOB);";

  while (1) {
    MarineRadar_RTdata = _MarineRadar.getMarineRadarRTdata();

    SpokeProcess_RTdata =
        _SpokeProcessing
            .DetectionOnSpoke(MarineRadar_RTdata.spokedata,
                              SAMPLES_PER_SPOKE / 2,
                              MarineRadar_RTdata.spoke_azimuth_deg,
                              MarineRadar_RTdata.spoke_samplerange_m)
            .getSpokeProcessRTdata();

    db << "INSERT INTO person (bearing, range, x, y) VALUES (?, ?, ?, ?)"
       << SpokeProcess_RTdata.surroundings_bearing_rad
       << SpokeProcess_RTdata.surroundings_range_m
       << SpokeProcess_RTdata.surroundings_x_m
       << SpokeProcess_RTdata.surroundings_y_m;

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}