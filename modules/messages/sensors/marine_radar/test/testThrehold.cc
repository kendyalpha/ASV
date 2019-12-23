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
#include <memory>
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"
#include "modules/perception/marine_radar/include/SpokeProcessing.h"

using namespace ASV;
using namespace sqlite;

void startRadarAndSpoke() {
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
      0.1,  // sample_time
      0.0,  // radar_x
      0.0   // radar_y
  };

  perception::AlarmZone Alarm_Zone{
      10,        // start_range_m
      20,        // end_range_m
      0,         // center_bearing_rad
      M_PI / 4,  // width_bearing_rad
      0xf0       // sensitivity_threhold
  };
  perception::SpokeProcessing _SpokeProcessing(Alarm_Zone, SpokeProcess_data);

  perception::SpokeProcessRTdata SpokeProcess_RTdata;

  // sqlite3
  database db("radar.db");
  db << "CREATE TABLE radar (id integer primary key autoincrement not "
        "null, bearing_rad BLOB, range_m BLOB, x_m BLOB, y_m BLOB);";

  while (1) {
    MarineRadar_RTdata = _MarineRadar.getMarineRadarRTdata();

    SpokeProcess_RTdata =
        _SpokeProcessing
            .DetectionOnSpoke(MarineRadar_RTdata.spokedata,
                              SAMPLES_PER_SPOKE / 2,
                              MarineRadar_RTdata.spoke_azimuth_deg,
                              MarineRadar_RTdata.spoke_samplerange_m)
            .getSpokeProcessRTdata();

    std::cout << "spoke_azimuth_deg: " << MarineRadar_RTdata.spoke_azimuth_deg
              << std::endl;
    std::size_t num_surroundings_alarm =
        SpokeProcess_RTdata.surroundings_bearing_rad.size();
    if (num_surroundings_alarm == 0) {
      std::cout << "No surrounding in the Alarm Zone\n";
    } else {
      std::cout << "Surrounding in the Alarm Zone: \n";

      db << "INSERT INTO radar (bearing_rad, range_m, x_m, y_m) VALUES (?, ?, "
            "?, ?)"
         << SpokeProcess_RTdata.surroundings_bearing_rad
         << SpokeProcess_RTdata.surroundings_range_m
         << SpokeProcess_RTdata.surroundings_x_m
         << SpokeProcess_RTdata.surroundings_y_m;

      for (std::size_t i = 0; i != num_surroundings_alarm; ++i) {
        std::cout << "bearing_deg: "
                  << SpokeProcess_RTdata.surroundings_bearing_rad[i] * 180 /
                         M_PI
                  << "range_m: " << SpokeProcess_RTdata.surroundings_range_m[i]
                  << std::endl;
      }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void readsqlitedata() {
  database db("../../data/Sun Dec 22 16:22:33 2019.db");

  for (int _id = 1; _id != 19470; ++_id) {
    db << "SELECT bearing_rad, range_m, x_m, y_m from surroundings where id = "
          "?;"
       << _id >>
        [](std::unique_ptr<std::vector<double>> surroundings_bearing_rad,
           std::unique_ptr<std::vector<double>> surroundings_range_m,
           std::unique_ptr<std::vector<double>> surroundings_x_m,
           std::unique_ptr<std::vector<double>> surroundings_y_m) {
          if (surroundings_bearing_rad == nullptr ||
              surroundings_range_m == nullptr || surroundings_x_m == nullptr ||
              surroundings_y_m == nullptr) {
            std::cout << "No surrounding in the Alarm Zone\n";
          } else {
            for (std::size_t i = 0; i != surroundings_bearing_rad->size(); ++i)
              std::cout << " bearing_rad: " << surroundings_bearing_rad->at(i)
                        << " range_m: " << surroundings_range_m->at(i)
                        << " x_m: " << surroundings_x_m->at(i)
                        << " y_m: " << surroundings_y_m->at(i) << std::endl;
          }
        };
  }
}

int main() {
  // startRadarAndSpoke();
  readsqlitedata();
}
