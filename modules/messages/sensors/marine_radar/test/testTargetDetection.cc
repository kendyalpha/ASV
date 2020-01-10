/*
****************************************************************************
* testTargetDetection.cc:
* example for marine radar and target detection
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <sqlite_modern_cpp.h>
#include <memory>
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"
#include "modules/perception/marine_radar/include/TargetTracking.h"

using namespace ASV;
using namespace sqlite;

void startRadarAndClustering() {
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

  perception::ClusteringData Clustering_Data{
      1,  // p_radius
      2   // p_minumum_neighbors
  };

  perception::TargetTracking Target_Tracking(Alarm_Zone, SpokeProcess_data,
                                             Clustering_Data);

  perception::TargetDetectionRTdata TargetDetection_RTdata;
  perception::SpokeProcessRTdata SpokeProcess_RTdata;

  // sqlite3
  database db("radar.db");
  db << "CREATE TABLE radar (id integer primary key autoincrement not "
        "null, azimuth DOUBLE, sample_range DOUBLE, spokedata BLOB);";
  db << "CREATE TABLE spoke (id integer primary key autoincrement not "
        "null, bearing_rad BLOB, range_m BLOB, x_m BLOB, y_m BLOB);";
  db << "CREATE TABLE target (id integer primary key autoincrement not "
        "null, target_x BLOB, target_y BLOB, target_radius BLOB);";

  while (1) {
    MarineRadar_RTdata = _MarineRadar.getMarineRadarRTdata();

    SpokeProcess_RTdata =
        Target_Tracking
            .AutoTracking(MarineRadar_RTdata.spokedata, SAMPLES_PER_SPOKE / 2,
                          MarineRadar_RTdata.spoke_azimuth_deg,
                          MarineRadar_RTdata.spoke_samplerange_m)
            .getSpokeProcessRTdata();

    TargetDetection_RTdata = Target_Tracking.getTargetDetectionRTdata();

    std::cout << "spoke_azimuth_deg: " << MarineRadar_RTdata.spoke_azimuth_deg
              << std::endl;
    db << "INSERT INTO radar (azimuth, sample_range, spokedata) "
          "VALUES (?, ?, ?)"
       << MarineRadar_RTdata.spoke_azimuth_deg
       << MarineRadar_RTdata.spoke_samplerange_m
       << std::vector<uint8_t>(
              &MarineRadar_RTdata.spokedata[0],
              &MarineRadar_RTdata.spokedata[SAMPLES_PER_SPOKE / 2]);

    db << "INSERT INTO spoke (bearing_rad, range_m, x_m, y_m) VALUES (?, ?, "
          "?, ?)"
       << SpokeProcess_RTdata.surroundings_bearing_rad
       << SpokeProcess_RTdata.surroundings_range_m
       << SpokeProcess_RTdata.surroundings_x_m
       << SpokeProcess_RTdata.surroundings_y_m;

    db << "INSERT INTO target (target_x, target_y, target_radius) "
          "VALUES (?, ?, ?)"
       << TargetDetection_RTdata.target_x << TargetDetection_RTdata.target_y
       << TargetDetection_RTdata.target_square_radius;

    // std::size_t num_surroundings_alarm =
    //     SpokeProcess_RTdata.surroundings_bearing_rad.size();
    // if (num_surroundings_alarm != 0) {
    //   std::cout << "Surrounding in the Alarm Zone: \n";

    //   for (std::size_t i = 0; i != num_surroundings_alarm; ++i) {
    //     std::cout << "bearing_deg: "
    //               << SpokeProcess_RTdata.surroundings_bearing_rad[i] * 180 /
    //                      M_PI
    //               << "range_m: " <<
    //               SpokeProcess_RTdata.surroundings_range_m[i]
    //               << std::endl;
    //   }
    // }

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void readsqlitedata() {
  database db("../../data/radar.db");

  // for (int _id = 1; _id != 1900; ++_id) {
  //   db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
  //         "?;"
  //      << _id >>
  //       [](std::unique_ptr<std::vector<double>> surroundings_bearing_rad,
  //          std::unique_ptr<std::vector<double>> surroundings_range_m,
  //          std::unique_ptr<std::vector<double>> surroundings_x_m,
  //          std::unique_ptr<std::vector<double>> surroundings_y_m) {
  //         if (surroundings_bearing_rad == nullptr ||
  //             surroundings_range_m == nullptr || surroundings_x_m == nullptr
  //             || surroundings_y_m == nullptr) {
  //           std::cout << "No surrounding in the Alarm Zone\n";
  //         } else {
  //           for (std::size_t i = 0; i != surroundings_bearing_rad->size();
  //           ++i)
  //             std::cout << " bearing_rad: " <<
  //             surroundings_bearing_rad->at(i)
  //                       << " range_m: " << surroundings_range_m->at(i)
  //                       << " x_m: " << surroundings_x_m->at(i)
  //                       << " y_m: " << surroundings_y_m->at(i) << std::endl;
  //         }
  //       };
  // }

  // read radar spoke data
  for (int _id = 1; _id != 30000; ++_id) {
    db << "SELECT azimuth, sample_range, spokedata from radar where id = "
          "?;"
       << _id >>
        [](std::unique_ptr<double> spoke_azimuth_deg,
           std::unique_ptr<double> spoke_samplerange_m,
           std::unique_ptr<std::vector<uint8_t>> spokedata) {
          if (spoke_azimuth_deg == nullptr || spoke_samplerange_m == nullptr ||
              spokedata == nullptr) {
            std::cout << "No spoke data\n";
          } else {
            std::cout << " spoke_azimuth_deg: " << *spoke_azimuth_deg
                      << " spoke_samplerange_m: " << *spoke_samplerange_m
                      << " spokedata:\n";
            for (std::size_t i = 0; i != spokedata->size(); ++i)
              printf("0x%x\n", spokedata->at(i));
          }
        };
  }

  // read target data
  for (int _id = 1; _id != 30000; ++_id) {
    db << "SELECT target_x, target_y, target_radius from target where id = "
          "?;"
       << _id >>
        [](std::unique_ptr<std::vector<double>> target_x,
           std::unique_ptr<std::vector<double>> target_y,
           std::unique_ptr<std::vector<double>> target_radius) {
          if (target_x == nullptr || target_y == nullptr ||
              target_radius == nullptr) {
            std::cout << "No target data\n";
          } else {
            for (std::size_t i = 0; i != target_x->size(); ++i)
              std::cout << " target_x: " << target_x->at(i)
                        << " target_y: " << target_y->at(i)
                        << " target_radius: " << target_radius->at(i)
                        << std::endl;
          }
        };
  }
}

int main() {
  // startRadarAndClustering();
  readsqlitedata();
}
