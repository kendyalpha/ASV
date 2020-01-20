#include "../include/datarecorder.h"
#include "modules/messages/sensors/gpsimu/include/gpsdata.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  LOG(INFO) << "The program has started!";

  const std::string folderp = "../../data/";

  // controller
  constexpr int m = 4;
  Eigen::Matrix<int, m, 1> azimuth;
  Eigen::Matrix<int, m, 1> rotation;
  azimuth << 100, 200, 300, 400;
  rotation << 900, 800, 700, 600;

  ASV::common::controller_db controller_db(folderp);
  controller_db.create_table();
  controller_db.update_setpoint_table(1, 0, 2, 3, 4, 5);
  controller_db.update_TA_table<m>(1, 0, 2, 3, 4, 5, azimuth, rotation);

  // GPS
  ASV::messages::gpsRTdata gps_data{
      1,    // UTC
      2,    // latitude
      3,    // longitude
      4,    // heading
      5,    // pitch
      6,    // roll
      7,    // altitude
      8,    // Ve
      9,    // Vn
      10,   // roti
      11,   // status
      12,   // UTM_x
      13,   // UTM_y
      "0n"  // UTM_zone
  };
  ASV::common::gps_db gps_db(folderp);
  gps_db.create_table();
  gps_db.update_table(gps_data.UTC, gps_data.latitude, gps_data.longitude,
                      gps_data.heading, gps_data.pitch, gps_data.roll,
                      gps_data.altitude, gps_data.Ve, gps_data.Vn,
                      gps_data.roti, gps_data.status, gps_data.UTM_x,
                      gps_data.UTM_y, gps_data.UTM_zone);

  // estimator
  ASV::common::estimator_db estimator_db(folderp);
  estimator_db.create_table();
  estimator_db.update_measurement_table(1, 2, 3, 4, 5, 6);
  estimator_db.update_state_table(1, 2, 3, 4, 5, 6, 7, 8, 9);
  estimator_db.update_error_table(6, 5, 4, 3, 2, 1);

  // planner
  ASV::common::planner_db planner_db(folderp);
  planner_db.create_table();
  planner_db.update_routeplanner_table(1, 2, 3, 4, 5, 6);

  // wind
  ASV::common::wind_db wind_db(folderp);
  wind_db.create_table();
  wind_db.update_table(1, 2);

  LOG(INFO) << "Shutting down.";
}