/*
****************************************************************************
* testTargetTracking_Radar.cc:
* unit test for Target Tracking using Radar data
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <iostream>
#include <thread>
#include "../include/TargetTracking.h"
#include "common/fileIO/recorder/include/dataparser.h"
#include "common/plotting/include/gnuplot-iostream.h"

// real time heatmap for target tracking
void readtime_heatmap(
    Gnuplot &gp, const std::vector<std::tuple<double, double, int>> &x_y_z) {
  gp << "plot " << gp.file1d(x_y_z) << " with image\n";
  gp.flush();

}  // readtime_heatmap

// real time plotting for target tracking
void readtime_tracking(
    Gnuplot &gp, const std::vector<std::pair<double, double>> &x_y,
    const std::vector<std::tuple<double, double, double>> &x_y_radius_detect,
    const std::vector<std::tuple<double, double, double>> &x_y_radius_target) {
  gp << "plot " << gp.file1d(x_y_radius_detect)
     << " with circles title 'envelope' lc rgb 'blue' fs transparent solid "
        "0.15 noborder,"
     << gp.file1d(x_y_radius_target)
     << " with circles title 'targets' lw 2 lc rgb 'red'," << gp.file1d(x_y)
     << "with points pt 1 lc rgb 'black' title 'spoke'\n";
  gp.flush();

}  // readtime_tracking

int main() {
  // read data from database
  const std::string config_path =
      "../../../../../common/fileIO/recorder/config/dbconfig.json";
  const std::string db_folder = "../../data/";
  ASV::common::marineradar_parser marineradar_parser(db_folder, config_path);
  auto read_marineradar = marineradar_parser.parse_table(10, 200);

  // Spoke Processing
  ASV::perception::SpokeProcessdata SpokeProcess_data{
      0.1,  // sample_time
      0.0,  // radar_x
      0.0   // radar_y
  };

  ASV::perception::AlarmZone Alarm_Zone{
      5,            // start_range_m
      50,           // end_range_m
      -0.5 * M_PI,  // center_bearing_rad
      M_PI,         // width_bearing_rad
      0x90          // sensitivity_threhold
  };

  ASV::perception::TrackingTargetData TrackingTarget_Data{
      1,    // min_squared_radius
      36,   // max_squared_radius
      0.8,  // speed_threhold
      20,   // max_speed
      4,    // max_acceleration
      600,  // max_roti
      1,    // safe_distance
      0.5,  // K_radius
      1,    // K_delta_speed
      1     // K_delta_yaw;
  };

  ASV::perception::ClusteringData Clustering_Data{
      4.4,  // p_radius
      2     // p_minumum_neighbors
  };

  const int max_num_targets = 20;

  ASV::perception::TargetTracking<max_num_targets> Target_Tracking(
      Alarm_Zone, SpokeProcess_data, TrackingTarget_Data, Clustering_Data);

  // plotting
  Gnuplot gp_heatmap;
  gp_heatmap << "set terminal x11 size 1000, 1000 0\n";
  gp_heatmap << "set title 'Radar Spoke'\n";
  gp_heatmap << "set tic scale 0\n";
  gp_heatmap << "set palette rgbformula 33,13,10\n";  // rainbow
  gp_heatmap << "set cbrange [0:255]\n";
  gp_heatmap << "set cblabel 'Score'\n";
  gp_heatmap << "set xrange [180:360]\n";
  gp_heatmap << "set yrange [0:100]\n";

  std::vector<std::tuple<double, double, int>> x_y_z_heatmap;

  Gnuplot gp_TT;
  gp_TT << "set terminal x11 size 1000, 1000 1\n";
  gp_TT << "set title 'Target Tracking'\n";
  gp_TT << "set xlabel 'Y(m)'\n";
  gp_TT << "set ylabel 'X(m)'\n";
  gp_TT << "set size ratio -1\n";
  gp_TT << "set xrange [-60:20]\n";

  std::vector<std::pair<double, double>> x_y_spoke;
  std::vector<std::tuple<double, double, double>> x_y_radius_detect;
  std::vector<std::tuple<double, double, double>> x_y_radius_target;

  // calculation
  for (std::size_t i = 0; i != read_marineradar.size(); ++i) {
    double localtime = read_marineradar[i].local_time;
    double spoke_azimuth_deg = read_marineradar[i].azimuth_deg;
    double spoke_samplerange_m = read_marineradar[i].sample_range;
    std::vector<uint8_t> spokedata = read_marineradar[i].spokedata;

    auto TargetTracking_RTdata =
        Target_Tracking
            .AutoTracking(&spokedata[0], spokedata.size(), spoke_azimuth_deg,
                          spoke_samplerange_m)
            .getTargetTrackerRTdata();

    static double previous_spokea_zimuth = 0;

    if (previous_spokea_zimuth != spoke_azimuth_deg) {
      previous_spokea_zimuth = spoke_azimuth_deg;

      switch (TargetTracking_RTdata.spoke_state) {
        case ASV::perception::SPOKESTATE::OUTSIDE_ALARM_ZONE: {
          break;
        }
        case ASV::perception::SPOKESTATE::ENTER_ALARM_ZONE:
        case ASV::perception::SPOKESTATE::IN_ALARM_ZONE: {
          // prepare the data for heatmap
          for (std::size_t j = 0; j != spokedata.size(); ++j)
            x_y_z_heatmap.push_back({spoke_azimuth_deg, j * spoke_samplerange_m,
                                     static_cast<int>(spokedata[j])});
          break;
        }
        case ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE: {
          std::cout << "leave the alarm zone!\n";
          std::cout << "index | target state | targets_x | targets_y | "
                       "targets_vx | targets_vy |\n";
          for (int j = 0; j != max_num_targets; ++j) {
            std::cout << j << " | " << TargetTracking_RTdata.targets_state(j)
                      << " | " << TargetTracking_RTdata.targets_x(j) << " | "
                      << TargetTracking_RTdata.targets_y(j) << " | "
                      << TargetTracking_RTdata.targets_vx(j) << " | "
                      << TargetTracking_RTdata.targets_vy(j) << std::endl;
          }
          // heatmap
          readtime_heatmap(gp_heatmap, x_y_z_heatmap);
          x_y_z_heatmap.clear();

          // target tracking
          auto SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
          auto TargetDetection_RTdata =
              Target_Tracking.getTargetDetectionRTdata();

          for (std::size_t j = 0;
               j != SpokeProcess_RTdata.surroundings_x_m.size(); ++j)
            x_y_spoke.push_back(
                std::make_pair(SpokeProcess_RTdata.surroundings_y_m[j],
                               SpokeProcess_RTdata.surroundings_x_m[j]));

          for (std::size_t j = 0; j != TargetDetection_RTdata.target_x.size();
               ++j)
            x_y_radius_detect.push_back(
                {TargetDetection_RTdata.target_y[j],
                 TargetDetection_RTdata.target_x[j],
                 std::sqrt(TargetDetection_RTdata.target_square_radius[j])});

          for (int j = 0; j != max_num_targets; ++j)
            x_y_radius_target.push_back(
                {TargetTracking_RTdata.targets_y(j),
                 TargetTracking_RTdata.targets_x(j),
                 std::sqrt(TargetTracking_RTdata.targets_square_radius(j))});

          readtime_tracking(gp_TT, x_y_spoke, x_y_radius_detect,
                            x_y_radius_target);
          x_y_spoke.clear();
          x_y_radius_detect.clear();
          x_y_radius_target.clear();

          std::cin.get();
          break;
        }
      }  // end switch
    }    // end if
  }      // end for loop

  return 0;
}
