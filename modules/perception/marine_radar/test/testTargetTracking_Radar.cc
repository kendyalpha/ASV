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
  gp << "plot " << gp.file1d(x_y_radius)
     << " with circles title 'envelope' lc rgb 'blue' fs transparent solid "
        "0.15 noborder,"
     << gp.file1d(x_y) << "with points pt 1 title ''\n";
  gp.flush();

}  // readtime_tracking

int main() {
  // read data from database
  const std::string config_path =
      "../../../../../common/fileIO/recorder/config/dbconfig.json";
  const std::string db_folder = "../../data/";
  ASV::common::marineradar_parser marineradar_parser(db_folder, config_path);
  auto read_marineradar = marineradar_parser.parse_table(0, 100);

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

  ASV::perception::TargetTracking<> Target_Tracking(
      Alarm_Zone, SpokeProcess_data, TrackingTarget_Data, Clustering_Data);

  // plotting
  Gnuplot gp_heatmap;
  gp_heatmap << "set terminal x11 size 1000, 1000 0\n";
  gp_heatmap << "set title 'Radar Spoke'\n";
  gp_heatmap << "set tic scale 0\n";
  gp_heatmap << "set palette rgbformula 33,13,10\n";  // rainbow
  gp_heatmap << "set cbrange [0:255]\n";
  gp_heatmap << "set cblabel 'Score'\n";

  std::vector<std::tuple<double, double, int>> x_y_z_heatmap;

  Gnuplot gp_TT;
  gp_TT << "set terminal x11 size 1000, 1000 1\n";
  gp_TT << "set title 'Target Tracking'\n";
  gp_TT << "set xlabel 'Y(m)'\n";
  gp_TT << "set ylabel 'X(m)'\n";
  gp_TT << "set size ratio -1\n";
  gp_TT << "set xrange [-60:20]\n";
  std::vector<std::tuple<double, double, double>> x_y_radius;
  std::vector<std::pair<double, double>> x_y;

  //

  for (std::size_t i = 0; i != read_marineradar.size(); ++i) {
    double localtime = read_marineradar[i].local_time;
    double spoke_azimuth_deg = read_marineradar[i].azimuth_deg;
    double spoke_samplerange_m = read_marineradar[i].sample_range;
    std::vector<uint8_t> spokedata = read_marineradar[i].spokedata;

    auto TargetTracking_RTdata =
        Target_Tracking
            .AutoTracking(&spokedata[0], 512, spoke_azimuth_deg,
                          spoke_samplerange_m)
            .getTargetTrackerRTdata();

    static double previous_spokea_zimuth = 0;

    if (previous_spokea_zimuth != spoke_azimuth_deg) {
      previous_spokea_zimuth = spoke_azimuth_deg;

      switch (TargetTracking_RTdata.spoke_state) : {
          case ASV::perception::SPOKESTATE::OUTSIDE_ALARM_ZONE: {
            break;
          }
          case ASV::perception::SPOKESTATE::ENTER_ALARM_ZONE:
          case ASV::perception::SPOKESTATE::IN_ALARM_ZONE: {
            // prepare the data for heatmap
            for (std::size_t j = 0; j != spokedata.size(); ++j)
              x_y_z_heatmap.push_back({spoke_azimuth_deg,
                                       j * spoke_samplerange_m,
                                       static_cast<int>(spokedata[j])});
            break;
          }
          case ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE: {
            std::cout << "leave the alarm zone!\n";

            // heatmap

            readtime_heatmap(gp_heatmap, x_y_z_heatmap);

            x_y_z_heatmap.clear();

            // target tracking
            auto SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
            auto TargetDetection_RTdata =
                Target_Tracking.getTargetDetectionRTdata();

            for (std::size_t j = 0;
                 j != SpokeProcess_RTdata.surroundings_x_m.size(); ++j)
              x_y.push_back(
                  std::make_pair(SpokeProcess_RTdata.surroundings_y_m[j],
                                 SpokeProcess_RTdata.surroundings_x_m[j]));
            readtime_tracking(gp_TT, x_y, x_y_radius);
            x_y.clear();
            x_y_radius.clear();

            break;
          }
        }

      if (static_cast<int>(TargetTracking_RTdata.spoke_state) >= 1) {
        if (TargetTracking_RTdata.spoke_state ==
            ASV::perception::SPOKESTATE::ENTER_ALARM_ZONE) {
          std::cout << "enter the alarm zone!\n";
        }

        cv::Mat t_image(1, 64, CV_8UC1, &spokedata[0]);
        Alarm_image.push_back(t_image);

        if (TargetTracking_RTdata.spoke_state ==
            ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
          std::cout << "leave the alarm zone!\n";
          auto SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
          auto TargetDetection_RTdata =
              Target_Tracking.getTargetDetectionRTdata();

          matplotlibcpp::figure_size(800, 780);

          matplotlibcpp::plot(SpokeProcess_RTdata.surroundings_y_m,
                              SpokeProcess_RTdata.surroundings_x_m, ".");
          for (std::size_t index = 0;
               index != TargetDetection_RTdata.target_x.size(); ++index) {
            std::vector<double> circle_x;
            std::vector<double> circle_y;

            generatecircle(
                TargetDetection_RTdata.target_x[index],
                TargetDetection_RTdata.target_y[index],
                std::sqrt(TargetDetection_RTdata.target_square_radius[index]),
                circle_x, circle_y);

            matplotlibcpp::plot(circle_y, circle_x, "-");
          }

          std::cout << "index | target state | targets_x | targets_y | "
                       "targets_vx | targets_vy |\n";
          for (int index = 0; index != 20; ++index) {
            std::cout << index << " | "
                      << TargetTracking_RTdata.targets_state(index) << " | "
                      << TargetTracking_RTdata.targets_x(index) << " | "
                      << TargetTracking_RTdata.targets_y(index) << " | "
                      << TargetTracking_RTdata.targets_vx(index) << " | "
                      << TargetTracking_RTdata.targets_vy(index) << std::endl;

            std::vector<double> circle_x;
            std::vector<double> circle_y;

            generatecircle(
                TargetTracking_RTdata.targets_x[index],
                TargetTracking_RTdata.targets_y[index],
                std::sqrt(TargetTracking_RTdata.targets_square_radius[index]),
                circle_x, circle_y);

            matplotlibcpp::plot(circle_y, circle_x, "-");
          }

          matplotlibcpp::title("Clustering and MiniBall results");
          matplotlibcpp::xlabel("Y(m)");
          matplotlibcpp::ylabel("X(m)");
          matplotlibcpp::axis("equal");
          matplotlibcpp::xlim(-60, 20);
          matplotlibcpp::show();

          // // plotting
          // cv::Mat img_color;
          // cv::applyColorMap(Alarm_image, img_color, cv::COLORMAP_OCEAN);
          // cv::namedWindow("colorMap", cv::WINDOW_NORMAL);
          // cv::imshow("colorMap", img_color);
          // cv::resizeWindow("colorMap", 2000, 700);

          waitKey(0);
        }
      }
    }

    std::cout << "time: " << localtime << " azimuth: " << spoke_azimuth_deg
              << std::endl;
  }

  return 0;
}
