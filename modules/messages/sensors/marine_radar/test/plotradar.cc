#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sqlite_modern_cpp.h>
#include "modules/perception/marine_radar/include/TargetTracking.h"

using namespace cv;
#include <iostream>

int main() {
  std::vector<double> surroundings_bearing_rad;
  std::vector<double> surroundings_range_m;
  std::vector<double> surroundings_x_m;
  std::vector<double> surroundings_y_m;
  std::vector<double> target_x;
  std::vector<double> target_y;
  std::vector<double> target_radius;
  double spoke_azimuth_deg;
  double spoke_samplerange_m;
  std::vector<uint8_t> spokedata;

  sqlite::database db("../../data/radar.db");

  // Spoke Processing
  ASV::perception::SpokeProcessdata SpokeProcess_data{
      0.1,  // sample_time
      0.0,  // radar_x
      0.0   // radar_y
  };

  ASV::perception::AlarmZone Alarm_Zone{
      10,        // start_range_m
      20,        // end_range_m
      0,         // center_bearing_rad
      M_PI / 4,  // width_bearing_rad
      0xf0       // sensitivity_threhold
  };

  ASV::perception::AlphaBetaData AlphaBeta_Data{
      0.1,  // sample_time
      0.1,  // alpha
      0.1   // beta
  };

  ASV::perception::ClusteringData Clustering_Data{
      1,  // p_radius
      2   // p_minumum_neighbors
  };

  ASV::perception::TargetTracking Target_Tracking(
      Alarm_Zone, SpokeProcess_data, AlphaBeta_Data, Clustering_Data);

  cv::Mat newimage;

  for (int _id = 1; _id != 11500; ++_id) {
    db << "SELECT azimuth, sample_range, spokedata from radar where id = "
          "?;"
       << _id >>
        std::tie(spoke_azimuth_deg, spoke_samplerange_m, spokedata);

    db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
          "?;"
       << _id >>
        std::tie(surroundings_bearing_rad, surroundings_range_m,
                 surroundings_x_m, surroundings_y_m);

    db << "SELECT target_x, target_y, target_radius from target where id = "
          "?;"
       << _id >>
        std::tie(target_x, target_y, target_radius);

    auto spokestate = Target_Tracking
                          .AutoTracking(&spokedata[0], 512, spoke_azimuth_deg,
                                        spoke_samplerange_m)
                          .getSpokeState();

    std::cout << static_cast<int>(spokestate) << std::endl;

    if (spokestate == ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
      auto SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
      auto TargetTracker_RTdata = Target_Tracking.getTargetTrackerRTdata();
    }

    // for (std::size_t i = 0;
    //      i != SpokeProcess_RTdata.surroundings_bearing_rad.size(); ++i)
    //   std::cout << " bearing_rad: "
    //             << SpokeProcess_RTdata.surroundings_bearing_rad.at(i)
    //             << " range_m: "
    //             << SpokeProcess_RTdata.surroundings_range_m.at(i)
    //             << " x_m: " << SpokeProcess_RTdata.surroundings_x_m.at(i)
    //             << " y_m: " << SpokeProcess_RTdata.surroundings_y_m.at(i)
    //             << std::endl;

    // if (spoke_azimuth_deg != previous_azimuth_deg) {
    //   cv::Mat image1(1, 512, CV_8UC1, &spokedata);
    //   newimage.push_back(image1);
    // }
  }

  // Holds the colormap version of the image:
  cv::Mat img_color;

  // Apply the colormap:
  cv::applyColorMap(newimage, img_color, cv::COLORMAP_OCEAN);
  // Show the result:
  cv::namedWindow("colorMap", cv::WINDOW_NORMAL);
  cv::imshow("colorMap", img_color);
  cv::resizeWindow("colorMap", 5120, 100);

  waitKey(0);
  cv::applyColorMap(newimage, img_color, cv::COLORMAP_JET);

  imshow("colorMap", img_color);
  waitKey(0);
  return 0;
}
