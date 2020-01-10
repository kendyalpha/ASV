#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sqlite_modern_cpp.h>
#include "modules/perception/marine_radar/include/TargetTracking.h"

using namespace cv;
#include <iostream>
#include <thread>
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
      M_PI / 2,  // width_bearing_rad
      0xc0       // sensitivity_threhold
  };

  ASV::perception::ClusteringData Clustering_Data{
      1,  // p_radius
      2   // p_minumum_neighbors
  };

  ASV::perception::TargetTracking<> Target_Tracking(
      Alarm_Zone, SpokeProcess_data, Clustering_Data);

  cv::Mat Alarm_image;

  for (int _id = 11000; _id != 51500; ++_id) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    static double previous_spokea_zimuth = 0;

    if (previous_spokea_zimuth != spoke_azimuth_deg) {
      previous_spokea_zimuth = spoke_azimuth_deg;

      if (static_cast<int>(spokestate) >= 1) {
        if (spokestate == ASV::perception::SPOKESTATE::ENTER_ALARM_ZONE) {
          std::cout << "enter the alarm zone!\n";
        }

        cv::Mat t_image(1, 64, CV_8UC1, &spokedata[0]);
        Alarm_image.push_back(t_image);

        if (spokestate == ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
          std::cout << "leave the alarm zone!\n";
          auto SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
          auto getTargetDetection_RTdata =
              Target_Tracking.getTargetDetectionRTdata();

          cv::Mat img_color;
          cv::applyColorMap(Alarm_image, img_color, cv::COLORMAP_OCEAN);
          cv::namedWindow("colorMap", cv::WINDOW_NORMAL);
          cv::imshow("colorMap", img_color);
          cv::resizeWindow("colorMap", 2000, 700);
          waitKey(0);

          // for (std::size_t i = 0;
          //      i != SpokeProcess_RTdata.surroundings_bearing_rad.size(); ++i)
          //   std::cout << " bearing_rad: "
          //             << SpokeProcess_RTdata.surroundings_bearing_rad.at(i)
          //             << " range_m: "
          //             << SpokeProcess_RTdata.surroundings_range_m.at(i)
          //             << " x_m: " <<
          //             SpokeProcess_RTdata.surroundings_x_m.at(i)
          //             << " y_m: " <<
          //             SpokeProcess_RTdata.surroundings_y_m.at(i)
          //             << std::endl;

          Alarm_image = cv::Mat();
        }
      }
    }
  }

  return 0;
}
