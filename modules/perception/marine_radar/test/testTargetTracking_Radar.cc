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

int main() {
  sqlite::database db("../../data/radar_new.db");
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

  cv::Mat Alarm_image;

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

  double timestamp0 = 0;
  db << "SELECT DATETIME from radar where id = ?;" << 100 >>
      [&](std::string str_datetime) { timestamp0 = std::stod(str_datetime); };

  double timestamp = 0;

  for (int _id = 200; _id != 117000; ++_id) {
    std::string str_datetime;

    db << "SELECT DATETIME, azimuth, sample_range, spokedata from radar where "
          "id = ?;"
       << _id >>
        std::tie(str_datetime, spoke_azimuth_deg, spoke_samplerange_m,
                 spokedata);

    // db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
    //       "?;"
    //    << _id >>
    //     std::tie(surroundings_bearing_rad, surroundings_range_m,
    //              surroundings_x_m, surroundings_y_m);

    // db << "SELECT target_x, target_y, target_radius from target where id = "
    //       "?;"
    //    << _id >>
    //     std::tie(target_x, target_y, target_radius);

    auto TargetTracking_RTdata =
        Target_Tracking
            .AutoTracking(&spokedata[0], 512, spoke_azimuth_deg,
                          spoke_samplerange_m)
            .getTargetTrackerRTdata();

    timestamp = (std::stod(str_datetime) - timestamp0) * 86400;

    static double previous_spokea_zimuth = 0;

    if (previous_spokea_zimuth != spoke_azimuth_deg) {
      previous_spokea_zimuth = spoke_azimuth_deg;

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
          // waitKey(0);

          // Alarm_image = cv::Mat();
        }
      }
    }
  }

  return 0;
}
