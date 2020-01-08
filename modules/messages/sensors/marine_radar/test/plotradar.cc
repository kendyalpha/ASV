#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sqlite_modern_cpp.h>

using namespace cv;
#include <iostream>
using namespace std;
int main(int argc, const char *argv[]) {
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

  Mat newimage;

  for (int _id = 10000; _id != 11500; ++_id) {
    static double previous_azimuth_deg = 0;
    db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
          "?;"
       << _id >>
        std::tie(surroundings_bearing_rad, surroundings_range_m,
                 surroundings_x_m, surroundings_y_m);
    db << "SELECT azimuth, sample_range, spokedata from radar where id = "
          "?;"
       << _id >>
        std::tie(spoke_azimuth_deg, spoke_samplerange_m, spokedata);

    db << "SELECT target_x, target_y, target_radius from target where id = "
          "?;"
       << _id >>
        std::tie(target_x, target_y, target_radius);

    // for (std::size_t i = 0; i != surroundings_bearing_rad.size(); ++i)
    //   std::cout << " bearing_rad: " << surroundings_bearing_rad.at(i)
    //             << " range_m: " << surroundings_range_m.at(i)
    //             << " x_m: " << surroundings_x_m.at(i)
    //             << " y_m: " << surroundings_y_m.at(i) << std::endl;

    if (spoke_azimuth_deg != previous_azimuth_deg) {
      Mat image1(1, 512, CV_8UC1, &spokedata);
      newimage.push_back(image1);
    }

    previous_azimuth_deg = spoke_azimuth_deg;
  }

  // We need an input image. (can be grayscale or color)
  if (argc < 2) {
    cerr << "We need an image to process here. Please run: colorMap "
            "[path_to_image]"
         << endl;
    return -1;
  }
  Mat img_in = imread(argv[1]);
  if (img_in.empty()) {
    cerr << "Sample image (" << argv[1]
         << ") is empty. Please adjust your path, so it points to a valid "
            "input image!"
         << endl;
    return -1;
  }

  // Holds the colormap version of the image:
  Mat img_color;

  // Apply the colormap:
  applyColorMap(newimage, img_color, COLORMAP_OCEAN);
  // Show the result:
  namedWindow("colorMap", WINDOW_NORMAL);
  imshow("colorMap", img_color);
  resizeWindow("colorMap", 5120, 100);

  waitKey(0);
  applyColorMap(img_in, img_color, COLORMAP_JET);

  imshow("colorMap", img_color);
  waitKey(0);
  return 0;
}

void readRadarData() {
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
  for (int _id = 1; _id != 19000; ++_id) {
    db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
          "?;"
       << _id >>
        std::tie(surroundings_bearing_rad, surroundings_range_m,
                 surroundings_x_m, surroundings_y_m);
    db << "SELECT azimuth, sample_range, spokedata from radar where id = "
          "?;"
       << _id >>
        std::tie(spoke_azimuth_deg, spoke_samplerange_m, spokedata);

    db << "SELECT target_x, target_y, target_radius from target where id = "
          "?;"
       << _id >>
        std::tie(target_x, target_y, target_radius);

    for (std::size_t i = 0; i != surroundings_bearing_rad.size(); ++i)
      std::cout << " bearing_rad: " << surroundings_bearing_rad.at(i)
                << " range_m: " << surroundings_range_m.at(i)
                << " x_m: " << surroundings_x_m.at(i)
                << " y_m: " << surroundings_y_m.at(i) << std::endl;
  }
}
