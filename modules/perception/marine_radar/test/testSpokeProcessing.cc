/*
****************************************************************************
* testSpokeProcessing.cc:
* obstacle detection using spoke data from marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include "../include/SpokeProcessing.h"

using namespace ASV::perception;

int main() {
  RadarConfig Radar_Config{
      -1.0,  // radar_x
      0.0    // radar_y
  };

  AlarmZone Alarm_Zone{
      10,        // start_range_m
      20,        // end_range_m
      0,         // center_bearing_rad
      M_PI / 2,  // width_bearing_rad
      0xac       // sensitivity_threhold
  };

  SpokeProcessRTdata SpokeProcess_RTdata;

  constexpr std::size_t size_array = 512;
  uint8_t spokedata[size_array] = {0xc0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
                                   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                   0xff, 0xff, 0xff, 0xca, 0x01, 0x30, 0x45};
  double spokeazimuth_deg = -30;
  double spoke_samplerange_m = 0.781;
  double vessel_x_m = 0;
  double vessel_y_m = 0;
  double vessel_theta_rad = 0;

  SpokeProcessing _SpokeProcessing(Alarm_Zone, Radar_Config);
  SpokeProcess_RTdata =
      _SpokeProcessing
          .DetectionOnSpoke(spokedata, size_array, spokeazimuth_deg,
                            spoke_samplerange_m, vessel_x_m, vessel_y_m,
                            vessel_theta_rad)
          .getSpokeProcessRTdata();

  std::size_t num = SpokeProcess_RTdata.surroundings_bearing_rad.size();
  assert(num == SpokeProcess_RTdata.surroundings_range_m.size());
  assert(num == SpokeProcess_RTdata.surroundings_x_m.size());
  assert(num == SpokeProcess_RTdata.surroundings_y_m.size());
  for (std::size_t i = 0; i != num; ++i) {
    std::cout << "bearing_rad: "
              << SpokeProcess_RTdata.surroundings_bearing_rad[i] << std::endl;
    std::cout << "range_m: " << SpokeProcess_RTdata.surroundings_range_m[i]
              << std::endl;
    std::cout << "x: " << SpokeProcess_RTdata.surroundings_x_m[i] << std::endl;
    std::cout << "y: " << SpokeProcess_RTdata.surroundings_y_m[i] << std::endl;
  }
}