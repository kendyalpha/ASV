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
      0.0,  // radar_x
      0.0   // radar_y
  };

  AlarmZone Alarm_Zone{
      0,    // start_range_m
      0,    // end_range_m
      0,    // center_bearing_rad
      0,    // width_bearing_rad
      0xac  // sensitivity_threhold
  };

  SpokeProcessRTdata SpokeProcess_RTdata;

  constexpr std::size_t size_array = 512;
  uint8_t spokedata[size_array];
  double spokeazimuth_deg = 230;
  double spoke_samplerange_m = 0.871;
  double vessel_xm = 0;
  double vessel_ym = 0;
  double vessel_theta_rad = 0;

  SpokeProcessing _SpokeProcessing(Alarm_Zone, Radar_Config);
  SpokeProcess_RTdata =
      _SpokeProcessing
          .DetectionOnSpoke(spokedata, size_array, spokeazimuth_deg,
                            spoke_samplerange_m, vessel_xm, vessel_ym,
                            vessel_theta_rad)
          .getSpokeProcessRTdata();
}