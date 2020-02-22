/*
***********************************************************************
* test_experimental.cc:
* uint test for data parser
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <iostream>
#include "../include/LatticePlanner.h"
#include "common/fileIO/include/utilityio.h"
#include "common/fileIO/recorder/include/dataparser.h"
#include "common/timer/include/timecounter.h"

int main() {
  const std::string folderp = "../../data/";
  const std::string config_path =
      "../../../../../common/fileIO/recorder/config/dbconfig.json";
  const double starting_time = 0;
  const double end_time = 10;

  // GPS
  ASV::common::GPS_parser GPS_parser(folderp, config_path);
  auto read_gps = GPS_parser.parse_table(starting_time, end_time);

  for (auto const &value : read_gps) {
    std::cout << value.local_time << " " << value.UTC << " " << value.UTM_zone
              << std::endl;
  }

  // stm32
  ASV::common::stm32_parser stm32_parser(folderp, config_path);
  auto read_stm32 = stm32_parser.parse_table(starting_time, end_time);

  // estimator
  ASV::common::estimator_parser estimator_parser(folderp, config_path);
  auto read_measurement =
      estimator_parser.parse_measurement_table(starting_time, end_time);
  auto read_error = estimator_parser.parse_error_table(starting_time, end_time);
  auto read_state = estimator_parser.parse_state_table(starting_time, end_time);

  // planner
  ASV::common::planner_parser planner_parser(folderp, config_path);
  auto read_route = planner_parser.parse_route_table(starting_time, end_time);

  // control
  ASV::common::control_parser control_parser(folderp, config_path);
  auto read_setpoint =
      control_parser.parse_setpoint_table(starting_time, end_time);
  auto read_TA = control_parser.parse_TA_table(starting_time, end_time);
}