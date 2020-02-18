/*
***********************************************************************
* testparser.cc:
* uint test for data parser
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/dataparser.h"

int main() {
  const std::string folderp = "../../data/";
  const std::string config_path = "../../config/dbconfig.json";

  ASV::common::GPS_parser GPS_parser(folderp, config_path);
  auto read_gps = GPS_parser.parse_table(0, 10);
}