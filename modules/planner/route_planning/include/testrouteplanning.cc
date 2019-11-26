/*
*******************************************************************************
* testplanner.cc:
* unit test for path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "common/fileIO/include/utilityio.h"
#include "planner.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  ASV::plannerdata _plannerdata{
      0.1,  // sample_time
  };
  ASV::plannerRTdata _plannerRTdata{
      0, 0,
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  ASV::planner _planner(_plannerdata);
}