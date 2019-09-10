/*
*******************************************************************************
* testplanner.cc:
* unit test for path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "planner.h"
#include "utilityio.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  plannerdata _plannerdata{
      0.1,  // sample_time
  };
  plannerRTdata _plannerRTdata{
      0, 0,
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  planner _planner(_plannerdata);
}