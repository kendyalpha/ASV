/*
*******************************************************************************
* guilinkdata.h:
* define the data struct used in the gui-communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _GUILINKDATA_H_
#define _GUILINKDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "controllerdata.h"
#include "estimatordata.h"
#include "gpsdata.h"
#include "motorclientdata.h"
#include "plannerdata.h"
#include "priority.h"
#include "winddata.h"

// real time data in gui link (岸基数据)
struct guilinkRTdata {
  int gui_connection;
  // indicator for automatic control mode
  int indicator_autocontrolmode;
  int indicator_windstatus;
  Eigen::Vector3d setpoints;      // dp data
  Eigen::Vector2d startingpoint;  // path planning data
  Eigen::Vector2d endingpoint;    // path planning data
  Eigen::Matrix<double, 2, 8> waypoints;
};

#endif /*_GUILINKDATA_H_*/