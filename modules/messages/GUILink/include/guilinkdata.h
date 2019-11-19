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

#include <vector>

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include "common/communication/include/linkdata.h"
#include "common/property/include/priority.h"

namespace ASV::messages {

enum class GUISTATUS {
  STANDBY = 0,    // 等待PC连接
  INITE,          // 初始化
  RUNNING,        // 正常运行
  MANUAL,         // 手动控制
  AUTO,           // 自动模式
  ALARM,          // 报警
  EMERGENCY_STOP  // 急停
};

// real time data in gui link (岸基数据)
template <int num_thruster, int num_battery = 1>
struct guilinkRTdata {
  // UTC
  std::string UTC_time;

  GUISTATUS guistutus_PC2gui;  // send command to stm32
  GUISTATUS guistutus_gui2PC;  // send command to stm32

  // link status
  common::LINKSTATUS linkstatus;
  // indicator for automatic control mode
  int indicator_autocontrolmode;
  int indicator_windstatus;

  // GPS
  double latitude;
  double longitude;

  // state estimator
  Eigen::Matrix<double, 6, 1> State;
  double roll;
  double pitch;

  // controller
  Eigen::Matrix<int, num_thruster, 1> feedback_rotation;
  Eigen::Matrix<int, num_thruster, 1> feedback_alpha;
  // battery
  Eigen::Matrix<double, num_battery, 1> battery_voltage;

  Eigen::Vector3d setpoints;      // dp data
  double desired_speed;           // desired speed
  Eigen::Vector2d startingpoint;  // path planning data
  Eigen::Vector2d endingpoint;    // path planning data
  Eigen::Matrix<double, 2, 8> waypoints;

  Eigen::VectorXd WX;
  Eigen::VectorXd WY;
};

}  //  namespace ASV::messages

#endif /*_GUILINKDATA_H_*/