/*
***********************************************************************
* stm32data.h:
* header file to define the constant and real-time data for
* communication to stm32
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _STM32DATA_H_
#define _STM32DATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <string>
#include "common/communication/include/linkdata.h"

namespace ASV::messages {

enum class STM32STATUS {
  STANDBY = 0,    // 等待PC连接
  INITE,          // 初始化
  RUNNING,        // 正常运行
  MANUAL,         // 手动控制
  AUTO,           // 自动模式
  ALARM,          // 报警
  EMERGENCY_STOP  // 急停
};

struct stm32data {
  // UTC
  std::string UTC_time;

  // motor
  double command_u1;
  double command_u2;
  double feedback_u1;
  double feedback_u2;
  int feedback_pwm1;
  int feedback_pwm2;

  // remote control
  double RC_X;
  double RC_Y;
  double RC_Mz;

  // battery
  double voltage_b1;
  double voltage_b2;
  double voltage_b3;

  // stm32status
  STM32STATUS feedback_stm32status;
  STM32STATUS command_stm32status;

  // link status
  common::LINKSTATUS linkstatus;

};  // stm32data

}  // namespace ASV::messages

#endif /* _STM32DATA_H_ */