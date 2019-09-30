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

#include <string>
#include "linkdata.h"

namespace ASV {

enum class STM32STATUS {
  STANDBY = 0,    // 等待PC连接
  INITE,          // 初始化
  RUNNING,        // 正常运行
  ALARM,          // 报警
  EMERGENCY_STOP  // 急停
};

enum class RCMODE {
  MANUAL = 0,  // 手动控制
  AUTO         // 自动模式
};

struct stm32data {
  // UTC
  std::string UTC_time;

  // motor
  int command_n1;
  int command_n2;
  int feedback_n1;
  int feedback_n2;

  // remote control
  double RC_X;
  double RC_Y;
  double RC_Mz;
  RCMODE rcmode;

  // battery
  double voltage_b1;
  double voltage_b2;
  double voltage_b3;

  // stm32status
  STM32STATUS stm32status;

  // link status
  LINKSTATUS linkstatus;

};  // stm32data

}  // end namespace ASV

#endif /* _STM32DATA_H_ */