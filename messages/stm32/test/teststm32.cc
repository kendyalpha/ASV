/*
*****************************************************************************
* teststm32.cc:
* unit test for gui communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include "stm32_link.h"
#include "timecounter.h"

using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  stm32data _stm32data{
      "",                    // UTC_time
      0,                     // command_n1
      0,                     // command_n2
      0,                     // feedback_n1
      0,                     // feedback_n2
      0,                     // RC_X
      0,                     // RC_Y
      0,                     // RC_Mz
      RCMODE::MANUAL,        // rcmode
      0,                     // voltage_b1
      0,                     // voltage_b2
      0,                     // voltage_b2
      STM32STATUS::STANDBY,  // stm32status
      LINKSTATUS::CONNECTED  // linkstatus;
  };

  while (1) {
    stm32_link _stm32_link(_stm32data, 115200);
    _stm32data = _stm32_link.stm32onestep().getstmdata();
  }
}