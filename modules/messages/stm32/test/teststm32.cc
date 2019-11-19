/*
*****************************************************************************
* teststm32.cc:
* unit test for gui communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include "../include/stm32_link.h"
#include "common/timer/include/timecounter.h"

using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  messages::stm32data _stm32data{
      "",                              // UTC_time
      0,                               // command_u1
      -10,                             // command_u2
      0,                               // feedback_u1
      0,                               // feedback_u2
      0,                               // feedback_pwm1
      0,                               // feedback_pwm2
      0,                               // RC_X
      0,                               // RC_Y
      0,                               // RC_Mz
      0,                               // voltage_b1
      0,                               // voltage_b2
      0,                               // voltage_b2
      messages::STM32STATUS::STANDBY,  // feedback_stm32status
      messages::STM32STATUS::STANDBY,  // command_stm32status
      common::LINKSTATUS::CONNECTED    // linkstatus;
  };

  common::timecounter _timer;
  messages::stm32_link _stm32_link(_stm32data, 115200);

  while (1) {
    std::string pt_utc = _timer.getUTCtime();
    long int et = _timer.timeelapsed();
    std::cout << _timer.getUTCtime() << " " << et << std::endl;

    static int n1 = 0;
    ++n1;
    _stm32data.UTC_time = pt_utc;
    _stm32data.command_u1 = n1;

    _stm32_link.setstm32data(_stm32data).stm32onestep();
    _stm32data = _stm32_link.getstmdata();

    std::cout << "status: " << static_cast<int>(_stm32data.feedback_stm32status)
              << std::endl;
    std::cout << "voltage_b1: " << _stm32data.voltage_b1 << std::endl;
    std::cout << "voltage_b2: " << _stm32data.voltage_b2 << std::endl;
    std::cout << "voltage_b3: " << _stm32data.voltage_b3 << std::endl;
    std::cout << "feedback_n1: " << _stm32data.feedback_u1 << std::endl;
    std::cout << "feedback_n2: " << _stm32data.feedback_u2 << std::endl;
    std::cout << "RC_X: " << _stm32data.RC_X << std::endl;
    std::cout << "RC_Y: " << _stm32data.RC_Y << std::endl;
    std::cout << "RC_Mz: " << _stm32data.RC_Mz << std::endl;

    // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}