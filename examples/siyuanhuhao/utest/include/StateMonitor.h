/*
***********************************************************************
* StateMonitor.h: State monitor
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _STATEMONITOR_H_
#define _STATEMONITOR_H_

#include <chrono>
#include <thread>

namespace ASV {

enum class STATETOGGLE {
  IDLE = 0,  //
  READY
};

class StateMonitor {
 public:
  StateMonitor()
      : indicator_planner(STATETOGGLE::IDLE),
        indicator_estimator(STATETOGGLE::IDLE),
        indicator_controller(STATETOGGLE::IDLE),
        indicator_sql(STATETOGGLE::IDLE),
        indicator_gps(STATETOGGLE::IDLE),
        indicator_utc(STATETOGGLE::IDLE),
        indicator_gui(STATETOGGLE::IDLE),
        indicator_stm32(STATETOGGLE::IDLE),
        indicator_socket(STATETOGGLE::IDLE) {}
  virtual ~StateMonitor() = default;

 protected:
  STATETOGGLE indicator_planner;
  STATETOGGLE indicator_estimator;
  STATETOGGLE indicator_controller;
  STATETOGGLE indicator_sql;
  STATETOGGLE indicator_gps;
  STATETOGGLE indicator_utc;
  STATETOGGLE indicator_gui;
  STATETOGGLE indicator_stm32;
  STATETOGGLE indicator_socket;

  void check_planner() {
    while (1) {
      if (indicator_planner == STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_planner

  void check_estimator() {
    while (1) {
      if (indicator_estimator == STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_estimator

  void check_controller() {
    while (1) {
      if (indicator_controller == STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_estimator
};

}  // namespace ASV

#endif /* _STATEMONITOR_H_ */