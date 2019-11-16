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

#include "common/property/include/priority.h"

namespace ASV {

class StateMonitor {
 public:
  StateMonitor()
      : indicator_planner(common::STATETOGGLE::IDLE),
        indicator_estimator(common::STATETOGGLE::IDLE),
        indicator_controller(common::STATETOGGLE::IDLE),
        indicator_sql(common::STATETOGGLE::IDLE),
        indicator_gps(common::STATETOGGLE::IDLE),
        indicator_utc(common::STATETOGGLE::IDLE),
        indicator_gui(common::STATETOGGLE::IDLE),
        indicator_stm32(common::STATETOGGLE::IDLE),
        indicator_socket(common::STATETOGGLE::IDLE) {}
  virtual ~StateMonitor() = default;

 protected:
  common::STATETOGGLE indicator_planner;
  common::STATETOGGLE indicator_estimator;
  common::STATETOGGLE indicator_controller;
  common::STATETOGGLE indicator_sql;
  common::STATETOGGLE indicator_gps;
  common::STATETOGGLE indicator_utc;
  common::STATETOGGLE indicator_gui;
  common::STATETOGGLE indicator_stm32;
  common::STATETOGGLE indicator_socket;

  void check_planner() {
    while (1) {
      if (indicator_planner == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_planner

  void check_estimator() {
    while (1) {
      if (indicator_estimator == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_estimator

  void check_controller() {
    while (1) {
      if (indicator_controller == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_estimator
};

}  // namespace ASV

#endif /* _STATEMONITOR_H_ */