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
      : indicator_marine_radar(common::STATETOGGLE::IDLE),
        indicator_spoke_process(common::STATETOGGLE::IDLE),
        indicator_routeplanner(common::STATETOGGLE::IDLE),
        indicator_pathplanner(common::STATETOGGLE::IDLE),
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
  common::STATETOGGLE indicator_marine_radar;
  common::STATETOGGLE indicator_spoke_process;
  common::STATETOGGLE indicator_routeplanner;
  common::STATETOGGLE indicator_pathplanner;
  common::STATETOGGLE indicator_estimator;
  common::STATETOGGLE indicator_controller;
  common::STATETOGGLE indicator_sql;
  common::STATETOGGLE indicator_gps;
  common::STATETOGGLE indicator_utc;
  common::STATETOGGLE indicator_gui;
  common::STATETOGGLE indicator_stm32;
  common::STATETOGGLE indicator_socket;

  void check_spoke_process() {
    while (1) {
      if (indicator_spoke_process == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_spoke_process

  void check_routeplanner() {
    while (1) {
      if (indicator_routeplanner == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_routeplanner

  void check_pathplanner() {
    while (1) {
      if (indicator_pathplanner == common::STATETOGGLE::READY) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // check_pathplanner

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