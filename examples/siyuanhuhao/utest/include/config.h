/*
***********************************************************************
* config.h: constexpr number
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <pthread.h>

#include "StateMonitor.h"
#include "common/communication/include/tcpserver.h"
#include "common/fileIO/include/jsonparse.h"
#include "common/fileIO/recorder/include/datarecorder.h"
#include "common/logging/include/easylogging++.h"
#include "common/timer/include/timecounter.h"
#include "modules/controller/include/controller.h"
#include "modules/controller/include/trajectorytracking.h"
#include "modules/estimator/include/estimator.h"
#include "modules/messages/GUILink/include/guilink.h"
#include "modules/messages/sensors/gpsimu/include/gps.h"
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"
#include "modules/messages/stm32/include/stm32_link.h"
#include "modules/perception/marine_radar/include/TargetTracking.h"
#include "modules/planner/path_planning/lanefollow/include/LatticePlanner.h"
#include "modules/planner/route_planning/include/RoutePlanning.h"
#include "modules/simulator/include/simulator.h"

namespace ASV {

const std::string parameter_json_path = "./../../properties/property.json";
constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr localization::USEKALMAN indicator_kalman =
    localization::USEKALMAN::KALMANOFF;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;
constexpr int max_num_targets = 20;

// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_DP;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_FRENET;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_AVOIDANCE;

// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_DP;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_FRENET;
constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_AVOIDANCE;

}  // namespace ASV

#endif /* _CONFIG_H_ */
