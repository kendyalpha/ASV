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
#include "common/fileIO/include/database.h"
#include "common/fileIO/include/jsonparse.h"
#include "common/logging/include/easylogging++.h"
#include "common/timer/include/timecounter.h"
#include "controller/include/controller.h"
#include "controller/include/trajectorytracking.h"
#include "estimator/include/estimator.h"
#include "messages/GUILink/include/guilink.h"
#include "messages/sensors/gpsimu/include/gps.h"
#include "messages/stm32/include/stm32_link.h"
#include "planner/lanefollow/include/LatticePlanner.h"
#include "planner/planner.h"
#include "simulator/include/simulator.h"

namespace ASV {
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_LOS;
constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_FRENET;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_FRENET;

constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = USEKALMAN::KALMANOFF;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;

}  // namespace ASV

#endif /* _CONFIG_H_ */