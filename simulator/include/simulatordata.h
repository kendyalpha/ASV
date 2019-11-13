/*
*******************************************************************************
* simulatordata.h:
* define the data struct used in the simulator
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/
#ifndef _SIMULATORDATA_H_
#define _SIMULATORDATA_H_

#include "common/property/include/priority.h"
#include "common/property/include/vesseldata.h"

namespace ASV::simulation {

struct simulatorRTdata {
  // state toggle
  common::STATETOGGLE state_toggle;

  //
  Eigen::Matrix<double, 6, 1> X;
};

}  // namespace ASV::simulation

#endif /*_SIMULATORDATA_H_*/
