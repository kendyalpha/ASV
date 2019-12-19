/*
****************************************************************************
* MarineRadarData.h:
* Marine radar for target tracking
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _MARINERADARDATA_H_
#define _MARINERADARDATA_H_

#include <ClientErrors.h>
#include <Feature.h>
#include <FeatureManager.h>
#include <ImageClient.h>
#include <ImageClientObserver.h>
#include <MultiRadarClient.h>
#include <NavRadarProtocol.h>
#include <PPIController.h>
#include <TargetTrackingClient.h>

#include "common/property/include/priority.h"

namespace ASV::messages {

enum class GUARDZONE {
  eGuardZone1 = 0,  // the first guard zone
  eGuardZone2
};

// common parameter in marine radar
struct MarineRadarConfig {};

// real time data from marine radar
struct MarineRadarRTdata {
  // state toggle
  common::STATETOGGLE state_toggle;

  double spoke_azimuth_deg;
  double spoke_samplerange_m;
  uint8_t spokedata[SAMPLES_PER_SPOKE / 2];
};

}  // namespace ASV::messages

#endif /* _MARINERADARDATA_H_ */
