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

namespace ASV::perception {

// common parameter in marine radar
struct MarineRadarConfig {};

// real time data from marine radar
struct MarineRadarRTdata {
  Navico::Protocol::NRP::tMode pMode;
};

}  // namespace ASV::perception

#endif /* _MARINERADARDATA_H_ */
