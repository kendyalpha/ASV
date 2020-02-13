/*
****************************************************************************
* MarineRadar.h:
* Marine radar for spoke updating, PPI display, target tracking
* and guard zone alarm, etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

// TODO:
// 为何雷达官方软件能显示连续的云图，我们采集的数据是离散的

#ifndef _MARINERADAR_H_
#define _MARINERADAR_H_

#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "MarineRadarData.h"
#include "MultiRadar.h"

namespace ASV::messages {

class MarineRadar
    : public Navico::Protocol::NRP::iImageClientStateObserver,
      public Navico::Protocol::NRP::iImageClientSpokeObserver,
      public Navico::Protocol::NRP::iFeatureObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientStateObserver {
 public:
  MarineRadar()
      : m_pImageClient(nullptr),
        m_pMode(nullptr),
        m_pSetup(nullptr),
        m_pSetupExtended(nullptr),
        m_pProperties(nullptr),
        m_pConfiguration(nullptr),
        m_pAdvancedSTCState(nullptr),
        m_pGuardZoneAlarms(nullptr),
        m_pRadarError(nullptr),
        m_PixelCellSize_mm(0),
        m_pTargetClient(nullptr),
        m_pTargetAlarmSetup(nullptr),
        m_pTargetProperties(nullptr),
        m_pTargets(nullptr),
        MarineRadar_RTdata({
            common::STATETOGGLE::IDLE,  // state_toggle
            0.0,                        // spoke_azimuth_deg
            0.0,                        // spoke_samplerange_m
            {0x00, 0x00, 0x00}          // spokedata
        }) {
    m_pImageClient = new Navico::Protocol::NRP::tImageClient();
    m_pTargetClient = new Navico::Protocol::NRP::tTargetTrackingClient();
    InitProtocolData();
    m_pMultiRadar = new MultiRadar();
  }
  ~MarineRadar() {
    delete m_pMultiRadar;

    // cleanup target-tracking
    delete m_pTargetClient;
    delete m_pTargetAlarmSetup;
    delete m_pTargetProperties;
    delete[] m_pTargets;

    // cleanup image handlling
    delete m_pImageClient;
    delete m_pMode;
    delete m_pSetup;
    delete m_pSetupExtended;
    delete m_pProperties;
    delete m_pConfiguration;
    delete m_pAdvancedSTCState;
    delete m_pRadarError;
    delete[] m_pGuardZoneAlarms;
  }

  void StartMarineRadar() {
    // unlock the sdk
    while (1) {
      if (m_pMultiRadar->InitiateUnlock() > 0) {
        MarineRadar_RTdata.state_toggle = common::STATETOGGLE::READY;
        break;
      } else
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    MultiRadar_Connect(true);
  }

  Navico::Protocol::NRP::Spoke::t9174Spoke getSpoke() const noexcept {
    return m_pSpoke;
  }

  MarineRadarRTdata getMarineRadarRTdata() const noexcept {
    return MarineRadar_RTdata;
  }

 private:
  //-------------------------------------------------------------------------
  //  Observer Callbacks
  //-------------------------------------------------------------------------
  // iImageClientSpokeObserver callbacks (real time)
  virtual void UpdateSpoke(
      const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke) {
    // copy pSpoke to m_pSpoke
    m_pSpoke.header = pSpoke->header;
    memcpy(m_pSpoke.data, pSpoke->data, SAMPLES_PER_SPOKE / 2);

    // update MarineRadar_RTdata
    MarineRadar_RTdata.spoke_azimuth_deg =
        pSpoke->header.spokeAzimuth * 360 / 4096.0;
    MarineRadar_RTdata.spoke_samplerange_m =
        Navico::Protocol::NRP::Spoke::GetSampleRange_mm(pSpoke->header) /
        1000.0;
    memcpy(MarineRadar_RTdata.spokedata, pSpoke->data, SAMPLES_PER_SPOKE / 2);

  }  // UpdateSpoke

  // iImageClientStateObserver callbacks
  virtual void UpdateMode(const Navico::Protocol::NRP::tMode* pMode) {
    std::cout << "\n # Mode updating #.....................\n";
    switch (pMode->state) {
      case Navico::Protocol::NRP::eOff:
        std::cout << "state: off\n";
        std::cout << "Scanner Power: false\n";
        std::cout << "Scanner Transmit: false\n";
        break;
      case Navico::Protocol::NRP::eStandby:
        std::cout << "state: standby\n";
        std::cout << "Scanner Power: true\n";
        std::cout << "Scanner Transmit: false\n";
        break;
      case Navico::Protocol::NRP::eTransmit:
        std::cout << "state: transmit\n";
        std::cout << "Scanner Power: true\n";
        std::cout << "Scanner Transmit: true\n";
        break;
      case Navico::Protocol::NRP::eWarming:
        std::cout << "state: Warming up\n";
        std::cout << "Scanner Power: true\n";
        std::cout << "Scanner Transmit: false\n";
        break;
      case Navico::Protocol::NRP::eNoScanner:
        std::cout << "state: no scanner\n";
        std::cout << "Scanner Power: false\n";
        std::cout << "Scanner Transmit: false\n";
        break;
      case Navico::Protocol::NRP::eDetectingScanner:
        std::cout << "state: Detecting Scanner\n";
        std::cout << "Scanner Power: true\n";
        std::cout << "Scanner Transmit: false\n";
        break;
      default:
        std::cout << "state: Undefined\n";
        break;
    }

    if (pMode->warmupTime_sec == 0) {
      std::cout << "Scanner Timeout = " << pMode->ttCount_sec << std::endl;
    } else {
      std::cout << "Scanner Timeout = " << pMode->warmupTime_sec << std::endl;
    }
    std::cout << "checkTimedTxMode: " << pMode->ttState << std::endl;

  }  // UpdateMode

  virtual void UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup) {
    std::cout << "\n # Setup updating #.....................\n";
    std::cout << "Scanner Range: " << pSetup->range_dm / 10.0 << " m\n";
    // Gain controls update
    uint32_t gainType = pSetup->gain[Navico::Protocol::NRP::eSetupGain].type;
    assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
    std::cout << "Radar Gain: "
              << std::to_string(
                     pSetup->gain[Navico::Protocol::NRP::eSetupGain].value)
              << std::endl;

    // Sea clutter controls update
    gainType = pSetup->gain[Navico::Protocol::NRP::eSetupSea].type;
    assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
    std::cout << "SeaClutter: "
              << std::to_string(
                     pSetup->gain[Navico::Protocol::NRP::eSetupSea].value)
              << std::endl;

    // Rain controls update
    gainType = pSetup->gain[Navico::Protocol::NRP::eSetupRain].type;
    assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
    std::cout << "Rain: "
              << std::to_string(
                     pSetup->gain[Navico::Protocol::NRP::eSetupRain].value)
              << std::endl;

    // FTC controls update
    std::cout << "FTC: " << std::to_string(pSetup->ftc.value) << std::endl;

    // Interference Reject controls update
    std::cout << "IRLevel: " << std::to_string(pSetup->interferenceReject)
              << std::endl;

    // Target Boost controls update
    std::cout << "TargetBoost: " << std::to_string(pSetup->targetBoost)
              << std::endl;

    // Target Stretch controls update
    std::cout << "TargetStretch: " << std::to_string(pSetup->targetStretch)
              << std::endl;

    // Pulse Group
    std::cout << "PulseWidth: " << std::to_string(pSetup->pwLength_ns)
              << std::endl;
    std::cout << "CoarseTune: " << std::to_string(pSetup->coarseTune)
              << std::endl;
    std::cout << "FineTune: " << std::to_string(pSetup->fineTune) << std::endl;
    std::cout << "UseModes: "
              << UseModeToString(static_cast<Navico::Protocol::NRP::eUseMode>(
                     pSetup->useMode))
              << std::endl;
    std::cout << "AutoTune: " << (pSetup->tuneType != 0u ? "true" : "false")
              << std::endl;

    std::cout << "\n # Guard Zone updating #.....................\n";
    std::cout << "GuardSensitivity: "
              << static_cast<unsigned>(pSetup->guardzones.sensitivity)
              << std::endl;

    bool gz1Enabled =
        pSetup->guardzones
            .active[static_cast<unsigned>(GUARDZONE::eGuardZone1)];
    std::cout << "GuardZone1: " << (gz1Enabled == true ? "true" : "false")
              << std::endl;
    std::cout << "Guard1Range: "
              << pSetup->guardzones
                     .zone[static_cast<unsigned>(GUARDZONE::eGuardZone1)]
                     .rangeStart_m
              << " - "
              << pSetup->guardzones
                     .zone[static_cast<unsigned>(GUARDZONE::eGuardZone1)]
                     .rangeEnd_m
              << std::endl;
    std::cout << "Guard1Bearing: "
              << pSetup->guardzones
                         .zone[static_cast<unsigned>(GUARDZONE::eGuardZone1)]
                         .azimuth_ddeg /
                     10.0
              << std::endl;
    std::cout << "Guard1Width: "
              << pSetup->guardzones
                         .zone[static_cast<unsigned>(GUARDZONE::eGuardZone1)]
                         .width_ddeg /
                     10.0
              << std::endl;
    std::cout << "Guard1AlarmType: "
              << pSetup->guardzones
                     .alarmType[static_cast<unsigned>(GUARDZONE::eGuardZone1)]
                     .alarmType
              << std::endl;

    bool gz2Enabled =
        pSetup->guardzones
            .active[static_cast<unsigned>(GUARDZONE::eGuardZone2)];
    std::cout << "GuardZone2: " << (gz2Enabled == true ? "true" : "false")
              << std::endl;
    std::cout << "Guard2Range: "
              << pSetup->guardzones
                     .zone[static_cast<unsigned>(GUARDZONE::eGuardZone2)]
                     .rangeStart_m
              << " - "
              << pSetup->guardzones
                     .zone[static_cast<unsigned>(GUARDZONE::eGuardZone2)]
                     .rangeEnd_m
              << std::endl;
    std::cout << "Guard2Bearing: "
              << pSetup->guardzones
                         .zone[static_cast<unsigned>(GUARDZONE::eGuardZone2)]
                         .azimuth_ddeg /
                     10.0
              << std::endl;
    std::cout << "Guard2Width: "
              << pSetup->guardzones
                         .zone[static_cast<unsigned>(GUARDZONE::eGuardZone2)]
                         .width_ddeg /
                     10.0
              << std::endl;
    std::cout << "Guard2AlarmType: "
              << pSetup->guardzones
                     .alarmType[static_cast<unsigned>(GUARDZONE::eGuardZone2)]
                     .alarmType
              << std::endl;

  }  // UpdateSetup

  virtual void UpdateSetupExtended(
      const Navico::Protocol::NRP::tSetupExtended* pSetupExtended) {
    std::cout << "\n # Setup Extended updating #.....................\n";
    std::string output_str =
        pSetupExtended->suppressMainBang != 0 ? "true" : "false";
    std::cout << "MainBangSuppression: " << output_str << std::endl;

    // Side lobe controls update
    int gainType = pSetupExtended->sidelobe.type;
    assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
    std::cout << "Side Lobe: "
              << static_cast<unsigned>(pSetupExtended->sidelobe.value)
              << std::endl;
    std::cout << "LocalIR: " << static_cast<unsigned>(pSetupExtended->localIR)
              << std::endl;
    std::cout << "NoiseReject: "
              << static_cast<unsigned>(pSetupExtended->noiseReject)
              << std::endl;
    std::cout << "BeamSharpening: "
              << static_cast<unsigned>(pSetupExtended->beamSharpening)
              << std::endl;
    std::cout << "STCCurveType: "
              << static_cast<unsigned>(pSetupExtended->stcCurveType)
              << std::endl;
    std::cout << "FastScan: "
              << static_cast<unsigned>(pSetupExtended->fastScanMode)
              << std::endl;
    std::cout << "RPM: " << pSetupExtended->rpmX10 / 10.0 << std::endl;
    std::cout << "Sea: " << pSetupExtended->sea.manualValue << std::endl;
    std::cout << "SeaAuto: " << pSetupExtended->sea.autoOffset << std::endl;

  }  // UpdateSetupExtended

  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tProperties* pProperties) {
    std::cout << "\n # Properties updating #.....................\n";
    switch (pProperties->scannerType) {
      case Navico::Protocol::NRP::eTypeNoScanner:
        std::cout << "No Scanner\n";
        break;
      case Navico::Protocol::NRP::eNKE_1065:
        std::cout << "JRC 2KW Radome\n";
        break;
      case Navico::Protocol::NRP::eNKE_249:
        std::cout << "JRC 4KW Radome\n";
        break;
      case Navico::Protocol::NRP::eNKE_250_4:
        std::cout << "JRC 6KW 4ft OA\n";
        break;
      case Navico::Protocol::NRP::eNKE_250_4_NAX:
        std::cout << "JRC 6KW 4ft N OA\n";
        break;
      case Navico::Protocol::NRP::eNKE_2102_6:
        std::cout << "JRC 10KW 6ft OA\n";
        break;
      case Navico::Protocol::NRP::eNKE_2252_7:
        std::cout << "JRC 25KW 7ft OA\n";
        break;
      case Navico::Protocol::NRP::eNKE_2252_9:
        std::cout << "JRC 25KW 9ft OA\n";
        break;
      case Navico::Protocol::NRP::e4kWSimulator:
        std::cout << "4kW Simulator\n";
        break;
      case Navico::Protocol::NRP::eGWTestScanner:
        std::cout << "GWTest\n";
        break;
      case Navico::Protocol::NRP::eFMCW400_BR24:
        std::cout << "Navico BR-24\n";
        break;
      case Navico::Protocol::NRP::eFMCW400_Simulator:
        std::cout << "Navico BR-24 Sim\n";
        break;
      case Navico::Protocol::NRP::eFMCW400_HD3G:
        std::cout << "Navico BRHD-3G\n";
        break;
      case Navico::Protocol::NRP::eFMCW400_HD4G:
        std::cout << "Navico BRHD-4G\n";
        break;
      case Navico::Protocol::NRP::ePCOMP_HALO:
        std::cout << "Navico BRHALO\n";
        break;
      case Navico::Protocol::NRP::ePROP_MAGGIE:
        std::cout << "Navico-Pro Maggie\n";
        break;
      default:
        std::cout << "Type " << pProperties->scannerId << std::endl;
    }
  }  // UpdateProperties

  virtual void UpdateConfiguration(
      const Navico::Protocol::NRP::tConfiguration* pConfiguration) {
    std::cout << "\n # Configuration updating #.....................\n";
    std::cout << "Timed Transmit: " << pConfiguration->timedTransmitPeriod_s
              << std::endl;
    std::cout << "Timed Standby: " << pConfiguration->timedStandbyPeriod_s
              << std::endl;
    std::cout << "LEDs: " << static_cast<unsigned>(pConfiguration->ledLevel)
              << std::endl;

    std::cout << "ParkPosition: " << pConfiguration->parkPosition_ddeg / 10.0
              << std::endl;
    std::cout << "AntennaHeight: " << pConfiguration->antennaHeight_mm / 1000.0
              << std::endl;
    std::cout << "ZeroRange: " << pConfiguration->zeroRange_mm / 1000.0
              << std::endl;
    std::cout << "ZeroBearing: " << pConfiguration->zeroBearing_ddeg / 10.0
              << std::endl;
    std::cout << "AntennaXOffset: "
              << pConfiguration->antennaOffsetX_mm.Get() / 1000.0 << std::endl;
    std::cout << "AntennaYOffset: "
              << pConfiguration->antennaOffsetY_mm.Get() / 1000.0 << std::endl;

    int m_antennaTypeIndex = pConfiguration->antennaType;
    if (m_antennaTypeIndex >= 0) {
      std::cout << "AntennaType: " << m_antennaTypeIndex << std::endl;
    } else {
      std::cout << "AntennaType: unknown\n ";
    }

  }  // UpdateConfiguration

  virtual void UpdateGuardZoneAlarm(
      const Navico::Protocol::NRP::tGuardZoneAlarm* pAlarm) {
    unsigned zone = pAlarm->zone;
    if (zone < Navico::Protocol::NRP::cMaxGuardZones) {
      m_pGuardZoneAlarms[zone] = *pAlarm;
      m_AlarmTypes[zone] = m_pGuardZoneAlarms[zone].type;
      std::string gz_type = std::to_string(m_pGuardZoneAlarms[zone].type);
      std::string gz_state;
      switch (m_pGuardZoneAlarms[zone].state) {
        case Navico::Protocol::NRP::eAlarmActive: {
          gz_state = "Active";
        } break;

        case Navico::Protocol::NRP::eAlarmInactive: {
          gz_state = "Inactive";
        } break;

        case Navico::Protocol::NRP::eAlarmCancelled: {
          gz_state = "Cancelled";
        } break;

        default: {
          gz_state =
              "Unknown " + std::to_string(m_pGuardZoneAlarms[zone].state);
        } break;
      }

      if (zone == static_cast<unsigned>(GUARDZONE::eGuardZone1)) {
        std::cout << "Guard1Type: " << gz_type << std::endl;
        std::cout << "Guard1State: " << gz_state << std::endl;
      } else {
        std::cout << "Guard2Type: " << gz_type << std::endl;
        std::cout << "Guard2State: " << gz_state << std::endl;
      }
    }
  }  // UpdateGuardZoneAlarm

  virtual void UpdateRadarError(
      const Navico::Protocol::NRP::tRadarError* pError) {
    // NOTE: this simplification may overwrite a previous error that hasn't yet
    // been reported
    std::cout << "Scanner error: " +
                     ErrorToString(
                         Navico::Protocol::NRP::eRadarErrorType(pError->type));

  }  // UpdateRadarError

  virtual void UpdateAdvancedState(
      const Navico::Protocol::NRP::tAdvancedSTCState* pState) {
    std::cout << "\n # Advanced State updating #.....................\n";
    std::cout << "Range Trim: " << pState->rangeSTCTrim_dB << std::endl;
    std::cout << "Range Rate: " << pState->rangeSTCRate_dBpDec << std::endl;
    std::cout << "SeaTrim: " << pState->seaSTCTrim_dB << std::endl;
    std::cout << "SeaRate1: " << pState->seaSTCRate1_dBpDec << std::endl;
    std::cout << "SeaRate2: " << pState->seaSTCRate2_dBpDec << std::endl;
    std::cout << "RainTrim: " << pState->rainSTCTrim_dB << std::endl;
    std::cout << "RainRate: " << pState->rainSTCRate_dBpDec << std::endl;
    std::cout << "UserMinSNR: " << pState->userMinSNR_dB << std::endl;
    std::cout << "VideoAperture: " << pState->videoAperture_dB << std::endl;

  }  // UpdateAdvancedState

  // iFeatureObserver callbacks
  virtual void UpdateFeature(
      const Navico::Protocol::NRP::tFeatureEnum* /*pFeature*/) {
    // if (pFeature != nullptr) {
    //   Navico::Protocol::NRP::tFeatureManager& featureManager =
    //       m_pImageClient->GetFeatureManager();

    //   std::cout << "\n # Feature updating #.....................\n";
    //   switch (static_cast<Navico::Protocol::NRP::tFeatureEnum>(*pFeature)) {
    //     case Navico::Protocol::NRP::eFeatureEnum_SupportedUseModes: {
    //       UpdateUseMode();
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_IRControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureIR();
    //       std::cout << "IR Level Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_NoiseRejectControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureNoiseReject();
    //       ExtractFeatureControlEnum(feature);
    //       std::cout << "Noise Reject Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_STCCurveControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureStcCurves();
    //       ExtractFeatureControlEnum(feature, StcCurveStrings());
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_BeamSharpeningControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureBeamSharpening();
    //       std::cout << "BeamSharpening Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_FastScanControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureFastScan();
    //       std::cout << "Fast Scan:" << (feature.enabled == true ? "On" :
    //       "Off")
    //                 << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_SidelobeGainControl: {
    //       const Navico::Protocol::NRP::tFeatureRangeLimits& feature =
    //           featureManager.GetFeatureSidelobeGain();
    //       if (feature.supported == true) {
    //         std::cout << "UserControlSidelobeGain Range: " << feature.minimum
    //                   << "-" << feature.maximum << std::endl;
    //       }
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_LocalIRControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureLocalIR();
    //       std::cout << "LocalIR Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_LEDControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureLED();
    //       std::cout << "LEDs Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_TargetStretchControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureTargetStretch();
    //       std::cout << "TargetStretch Range: " << 0 << "-"
    //                 << static_cast<unsigned>(feature.maxLevel) << std::endl;
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_SeaUserGainLimits: {
    //       const Navico::Protocol::NRP::tFeatureGainLimits& feature =
    //           featureManager.GetFeatureSeaUserGainLimits();
    //       if (feature.supported == true) {
    //         std::cout << "SeaLimitsManual: " << feature.manualLevelMin << "-"
    //                   << feature.manualLevelMax << std::endl;
    //         std::cout << "SeaLimitsAuto: " << feature.autoOffsetMin << "-"
    //                   << feature.autoOffsetMax << std::endl;
    //       }
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_SupportedAntennas: {
    //       const Navico::Protocol::NRP::tFeatureAntennaTypes& feature =
    //           featureManager.GetFeatureSupportedAntennaTypes();
    //       std::cout << "Antenna: " << feature.count << std::endl;

    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_CombinedNoiseIFReject: {
    //       const Navico::Protocol::NRP::tFeatureBase& feature =
    //           featureManager.GetFeatureCombinedNoiseIFReject();
    //       if (feature.supported == true) {
    //         std::cout << "CombinedNoiseIR: " << feature.enabled;
    //       }
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_RangeStretchControl: {
    //       const Navico::Protocol::NRP::tFeatureLevel& feature =
    //           featureManager.GetFeatureRangeStretch();
    //       ExtractFeatureControlEnum(feature);
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_InstrRangeLimits: {
    //       const Navico::Protocol::NRP::tFeatureRangeLimits& feature =
    //           featureManager.GetFeatureInstrRangeLimits();
    //       if (feature.supported == true) {
    //         std::cout << "InstrRangeMin: " << feature.minimum << " dm\n";
    //         std::cout << "InstrRangeMax: " << feature.maximum << " dm\n";
    //       }
    //     } break;

    //     case Navico::Protocol::NRP::eFeatureEnum_SectorBlanking: {
    //       const Navico::Protocol::NRP::tFeatureSectorBlanking& feature =
    //           featureManager.GetFeatureSectorBlanking();
    //       if (feature.supported == true) {
    //         std::cout << "BlankSectors: " << feature.sectorCount
    //                   << " sectors\n";
    //       }
    //     } break;
    //     case Navico::Protocol::NRP::eFeatureEnum_PerformanceMonitor:
    //     default:
    //       break;
    //   }
    // }
  }  // UpdateFeature

  // iTargetTrackingClientObserver callbacks
  virtual void UpdateTarget(
      const Navico::Protocol::NRP::tTrackedTarget* /*pTarget*/) {
  }  // UpdateTarget

  // iTargetTrackingClientStateObserver callbacks
  virtual void UpdateNavigation(
      const Navico::Protocol::NRP::tNavigationState* /*pNavigationState*/) {
  }  // UpdateNavigation
  virtual void UpdateAlarmSetup(
      const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* /*pAlarmSetup*/) {
  }  // UpdateAlarmSetup
  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tTargetTrackingProperties* /*pProperties*/) {
  }  // UpdateProperties

  //-------------------------------------------------------------------------
  //  MutliRadar
  //-------------------------------------------------------------------------
  void MultiRadar_Connect(bool connect) {
    if (connect) {
      std::string serialNumber = m_pMultiRadar->GetRadarSerialNumber();
      unsigned instance = m_pMultiRadar->GetRadarInstance();

      Navico::Protocol::eErrors imageError =
          static_cast<Navico::Protocol::eErrors>(
              ConnectImageClient(serialNumber, instance));
      Navico::Protocol::eErrors targetError =
          static_cast<Navico::Protocol::eErrors>(
              ConnectTargetClient(serialNumber, instance));

      if (imageError != Navico::Protocol::EOK ||
          targetError != Navico::Protocol::EOK) {
        // something wrong!
        if (imageError == Navico::Protocol::EOK)
          DisconnectImageClient();  // imageclient error
        if (targetError == Navico::Protocol::EOK)
          DisconnectTargetClient();  // targetclient error
        if (imageError == Navico::Protocol::ELocked ||
            targetError == Navico::Protocol::ELocked) {
          // radar sdk unlock error
          std::cout << "Radar is locked!";
          m_pMultiRadar->InitiateUnlock();
        } else {
          std::string err;
          std::string msg = "Initialisation of ";
          if (imageError && targetError) {
            msg += "Image and Target-Tracking Clients";
            err = "errors " + ErrorToString(imageError) + " and " +
                  ErrorToString(targetError) + " resp";
          } else if (imageError) {
            msg += "Image Client";
            err = "error " + ErrorToString(imageError);
          } else if (targetError) {
            msg += "Target-Tracking Client";
            err = "error " + ErrorToString(targetError);
          }
          msg += " for Radar \"" + m_pMultiRadar->GetRadarSelection() +
                 "\" failed with " + err;

          std::cout << msg << std::endl;
        }
      } else {
        // there is no error

        // Image
        UpdateUseMode();

        //    m_pImageClient->SetToFactoryDefaults();
        bool set_results = m_pImageClient->SetUseMode(
            Navico::Protocol::NRP::eUseMode::eUseMode_Custom);

        // setup
        m_pImageClient->SetRange(400u);
        set_results = m_pImageClient->SetFastScanMode(
            uint8_t(0));  // 0: fast scan mode; otherwise, normal speed
        std::cout << "setresults: " << (set_results == true ? "true" : "false")
                  << std::endl;
        m_pImageClient->SetLEDsLevel(0);
        m_pImageClient->SetInterferenceReject(2);
        m_pImageClient->SetLocalIR(1);
        m_pImageClient->SetNoiseReject(3);
        m_pImageClient->SetBeamSharpening(3);
        m_pImageClient->SetTargetBoost(2);
        m_pImageClient->SetTargetStretch(true);
        m_pImageClient->SetSTCCurveType(
            Navico::Protocol::NRP::eStcCurveType::eCalm);
        m_pImageClient->SetGain(
            Navico::Protocol::NRP::eUserGainManualAuto::eUserGainAuto, 0);
        m_pImageClient->SetSeaClutter(
            Navico::Protocol::NRP::eUserGainManualAuto::eUserGainAuto, 0);
        m_pImageClient->SetSideLobe(
            Navico::Protocol::NRP::eUserGainManualAuto::eUserGainAuto, 0);
        m_pImageClient->SetRain(0);
        m_pImageClient->SetFTC(0);

        // pulse
        m_pImageClient->SetTuneState(m_pSetup->tuneType != 0u);
        m_pImageClient->SetTuneCoarse(0);
        m_pImageClient->SetTuneFine(0);

        // TimedTransmit
        m_pImageClient->SetTimedTransmit(false);
        m_pImageClient->SetTimedTransmitSetup(10, 10);

        // Installation
        m_pImageClient->SetParkPosition(0);
        m_pImageClient->SetAntennaHeight(3000);
        m_pImageClient->SetAntennaOffsets(0.0, 0.0);
        m_pImageClient->SetZeroRangeOffset(0.0);
        m_pImageClient->SetZeroBearingOffset(0);
        m_pImageClient->SetAntennaType(0);

        // advanced control
        /* STC Range */
        m_pImageClient->SetRangeSTCTrim(0.0);
        m_pImageClient->SetRangeSTCRate(40);

        /* STC Sea */
        m_pImageClient->SetSeaSTCTrim(0.0);
        m_pImageClient->SetSeaSTCRate1(40);
        m_pImageClient->SetSeaSTCRate2(40);

        /* STC Rain */
        m_pImageClient->SetRainSTCTrim(0.0);
        m_pImageClient->SetRainSTCRate(20);

        /* Miscellaneous */
        m_pImageClient->SetUserMinSNR(0.0);
        m_pImageClient->SetVideoAperture(4.0);
        m_pImageClient->SetMainBangSuppression(false);

        // Guard zone
        for (unsigned i = 0; i < Navico::Protocol::NRP::cMaxGuardZones; ++i) {
          m_AlarmTypes[i] = Navico::Protocol::NRP::eGZAlarmEntry;
        }
        m_pImageClient->SetGuardZoneSensitivity(200);

        bool gz1enables = true;
        uint32_t gz1_startRange_m = 40u;
        uint32_t gz1_endRange_m = 200u;
        uint16_t gz1_bearing_deg = 0u;
        uint16_t gz1_width_deg = 60u;
        m_pImageClient->SetGuardZoneEnable(
            static_cast<uint8_t>(GUARDZONE::eGuardZone1), gz1enables);
        // Set the guard zone in Radar
        m_pImageClient->SetGuardZoneSetup(
            static_cast<uint8_t>(GUARDZONE::eGuardZone1), gz1_startRange_m,
            gz1_endRange_m, gz1_bearing_deg, gz1_width_deg);
        m_pImageClient->SetGuardZoneAlarmSetup(
            static_cast<uint8_t>(GUARDZONE::eGuardZone1),
            Navico::Protocol::NRP::eGuardZoneAlarmType::eGZAlarmEntry);

        bool gz2enables = true;
        uint32_t gz2_startRange_m = 20u;
        uint32_t gz2_endRange_m = 100u;
        uint16_t gz2_bearing_deg = 180u;
        uint16_t gz2_width_deg = 40u;
        m_pImageClient->SetGuardZoneEnable(
            static_cast<uint8_t>(GUARDZONE::eGuardZone2), gz2enables);
        // Set the guard zone in Radar
        m_pImageClient->SetGuardZoneSetup(
            static_cast<uint8_t>(GUARDZONE::eGuardZone2), gz2_startRange_m,
            gz2_endRange_m, gz2_bearing_deg, gz2_width_deg);
        m_pImageClient->SetGuardZoneAlarmSetup(
            static_cast<uint8_t>(GUARDZONE::eGuardZone2),
            Navico::Protocol::NRP::eGuardZoneAlarmType::eGZAlarmEntry);

        // power on and start to transmit
        m_pImageClient->SetPower(true);
        m_pImageClient->SetTransmit(true);

        // client services connected ok - initialise all dependent user
        // interfaces
        // m_pTabTargets->OnConnect(m_pTargetClient);
        // m_pTabBScan->OnConnect();
        // m_pTabPPI->OnConnect();

        // Send the feature request message
        m_pImageClient->QueryAll();
      }
    } else {
      m_pImageClient->SetTransmit(false);
      m_pImageClient->SetPower(false);

      // now disconnect the client services
      DisconnectTargetClient();
      DisconnectImageClient();

      // first disconnect all dependent user interfaces
      // m_pTabTargets->OnDisconnect();
      // m_pTabBScan->OnDisconnect();
      // m_pTabPPI->OnDisconnect();

      // zero data for next time we connect
      InitProtocolData();
    }
  }

  //-------------------------------------------------------------------------
  //  Image Client
  //-------------------------------------------------------------------------
  int ConnectImageClient(const std::string& serialNumber, unsigned instance) {
    int error = Navico::Protocol::EOK + 1;
    if (m_pImageClient) {
      m_pImageClient->AddStateObserver(this);
      m_pImageClient->AddSpokeObserver(this);
      m_pImageClient->AddFeatureObserver(this);
      m_pImageClient->SetAutoSendClientWatchdog(true);

      error = m_pImageClient->Connect(serialNumber.c_str(), instance);
    }
    return error;
  }  // ConnectImageClient

  void DisconnectImageClient() {
    if (m_pImageClient) {
      m_pImageClient->Disconnect();
      m_pImageClient->RemoveStateObserver(this);
      m_pImageClient->RemoveSpokeObserver(this);
    }
  }  // DisconnectImageClient

  //-------------------------------------------------------------------------
  //  Target Tracking Client
  //-------------------------------------------------------------------------
  int ConnectTargetClient(const std::string& serialNumber, unsigned instance) {
    int error = Navico::Protocol::EOK + 1;
    if (m_pTargetClient) {
      m_pTargetClient->AddStateObserver(this);
      m_pTargetClient->AddTargetTrackingObserver(this);

      error = m_pTargetClient->Connect(serialNumber.c_str(), instance);

      // std::thread _socketthread(&GUIDemo::updateboatstate, this);
      // _socketthread.detach();
    }
    return error;
  }  // ConnectTargetClient

  void DisconnectTargetClient() {
    if (m_pTargetClient) {
      m_pTargetClient->RemoveStateObserver(this);
      m_pTargetClient->RemoveTargetTrackingObserver(this);
      m_pTargetClient->Disconnect();
    }
  }  // DisconnectTargetClient

  std::string ErrorToString(Navico::Protocol::NRP::eRadarErrorType type) {
    std::string estring;
    switch (type) {
      case Navico::Protocol::NRP::eErrorPersistenceCorrupt:
        estring = "Persistence Corrupt";
        break;
      case Navico::Protocol::NRP::eErrorZeroBearingFault:
        estring = "Zero Bearing Fault";
        break;
      case Navico::Protocol::NRP::eErrorBearingPulseFault:
        estring = "Bearing Pulse Fault";
        break;
      case Navico::Protocol::NRP::eErrorMotorNotRunning:
        estring = "Motor Not Running";
        break;
      case Navico::Protocol::NRP::eErrorCommsNotActive:
        estring = "CommsNotActive";
        break;
      case Navico::Protocol::NRP::eErrorMagnetronHeaterVoltage:
        estring = "Magnetron Heater Voltage";
        break;
      case Navico::Protocol::NRP::eErrorModulationVoltage:
        estring = "Modulation Voltage";
        break;
      case Navico::Protocol::NRP::eErrorTriggerFault:
        estring = "Trigger Fault";
        break;
      case Navico::Protocol::NRP::eErrorVideoFault:
        estring = "Video Fault";
        break;
      case Navico::Protocol::NRP::eErrorFanFault:
        estring = "Fan Fault";
        break;
      case Navico::Protocol::NRP::eErrorScannerConfigFault:
        estring = "Scanner Config Fault";
        break;
      case Navico::Protocol::NRP::eErrorPowerSupplyTransient:
        estring = "Power Supply Transient";
        break;
      case Navico::Protocol::NRP::eErrorScannerDetectFail:
        estring = "Scanner Detect Fail";
        break;
      case Navico::Protocol::NRP::eErrorPASoftOverheat:
        estring = "PA Soft-Overheat";
        break;
      case Navico::Protocol::NRP::eErrorPAHardOverheat:
        estring = "PA Hard-Overheat";
        break;
      case Navico::Protocol::NRP::eErrorGWDatapathError:
        estring = "GW Datapath Error";
        break;
      case Navico::Protocol::NRP::eErrorPSUOverheat:
        estring = "PSU Overheat";
        break;
      case Navico::Protocol::NRP::eErrorPSUVoltage:
        estring = "PSU Voltage";
        break;
      case Navico::Protocol::NRP::eErrorPSUPower:
        estring = "PSU Power";
        break;
      case Navico::Protocol::NRP::eErrorPSUHWFault:
        estring = "PSU HW-Fault";
        break;
      case Navico::Protocol::NRP::eErrorPSUCommsFault:
        estring = "PSU Comms-Fault";
        break;
      case Navico::Protocol::NRP::eErrorMechanicalFault:
        estring = "Mechanical Fault";
        break;
      case Navico::Protocol::NRP::eErrorLEDFault:
        estring = "LED Fault";
        break;
      case Navico::Protocol::NRP::eErrorScannerFail:
        estring = "Scanner Fail";
        break;
      case Navico::Protocol::NRP::eErrorRIFault:
        estring = "Radar-Interface Fault";
        break;
      case Navico::Protocol::NRP::eErrorLowBattery:
        estring = "Low Battery";
        break;
      case Navico::Protocol::NRP::eErrorMotorStall:
        estring = "Motor Stall";
        break;
      case Navico::Protocol::NRP::eErrorSafetyMode:
        estring = "Safety Mode";
        break;
      default:
        estring = "Radar Error" + std::to_string(type);
        break;
    }
    return estring;
  }

  std::string ErrorToString(Navico::Protocol::eErrors error) {
    std::string estring;
    switch (error) {
      case Navico::Protocol::EOK:
        estring = "EOK";
        break;
      case Navico::Protocol::ELocked:
        estring = "ELocked";
        break;
      case Navico::Protocol::EPending:
        estring = "EPending";
        break;
      case Navico::Protocol::ETimedOut:
        estring = "ETimedOut";
        break;
      case Navico::Protocol::EBusy:
        estring = "EBusy";
        break;
      case Navico::Protocol::EBadSerialNumber:
        estring = "EBadSerialNumber";
        break;
      case Navico::Protocol::ENoUnlockKey:
        estring = "ENoUnlockKey";
        break;
      case Navico::Protocol::EBadUnlockKey:
        estring = "EBadUnlockKey";
        break;
      case Navico::Protocol::EWrongUnlockKey:
        estring = "EWrongUnlockKey";
        break;
      case Navico::Protocol::ENotRunning:
        estring = "ENotRunning";
        break;
      case Navico::Protocol::EUnknownRadar:
        estring = "EUnknownRadar";
        break;
      case Navico::Protocol::ENonStdAddress:
        estring = "ENonStdAddress";
        break;
      case Navico::Protocol::ECommsFailure:
        estring = "ECommsFailure";
        break;
      case Navico::Protocol::EThreadCreation:
        estring = "EThreadCreation";
        break;
      case Navico::Protocol::EBadParameter:
        estring = "EBadParameter";
        break;
      case Navico::Protocol::EUnused:
        estring = "EUnused";
        break;
      case Navico::Protocol::EBadUnlockLevel:
        estring = "EBadUnlockLevel";
        break;
      default:
        estring = "error" + std::to_string(error);
        break;
    }
    return estring;
  }  // ErrorToString

  void UpdateUseMode() {
    // Check feature manager for supported use modes
    Navico::Protocol::NRP::tFeatureManager& featureManager =
        m_pImageClient->GetFeatureManager();
    const Navico::Protocol::NRP::tFeatureUseModes& feature =
        featureManager.GetFeatureSupportedUseModes();

    for (int i = 0; i < feature.useModeCount; ++i) {
      Navico::Protocol::NRP::eUseMode useMode = feature.useModes[i];
      if (useMode != Navico::Protocol::NRP::eUseMode_Custom) {
        std::cout << "Use Mode: " << UseModeToString(useMode) << std::endl;
      } else
        std::cout << "Use Mode: "
                  << UseModeToString(Navico::Protocol::NRP::eUseMode_Custom)
                  << std::endl;
    }

    if (feature.supported == true) {
      std::string useModes("");
      if (feature.useModeCount > 0u) {
        useModes = UseModeToString(feature.useModes[0]);
        for (int i = 1; i < feature.useModeCount; ++i) {
          useModes += "|";
          useModes += UseModeToString(feature.useModes[i]);
        }
      }
      std::cout << "UseModes: " << useModes << std::endl;
    }

  }  // UpdateUseMode

  //-----------------------------------------------------------------------------
  std::string UseModeToString(Navico::Protocol::NRP::eUseMode useMode) {
    std::string umstring;
    switch (useMode) {
      case Navico::Protocol::NRP::eUseMode_Custom:
        umstring = "Manual";
        break;
      case Navico::Protocol::NRP::eUseMode_Harbour:
        umstring = "Harbour";
        break;
      case Navico::Protocol::NRP::eUseMode_Offshore:
        umstring = "Offshore";
        break;
      case Navico::Protocol::NRP::eUseMode_Buoy:
        umstring = "Buoy";
        break;
      case Navico::Protocol::NRP::eUseMode_Weather:
        umstring = "Weather";
        break;
      case Navico::Protocol::NRP::eUseMode_Bird:
        umstring = "Bird";
        break;
      case Navico::Protocol::NRP::eUseMode_Netfinder:
        umstring = "Netfinder";
        break;
      case Navico::Protocol::NRP::eUseMode_SaRT:
        umstring = "SaRT";
        break;
      case Navico::Protocol::NRP::eUseMode_Doppler:
        umstring = "Doppler";
        break;
      case Navico::Protocol::NRP::eUseMode_RTE:
        umstring = "RTE";
        break;
      default:
        umstring = "Unknown " + std::to_string(useMode);
        break;
    }
    return umstring;
  }  // UseModeToString

  //-----------------------------------------------------------------------------
  void InitProtocolData() {
    // setup image protocol data structures
    InitStruct(m_pMode);
    InitStruct(m_pSetup);
    InitStruct(m_pSetupExtended);
    InitStruct(m_pProperties);
    InitStruct(m_pConfiguration);
    InitStruct(m_pAdvancedSTCState);
    InitStruct(m_pRadarError);
    InitStruct(m_pGuardZoneAlarms, Navico::Protocol::NRP::cMaxGuardZones);

    // setup trget-tracking protocol data structures
    InitStruct(m_pTargetAlarmSetup);
    InitStruct(m_pTargetProperties);
    InitStruct(m_pTargets, 1 + cMaxTargets);
  }  // InitProtocolData

  template <typename Type>
  inline void InitStruct(Type*& ptr, unsigned count = 1) {
    if (ptr == nullptr) {
      if (count > 1)
        ptr = new Type[count];
      else
        ptr = new Type;
    }

    if (ptr != nullptr) {
      if (count > 1)
        memset(ptr, 0, count * sizeof(Type));
      else
        memset(ptr, 0, sizeof(Type));
    }
  }  // InitStruct

 private:
  const unsigned cMaxTargets = 10;
  // image client protocol manager
  Navico::Protocol::NRP::tImageClient* m_pImageClient;

  // cached data from the radar
  Navico::Protocol::NRP::tMode* m_pMode;
  Navico::Protocol::NRP::tSetup* m_pSetup;
  Navico::Protocol::NRP::tSetupExtended* m_pSetupExtended;
  Navico::Protocol::NRP::tProperties* m_pProperties;
  Navico::Protocol::NRP::tConfiguration* m_pConfiguration;
  Navico::Protocol::NRP::tAdvancedSTCState* m_pAdvancedSTCState;
  Navico::Protocol::NRP::tGuardZoneAlarm* m_pGuardZoneAlarms;
  Navico::Protocol::NRP::tRadarError* m_pRadarError;
  unsigned m_PixelCellSize_mm;
  Navico::Protocol::NRP::tNavigationState* m_pNavigationState;
  Navico::Protocol::NRP::tTargetTrackingClient* m_pTargetClient;
  Navico::Protocol::NRP::tTargetTrackingAlarmSetup* m_pTargetAlarmSetup;
  Navico::Protocol::NRP::tTargetTrackingProperties* m_pTargetProperties;
  Navico::Protocol::NRP::tTrackedTarget* m_pTargets;

  MultiRadar* m_pMultiRadar;
  Navico::Protocol::NRP::Spoke::t9174Spoke m_pSpoke;
  MarineRadarRTdata MarineRadar_RTdata;
  bool m_AlarmTypes[Navico::Protocol::NRP::cMaxGuardZones];
};

}  // namespace ASV::messages

#endif /* _MARINERADAR_H_ */