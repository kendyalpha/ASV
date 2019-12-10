//-----------------------------------------------------------------------------
// Copyright (C) 2007-2016 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "GUIDemo.h"
//-----------------------------------------------------------------------------
// GUIDemo Class Implementation
//-----------------------------------------------------------------------------
GUIDemo::GUIDemo(QWidget* pParent, Qt::WindowFlags flags)
    : QMainWindow(pParent, flags),
      m_pImageClient(nullptr),
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
      m_pTargetLocations(nullptr) {
  m_pImageClient = new Navico::Protocol::NRP::tImageClient();
  m_pTargetClient = new Navico::Protocol::NRP::tTargetTrackingClient();
  InitProtocolData();

  // setup UI
  ui.setupUi(this);
  m_pMultiRadar = new tMultiRadar(ui, this);
  m_pTabGuardZone =
      new tTabGuardZone(ui, this, *ui.tabGuardZone, m_OverlayManager);
  m_pTabTargets = new tTabTargets(ui, this, *ui.tabTargets);
  m_pTabBScan = new tTabBScan(ui, m_pTargetLocations, cMaxTargets, this,
                              m_OverlayManager);
  m_pTabPPI =
      new tTabPPI(ui, m_pTargetLocations, cMaxTargets, this, m_OverlayManager);

  MakeConnections(true);
}

//-----------------------------------------------------------------------------
GUIDemo::~GUIDemo() {
  MakeConnections(false);

  DisconnectTargetClient();
  DisconnectImageClient();

  delete m_pTabPPI;
  delete m_pTabBScan;
  delete m_pTabTargets;
  delete m_pTabGuardZone;
  delete m_pMultiRadar;

  // cleanup target-tracking
  delete m_pTargetClient;
  delete m_pTargetAlarmSetup;
  delete m_pTargetProperties;
  delete[] m_pTargets;
  delete[] m_pTargetLocations;

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

//-----------------------------------------------------------------------------
void GUIDemo::MakeConnections(bool connect) {
  // connections for user requested connect/disconnect
  Connect(connect, m_pMultiRadar, SIGNAL(ConnectChanged(bool)), this,
          SLOT(MultiRadar_ConnectChanged(bool)));

  // connections for updated state data
  Connect(connect, this, SIGNAL(UpdateGuardZoneAlarm_signal(unsigned)), this,
          SLOT(UpdateGuardZoneAlarm_slot(unsigned)), Qt::QueuedConnection);

  // connections for updated target data
  Connect(connect, this, SIGNAL(UpdateTargetAlarmSetup_signal()), this,
          SLOT(UpdateTargetAlarmSetup_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateTargetTarget_signal(unsigned)), this,
          SLOT(UpdateTargetTarget_slot(unsigned)), Qt::QueuedConnection);

  // connections for target acquire
  Connect(connect, m_pTabBScan, SIGNAL(AcquireTarget(double, double)), this,
          SLOT(TargetAcquire(double, double)));
  Connect(connect, m_pTabPPI, SIGNAL(AcquireTarget(double, double)), this,
          SLOT(TargetAcquire(double, double)));
}

//-----------------------------------------------------------------------------
void GUIDemo::MultiRadar_ConnectChanged(bool connect) {
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
      if (imageError == Navico::Protocol::EOK) DisconnectImageClient();
      if (targetError == Navico::Protocol::EOK) DisconnectTargetClient();

      if (imageError == Navico::Protocol::ELocked ||
          targetError == Navico::Protocol::ELocked) {
        m_pMultiRadar->SetConnectState(false);

        if (QMessageBox::warning(
                this, "Unlock Radar?",
                "Radar is locked - would you like to unlock the radar?",
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::Yes) == QMessageBox::Yes) {
          m_pMultiRadar->InitiateUnlock();
        }
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

        // TODO
        /*std::cout << msg << std::endl; */

        m_pMultiRadar->SetConnectState(false);
      }
    } else {
      if (m_pImageClient) {
        m_pImageClient->SetPower(true);
        m_pImageClient->SetTransmit(true);
      }

      m_OverlayManager.Clear();

      // client services connected ok - initialise all dependent user interfaces
      m_pTabGuardZone->OnConnect(m_pImageClient);
      m_pTabTargets->OnConnect(m_pTargetClient);
      m_pTabBScan->OnConnect();
      m_pTabPPI->OnConnect();

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
    m_pTabGuardZone->OnDisconnect();
    m_pTabTargets->OnDisconnect();
    m_pTabBScan->OnDisconnect();
    m_pTabPPI->OnDisconnect();

    // zero data for next time we connect
    InitProtocolData();
  }
}

//-----------------------------------------------------------------------------
//  Image Handling
//-----------------------------------------------------------------------------
int GUIDemo::ConnectImageClient(const std::string& serialNumber,
                                unsigned instance) {
  int error = Navico::Protocol::EOK + 1;
  bool results = false;
  if (m_pImageClient) {
    m_pImageClient->AddStateObserver(this);
    m_pImageClient->AddSpokeObserver(this);
    m_pImageClient->AddFeatureObserver(this);
    m_pImageClient->SetAutoSendClientWatchdog(true);

    m_ForceSetup = true;
    m_ForceAdvancedSTCState = true;
    error = m_pImageClient->Connect(serialNumber.c_str(), instance);

    // Image
    UpdateUseMode();

    //    m_pImageClient->SetToFactoryDefaults();
    m_pImageClient->SetUseMode(
        Navico::Protocol::NRP::eUseMode::eUseMode_Custom);


    // setup
    m_pImageClient->SetRange(400);

//    results = m_pImageClient->SetScannerRPM(240u);
    results = m_pImageClient->SetFastScanMode(
        uint8_t(0));  // 0: fast scan mode; otherwise, normal speed
    m_pImageClient->SetLEDsLevel(0);
    m_pImageClient->SetInterferenceReject(2);
    m_pImageClient->SetLocalIR(1);
    m_pImageClient->SetNoiseReject(2);
    m_pImageClient->SetBeamSharpening(3);
    m_pImageClient->SetTargetBoost(1);
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
    m_pImageClient->SetAntennaHeight(4);
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
  }
  return error;
}

//-----------------------------------------------------------------------------
void GUIDemo::DisconnectImageClient() {
  if (m_pImageClient) {
    m_pImageClient->Disconnect();
    m_pImageClient->RemoveStateObserver(this);
    m_pImageClient->RemoveSpokeObserver(this);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateMode(const Navico::Protocol::NRP::tMode* pMode) {
  if (memcmp(pMode, m_pMode, sizeof(Navico::Protocol::NRP::tMode)) != 0) {
    *m_pMode = *pMode;

    /*switch (m_pMode->state) {
      case Navico::Protocol::NRP::eOff:
        std::cout << "state: off\n";
        ui.checkScannerPower->setChecked(false);
        ui.checkScannerTransmit->setChecked(false);
        break;
      case Navico::Protocol::NRP::eStandby:
        std::cout << "state: standby\n";
        ui.checkScannerPower->setChecked(true);
        ui.checkScannerTransmit->setChecked(false);
        break;
      case Navico::Protocol::NRP::eTransmit:
        std::cout << "state: transmit\n";
        ui.checkScannerPower->setChecked(true);
        ui.checkScannerTransmit->setChecked(true);
        break;
      case Navico::Protocol::NRP::eWarming:
        std::cout << "state: Warming up\n";
        ui.checkScannerPower->setChecked(true);
        ui.checkScannerTransmit->setChecked(false);
        break;
      case Navico::Protocol::NRP::eNoScanner:
        std::cout << "state: no scanner\n";
        ui.checkScannerPower->setChecked(false);
        ui.checkScannerTransmit->setChecked(false);
        break;
      case Navico::Protocol::NRP::eDetectingScanner:
        std::cout << "state: Detecting Scanner\n";
        ui.checkScannerPower->setChecked(true);
        ui.checkScannerTransmit->setChecked(false);
        break;
      default:
        std::cout << "state: Undefined\n";
        break;
    }

    if (m_pMode->warmupTime_sec == 0) {
      std::cout << "Scanner Timeout = " << m_pMode->ttCount_sec << std::endl;
    } else {
      std::cout << "Scanner Timeout = " << m_pMode->warmupTime_sec << std::endl;
    }

    std::cout << "checkTimedTxMode: " << m_pMode->ttState << std::endl;*/
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup) {
  if (m_ForceSetup ||
      memcmp(pSetup, m_pSetup, sizeof(Navico::Protocol::NRP::tSetup)) != 0) {
    m_ForceSetup = false;
    *m_pSetup = *pSetup;

    std::cout << "Scanner Range: " << m_pSetup->range_dm / 10.0 << " m\n";

    /* // Gain controls update
     uint32_t gainType = m_pSetup->gain[Navico::Protocol::NRP::eSetupGain].type;
     assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
     std::cout << "Radar Gain: "
               << std::to_string(
                      m_pSetup->gain[Navico::Protocol::NRP::eSetupGain].value)
               << std::endl;

     // Sea clutter controls update
     gainType = m_pSetup->gain[Navico::Protocol::NRP::eSetupSea].type;
     assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
     std::cout << "SeaClutter: "
               << std::to_string(
                      m_pSetup->gain[Navico::Protocol::NRP::eSetupSea].value)
               << std::endl;

     // Rain controls update
     gainType = m_pSetup->gain[Navico::Protocol::NRP::eSetupRain].type;
     assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
     std::cout << "Rain: "
               << std::to_string(
                      m_pSetup->gain[Navico::Protocol::NRP::eSetupRain].value)
               << std::endl;

     // FTC controls update
     std::cout << "FTC: " << std::to_string(m_pSetup->ftc.value) << std::endl;

     // Interference Reject controls update
     std::cout << "IRLevel: " << std::to_string(m_pSetup->interferenceReject)
               << std::endl;

     // Target Boost controls update
     std::cout << "TargetBoost: " << std::to_string(m_pSetup->targetBoost)
               << std::endl;

     // Target Stretch controls update
     std::cout << "TargetStretch: " << std::to_string(m_pSetup->targetStretch)
               << std::endl;

     // Pulse Group
     std::cout << "TargetStretch: " << std::to_string(m_pSetup->targetStretch)
               << std::endl;
     std::cout << "PulseWidth: " << std::to_string(m_pSetup->pwLength_ns)
               << std::endl;
     std::cout << "CoarseTune: " << std::to_string(m_pSetup->coarseTune)
               << std::endl;
     std::cout << "FineTune: " << std::to_string(m_pSetup->fineTune)
               << std::endl;
     std::cout << "UseModes: "
               << UseModeToString(static_cast<Navico::Protocol::NRP::eUseMode>(
                      m_pSetup->useMode))
               << std::endl;
     std::cout << "AutoTune: " << (m_pSetup->tuneType != 0u ? "true" : "false")
               << std::endl;
 */
    m_pTabGuardZone->OnSetupChanged(m_pSetup);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetupExtended(
    const Navico::Protocol::NRP::tSetupExtended* pSetupExtended) {
  if (memcmp(pSetupExtended, m_pSetupExtended,
             sizeof(Navico::Protocol::NRP::tSetupExtended)) != 0) {
    *m_pSetupExtended = *pSetupExtended;
    std::string output_str =
        pSetupExtended->suppressMainBang != 0 ? "true" : "false";
    std::cout << "MainBangSuppression: " << output_str << std::endl;

    // Side lobe controls update
    int gainType = m_pSetupExtended->sidelobe.type;
    assert(gainType < Navico::Protocol::NRP::eTotalUserGains);
    std::cout << "Side Lobe: " << m_pSetupExtended->sidelobe.value << std::endl;
    std::cout << "LocalIR: " << m_pSetupExtended->localIR << std::endl;
    std::cout << "NoiseReject: " << m_pSetupExtended->noiseReject << std::endl;
    std::cout << "BeamSharpening: " << m_pSetupExtended->beamSharpening
              << std::endl;
    std::cout << "STCCurveType: " << m_pSetupExtended->stcCurveType
              << std::endl;
    std::cout << "FastScan: " << m_pSetupExtended->fastScanMode << std::endl;
    std::cout << "RPM: " << m_pSetupExtended->rpmX10 / 10.0 << std::endl;
    std::cout << "Sea: " << m_pSetupExtended->sea.manualValue << std::endl;
    std::cout << "SeaAuto: " << m_pSetupExtended->sea.autoOffset << std::endl;
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties(
    const Navico::Protocol::NRP::tProperties* pProperties) {
  if (memcmp(pProperties, m_pProperties,
             sizeof(Navico::Protocol::NRP::tProperties)) != 0) {
    *m_pProperties = *pProperties;
    /*switch (pProperties->scannerType) {
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
    }*/
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateConfiguration(
    const Navico::Protocol::NRP::tConfiguration* pConfiguration) {
  if (memcmp(pConfiguration, m_pConfiguration,
             sizeof(Navico::Protocol::NRP::tConfiguration)) != 0) {
    *m_pConfiguration = *pConfiguration;
    /*std::cout << "Timed Transmit: " << m_pConfiguration->timedTransmitPeriod_s
              << std::endl;
    std::cout << "Timed Standby: " << m_pConfiguration->timedStandbyPeriod_s
              << std::endl;
    std::cout << "LEDs: " << m_pConfiguration->ledLevel << std::endl;

    std::cout << "ParkPosition: " << m_pConfiguration->parkPosition_ddeg / 10.0
              << std::endl;
    std::cout << "AntennaHeight: "
              << m_pConfiguration->antennaHeight_mm / 1000.0 << std::endl;
    std::cout << "ZeroRange: " << m_pConfiguration->zeroRange_mm / 1000.0
              << std::endl;
    std::cout << "ZeroBearing: " << m_pConfiguration->zeroBearing_ddeg / 10.0
              << std::endl;
    std::cout << "AntennaXOffset: "
              << m_pConfiguration->antennaOffsetX_mm.Get() / 1000.0
              << std::endl;
    std::cout << "AntennaYOffset: "
              << m_pConfiguration->antennaOffsetY_mm.Get() / 1000.0
              << std::endl;

    int m_antennaTypeIndex = m_pConfiguration->antennaType;
    if (m_antennaTypeIndex >= 0) {
      std::cout << "AntennaType: " << m_antennaTypeIndex << std::endl;
    } else {
      std::cout << "AntennaType: unknown\n ";
    }
    */
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSpoke(
    const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke) {
  m_PixelCellSize_mm =
      Navico::Protocol::NRP::Spoke::GetPixelCellSize_mm(pSpoke->header);
  m_pTabBScan->OnUpdateSpoke(pSpoke);
  m_pTabPPI->OnUpdateSpoke(pSpoke);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateAdvancedState(
    const Navico::Protocol::NRP::tAdvancedSTCState* pState) {
  if (m_ForceAdvancedSTCState ||
      memcmp(pState, m_pAdvancedSTCState,
             sizeof(Navico::Protocol::NRP::tAdvancedSTCState)) != 0) {
    m_ForceAdvancedSTCState = false;
    *m_pAdvancedSTCState = *pState;

    /*std::cout << "Range Trim: " << pState->rangeSTCTrim_dB << std::endl;
    std::cout << "Range Rate: " << pState->rangeSTCRate_dBpDec << std::endl;
    std::cout << "SeaTrim: " << pState->seaSTCTrim_dB << std::endl;
    std::cout << "SeaRate1: " << pState->seaSTCRate1_dBpDec << std::endl;
    std::cout << "SeaRate2: " << pState->seaSTCRate2_dBpDec << std::endl;
    std::cout << "RainTrim: " << pState->rainSTCTrim_dB << std::endl;
    std::cout << "RainRate: " << pState->rainSTCRate_dBpDec << std::endl;
    std::cout << "UserMinSNR: " << pState->userMinSNR_dB << std::endl;
    std::cout << "VideoAperture: " << pState->videoAperture_dB << std::endl;*/
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateFeature(
    const Navico::Protocol::NRP::tFeatureEnum* pFeature) {
  if (pFeature != nullptr) {
    Navico::Protocol::NRP::tFeatureManager& featureManager =
        m_pImageClient->GetFeatureManager();

    /* switch (static_cast<Navico::Protocol::NRP::tFeatureEnum>(*pFeature)) {
       case Navico::Protocol::NRP::eFeatureEnum_SupportedUseModes: {
         UpdateUseMode();
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_IRControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureIR();
         std::cout << "IR Level Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_NoiseRejectControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureNoiseReject();
         ExtractFeatureControlEnum(feature);
         std::cout << "Noise Reject Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_STCCurveControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureStcCurves();
         ExtractFeatureControlEnum(feature, StcCurveStrings());
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_BeamSharpeningControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureBeamSharpening();
         std::cout << "BeamSharpening Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_FastScanControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureFastScan();
         std::cout << "Fast Scan:" << (feature.enabled == true ? "On" : "Off")
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_SidelobeGainControl: {
         const Navico::Protocol::NRP::tFeatureRangeLimits& feature =
             featureManager.GetFeatureSidelobeGain();
         if (feature.supported == true) {
           std::cout << "UserControlSidelobeGain Range: " << feature.minimum
                     << "-" << feature.maximum << std::endl;
         }
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_LocalIRControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureLocalIR();
         std::cout << "LocalIR Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_LEDControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureLED();
         std::cout << "LEDs Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_TargetStretchControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureTargetStretch();
         std::cout << "TargetStretch Range: " << 0 << "-" << feature.maxLevel
                   << std::endl;
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_SeaUserGainLimits: {
         const Navico::Protocol::NRP::tFeatureGainLimits& feature =
             featureManager.GetFeatureSeaUserGainLimits();
         if (feature.supported == true) {
           std::cout << "SeaLimitsManual: " << feature.manualLevelMin << "-"
                     << feature.manualLevelMax << std::endl;
           std::cout << "SeaLimitsAuto: " << feature.autoOffsetMin << "-"
                     << feature.autoOffsetMax << std::endl;
         }
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_SupportedAntennas: {
         const Navico::Protocol::NRP::tFeatureAntennaTypes& feature =
             featureManager.GetFeatureSupportedAntennaTypes();
         std::cout << "Antenna: " << feature.count << std::endl;

       } break;

       case Navico::Protocol::NRP::eFeatureEnum_CombinedNoiseIFReject: {
         const Navico::Protocol::NRP::tFeatureBase& feature =
             featureManager.GetFeatureCombinedNoiseIFReject();
         if (feature.supported == true) {
           std::cout << "CombinedNoiseIR: " << feature.enabled;
         }
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_RangeStretchControl: {
         const Navico::Protocol::NRP::tFeatureLevel& feature =
             featureManager.GetFeatureRangeStretch();
         ExtractFeatureControlEnum(feature);
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_InstrRangeLimits: {
         const Navico::Protocol::NRP::tFeatureRangeLimits& feature =
             featureManager.GetFeatureInstrRangeLimits();
         if (feature.supported == true) {
           std::cout << "InstrRangeMin: " << feature.minimum << " dm\n";
           std::cout << "InstrRangeMax: " << feature.maximum << " dm\n";
         }
       } break;

       case Navico::Protocol::NRP::eFeatureEnum_SectorBlanking: {
         const Navico::Protocol::NRP::tFeatureSectorBlanking& feature =
             featureManager.GetFeatureSectorBlanking();
         if (feature.supported == true) {
           std::cout << "BlankSectors: " << feature.sectorCount << " sectors\n";
         }
       } break;
       case Navico::Protocol::NRP::eFeatureEnum_PerformanceMonitor:
       default:
         break;
     }*/
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateGuardZoneAlarm(
    const Navico::Protocol::NRP::tGuardZoneAlarm* pAlarm) {
  unsigned zone = pAlarm->zone;
  if (zone < Navico::Protocol::NRP::cMaxGuardZones) {
    m_pGuardZoneAlarms[zone] = *pAlarm;
    emit UpdateGuardZoneAlarm_signal(zone);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateRadarError(
    const Navico::Protocol::NRP::tRadarError* pError) {
  // NOTE: this simplification may overwrite a previous error that hasn't yet
  // been reported
  *m_pRadarError = *pError;
  /* std::cout << "Scanner error: " +
                    ToString(Navico::Protocol::NRP::eRadarErrorType(
                        m_pRadarError->type)); */
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateGuardZoneAlarm_slot(unsigned zone) {
  m_pTabGuardZone->OnGuardZoneAlarmChanged(zone, &m_pGuardZoneAlarms[zone]);
}

//-----------------------------------------------------------------------------
//  Target Tracking Handling
//-----------------------------------------------------------------------------
int GUIDemo::ConnectTargetClient(const std::string& serialNumber,
                                 unsigned instance) {
  int error = Navico::Protocol::EOK + 1;
  if (m_pTargetClient) {
    m_pTargetClient->AddStateObserver(this);
    m_pTargetClient->AddTargetTrackingObserver(this);

    m_ForceTargetAlarmSetup = true;
    m_ForceTargetProperties = true;

    error = m_pTargetClient->Connect(serialNumber.c_str(), instance);
  }
  return error;
}

//-----------------------------------------------------------------------------
void GUIDemo::DisconnectTargetClient() {
  if (m_pTargetClient) {
    m_pTargetClient->RemoveStateObserver(this);
    m_pTargetClient->RemoveTargetTrackingObserver(this);
    m_pTargetClient->Disconnect();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::TargetAcquire(double sample, double degrees) {
  if (m_PixelCellSize_mm > 0) {
    unsigned range = (sample * m_PixelCellSize_mm) / 1000 + 0.5;
    m_pTargetClient->Acquire(0, range, int32_t(degrees + 0.5),
                             Navico::Protocol::NRP::eRelative);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateAlarmSetup(
    const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* pAlarmSetup) {
  if (m_ForceTargetAlarmSetup ||
      memcmp(pAlarmSetup, m_pTargetAlarmSetup,
             sizeof(Navico::Protocol::NRP::tTargetTrackingAlarmSetup)) != 0) {
    m_ForceTargetAlarmSetup = false;
    *m_pTargetAlarmSetup = *pAlarmSetup;
    emit UpdateTargetAlarmSetup_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties(
    const Navico::Protocol::NRP::tTargetTrackingProperties* pProperties) {
  if (m_ForceTargetProperties ||
      memcmp(pProperties, m_pTargetProperties,
             sizeof(Navico::Protocol::NRP::tTargetTrackingProperties)) != 0) {
    m_ForceTargetProperties = false;
    *m_pTargetProperties = *pProperties;
    m_pTabTargets->OnTTPropertiesChanged(m_pTargetProperties);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateNavigation(
    const Navico::Protocol::NRP::tNavigationState* pNavigationState) {
  if (m_ForceNavigationState ||
      memcmp(pNavigationState, m_pNavigationState,
             sizeof(Navico::Protocol::NRP::tNavigationState)) != 0) {
    m_ForceNavigationState = false;
    *m_pNavigationState = *pNavigationState;
    // TODO::
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTarget(
    const Navico::Protocol::NRP::tTrackedTarget* pTarget) {
  unsigned index = static_cast<unsigned>(std::max(0, pTarget->serverTargetID));

  if (index > cMaxTargets) index = 0;

  m_pTargets[index] = *pTarget;
  if (index > 0 && m_PixelCellSize_mm > 0) {
    tTargetLocation* pLoc = &m_pTargetLocations[index - 1];
    uint32_t state = pTarget->targetState;

    pLoc->isValid = false;
    if (pTarget->targetValid && state <= 3) {
      pLoc->sample =
          (pTarget->infoRelative.distance_m * 1000.0 / m_PixelCellSize_mm);
      pLoc->degrees = pTarget->infoRelative.bearing_ddeg / 10.0;
      pLoc->isValid = true;
    }
  }
  emit UpdateTargetTarget_signal(index);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTargetAlarmSetup_slot() {
  m_pTabTargets->OnTTAlarmSetupChanged(m_pTargetAlarmSetup);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTargetTarget_slot(unsigned target) {
  m_pTabTargets->OnTrackedTargetChanged(target, &m_pTargets[target]);
}

////-----------------------------------------------------------------------------
// void GUIDemo::ScannerPower_clicked(bool checked) {
//  if (m_pImageClient) {
//    m_pImageClient->SetPower(checked);
//    ui.checkScannerPower->setChecked(false);
//  }
//}

////-----------------------------------------------------------------------------
// void GUIDemo::ScannerTransmit_clicked(bool checked) {
//  if (m_pImageClient) {
//    m_pImageClient->SetTransmit(checked);
//    ui.checkScannerTransmit->setChecked(false);
//  }
//}
