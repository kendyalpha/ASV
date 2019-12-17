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
      // something wrong!
      if (imageError == Navico::Protocol::EOK)
        DisconnectImageClient();  // imageclient error
      if (targetError == Navico::Protocol::EOK)
        DisconnectTargetClient();  // targetclient error
      if (imageError == Navico::Protocol::ELocked ||
          targetError == Navico::Protocol::ELocked) {
        // radar sdk unlock error
        m_pMultiRadar->SetConnectState(false);
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
        m_pMultiRadar->SetConnectState(false);
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
      m_OverlayManager.Clear();
      for (unsigned i = 0; i < Navico::Protocol::NRP::cMaxGuardZones; ++i) {
        m_AlarmTypes[i] = Navico::Protocol::NRP::eGZAlarmEntry;
      }
      m_pImageClient->SetGuardZoneSensitivity(200);

      bool gz1enables = true;
      uint32_t gz1_startRange_m = 40u;
      uint32_t gz1_endRange_m = 200u;
      uint16_t gz1_bearing_deg = 0;
      uint16_t gz1_width_deg = 60;
      m_pImageClient->SetGuardZoneEnable(eGuardZone1, gz1enables);
      // Set the guard zone in Radar
      m_pImageClient->SetGuardZoneSetup(eGuardZone1, gz1_startRange_m,
                                        gz1_endRange_m, gz1_bearing_deg,
                                        gz1_width_deg);
      m_pImageClient->SetGuardZoneAlarmSetup(
          eGuardZone1,
          Navico::Protocol::NRP::eGuardZoneAlarmType::eGZAlarmEntry);
      // Update the guard zone overlay in GUI
      m_OverlayManager.SetGuardZone(eGuardZone1, gz1enables, gz1_startRange_m,
                                    gz1_endRange_m, gz1_bearing_deg,
                                    gz1_width_deg);

      bool gz2enables = true;
      uint32_t gz2_startRange_m = 20u;
      uint32_t gz2_endRange_m = 100u;
      uint16_t gz2_bearing_deg = 180u;
      uint16_t gz2_width_deg = 40u;
      m_pImageClient->SetGuardZoneEnable(eGuardZone2, gz2enables);
      // Set the guard zone in Radar
      m_pImageClient->SetGuardZoneSetup(eGuardZone2, gz2_startRange_m,
                                        gz2_endRange_m, gz2_bearing_deg,
                                        gz2_width_deg);
      m_pImageClient->SetGuardZoneAlarmSetup(
          eGuardZone2,
          Navico::Protocol::NRP::eGuardZoneAlarmType::eGZAlarmEntry);
      // Update the guard zone overlay in GUI
      m_OverlayManager.SetGuardZone(eGuardZone2, gz2enables, gz2_startRange_m,
                                    gz2_endRange_m, gz2_bearing_deg,
                                    gz2_width_deg);

      // power on and start to transmit
      m_pImageClient->SetPower(true);
      m_pImageClient->SetTransmit(true);

      // client services connected ok - initialise all dependent user interfaces
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
  if (m_pImageClient) {
    m_pImageClient->AddStateObserver(this);
    m_pImageClient->AddSpokeObserver(this);
    m_pImageClient->AddFeatureObserver(this);
    m_pImageClient->SetAutoSendClientWatchdog(true);

    m_ForceSetup = true;
    m_ForceAdvancedSTCState = true;
    error = m_pImageClient->Connect(serialNumber.c_str(), instance);
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
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup) {
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

  bool gz1Enabled = pSetup->guardzones.active[eGuardZone1];
  std::cout << "GuardZone1: " << (gz1Enabled == true ? "true" : "false")
            << std::endl;
  std::cout << "Guard1Range: "
            << pSetup->guardzones.zone[eGuardZone1].rangeStart_m << " - "
            << pSetup->guardzones.zone[eGuardZone1].rangeEnd_m << std::endl;
  std::cout << "Guard1Bearing: "
            << pSetup->guardzones.zone[eGuardZone1].azimuth_ddeg / 10.0
            << std::endl;
  std::cout << "Guard1Width: "
            << pSetup->guardzones.zone[eGuardZone1].width_ddeg / 10.0
            << std::endl;
  std::cout << "Guard1AlarmType: "
            << pSetup->guardzones.alarmType[eGuardZone1].alarmType << std::endl;

  m_OverlayManager.SetGuardZone(
      eGuardZone1, gz1Enabled,
      pSetup->guardzones.zone[eGuardZone1].rangeStart_m,
      pSetup->guardzones.zone[eGuardZone1].rangeEnd_m,
      pSetup->guardzones.zone[eGuardZone1].azimuth_ddeg / 10.0,
      pSetup->guardzones.zone[eGuardZone1].width_ddeg / 10.0);

  bool gz2Enabled = pSetup->guardzones.active[eGuardZone2];
  std::cout << "GuardZone2: " << (gz2Enabled == true ? "true" : "false")
            << std::endl;
  std::cout << "Guard2Range: "
            << pSetup->guardzones.zone[eGuardZone2].rangeStart_m << " - "
            << pSetup->guardzones.zone[eGuardZone2].rangeEnd_m << std::endl;
  std::cout << "Guard2Bearing: "
            << pSetup->guardzones.zone[eGuardZone2].azimuth_ddeg / 10.0
            << std::endl;
  std::cout << "Guard2Width: "
            << pSetup->guardzones.zone[eGuardZone2].width_ddeg / 10.0
            << std::endl;
  std::cout << "Guard2AlarmType: "
            << pSetup->guardzones.alarmType[eGuardZone2].alarmType << std::endl;

  m_OverlayManager.SetGuardZone(
      eGuardZone2, gz2Enabled,
      pSetup->guardzones.zone[eGuardZone2].rangeStart_m,
      pSetup->guardzones.zone[eGuardZone2].rangeEnd_m,
      pSetup->guardzones.zone[eGuardZone2].azimuth_ddeg / 10.0,
      pSetup->guardzones.zone[eGuardZone2].width_ddeg / 10.0);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetupExtended(
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
            << static_cast<unsigned>(pSetupExtended->noiseReject) << std::endl;
  std::cout << "BeamSharpening: "
            << static_cast<unsigned>(pSetupExtended->beamSharpening)
            << std::endl;
  std::cout << "STCCurveType: "
            << static_cast<unsigned>(pSetupExtended->stcCurveType) << std::endl;
  std::cout << "FastScan: "
            << static_cast<unsigned>(pSetupExtended->fastScanMode) << std::endl;
  std::cout << "RPM: " << pSetupExtended->rpmX10 / 10.0 << std::endl;
  std::cout << "Sea: " << pSetupExtended->sea.manualValue << std::endl;
  std::cout << "SeaAuto: " << pSetupExtended->sea.autoOffset << std::endl;
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties(
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
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateConfiguration(
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
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateFeature(
    const Navico::Protocol::NRP::tFeatureEnum* pFeature) {
  if (pFeature != nullptr) {
    Navico::Protocol::NRP::tFeatureManager& featureManager =
        m_pImageClient->GetFeatureManager();

    std::cout << "\n # Feature updating #.....................\n";
    switch (static_cast<Navico::Protocol::NRP::tFeatureEnum>(*pFeature)) {
      case Navico::Protocol::NRP::eFeatureEnum_SupportedUseModes: {
        UpdateUseMode();
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_IRControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureIR();
        std::cout << "IR Level Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
        ExtractFeatureControlEnum(feature);
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_NoiseRejectControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureNoiseReject();
        ExtractFeatureControlEnum(feature);
        std::cout << "Noise Reject Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_STCCurveControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureStcCurves();
        ExtractFeatureControlEnum(feature, StcCurveStrings());
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_BeamSharpeningControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureBeamSharpening();
        std::cout << "BeamSharpening Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
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
        std::cout << "LocalIR Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
        ExtractFeatureControlEnum(feature);
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_LEDControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureLED();
        std::cout << "LEDs Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
        ExtractFeatureControlEnum(feature);
      } break;

      case Navico::Protocol::NRP::eFeatureEnum_TargetStretchControl: {
        const Navico::Protocol::NRP::tFeatureLevel& feature =
            featureManager.GetFeatureTargetStretch();
        std::cout << "TargetStretch Range: " << 0 << "-"
                  << static_cast<unsigned>(feature.maxLevel) << std::endl;
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
    }
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateGuardZoneAlarm(
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
        gz_state = "Unknown " + std::to_string(m_pGuardZoneAlarms[zone].state);
      } break;
    }

    if (zone == eGuardZone1) {
      std::cout << "Guard1Type: " << gz_type << std::endl;
      std::cout << "Guard1State: " << gz_state << std::endl;
    } else {
      std::cout << "Guard2Type: " << gz_type << std::endl;
      std::cout << "Guard2State: " << gz_state << std::endl;
    }
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateRadarError(
    const Navico::Protocol::NRP::tRadarError* pError) {
  // NOTE: this simplification may overwrite a previous error that hasn't yet
  // been reported
  std::cout << "Scanner error: " +
                   ToString(
                       Navico::Protocol::NRP::eRadarErrorType(pError->type));
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

    //    std::thread _socketthread(&GUIDemo::updateboatstate, this);
    //    _socketthread.detach();
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
    unsigned range =
        static_cast<unsigned>((sample * m_PixelCellSize_mm) / 1000 + 0.5);
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
