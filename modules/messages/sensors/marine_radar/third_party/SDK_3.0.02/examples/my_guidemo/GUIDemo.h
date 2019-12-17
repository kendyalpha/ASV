//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file GUIDemo.h
//!
//! Window for testing/demonstrating the NRP library protocol
//-----------------------------------------------------------------------------
#ifndef GUIDEMO_H
#define GUIDEMO_H

#include <ClientErrors.h>
#include <Feature.h>
#include <FeatureManager.h>
#include <ImageClient.h>
#include <ImageClientObserver.h>
#include <MultiRadarClient.h>
#include <NavRadarProtocol.h>
#include <PPIController.h>
#include <TargetTrackingClient.h>

#include <QMainWindow>
#include <QMessageBox>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include "MultiRadar.h"
#include "QControlUtils.h"
#include "TabPPI.h"
#include "TabTargets.h"

#include "common/communication/include/tcpclient.h"
#include "ui_GUIDemo.h"
//-----------------------------------------------------------------------------
// GUIDemo Class
//-----------------------------------------------------------------------------

enum { eGuardZone1 = 0, eGuardZone2 };

class GUIDemo
    : public QMainWindow,
      public Navico::Protocol::NRP::iImageClientStateObserver,
      public Navico::Protocol::NRP::iImageClientSpokeObserver,
      public Navico::Protocol::NRP::iFeatureObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientStateObserver {
  Q_OBJECT

 public:
  GUIDemo(QWidget* parent = nullptr, Qt::WindowFlags flags = nullptr);
  ~GUIDemo();

 private:
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
    InitStruct(m_pTargetLocations, cMaxTargets);
  }

  void MakeConnections(bool connect);

  //-------------------------------------------------------------------------
  //  Observer Callbacks
  //-------------------------------------------------------------------------
 private:
  // iImageClientSpokeObserver callbacks (real time)
  virtual void UpdateSpoke(
      const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke);

  // iImageClientStateObserver callbacks
  virtual void UpdateMode(const Navico::Protocol::NRP::tMode* pMode);
  virtual void UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup);
  virtual void UpdateSetupExtended(
      const Navico::Protocol::NRP::tSetupExtended* pSetupExtended);
  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tProperties* pProperties);
  virtual void UpdateConfiguration(
      const Navico::Protocol::NRP::tConfiguration* pConfiguration);
  virtual void UpdateGuardZoneAlarm(
      const Navico::Protocol::NRP::tGuardZoneAlarm* pAlarm);
  virtual void UpdateRadarError(
      const Navico::Protocol::NRP::tRadarError* pAlarm);
  virtual void UpdateAdvancedState(
      const Navico::Protocol::NRP::tAdvancedSTCState* pState);

  // iFeatureObserver callbacks
  virtual void UpdateFeature(
      const Navico::Protocol::NRP::tFeatureEnum* pFeature);

  // iTargetTrackingClientObserver callbacks
  virtual void UpdateTarget(
      const Navico::Protocol::NRP::tTrackedTarget* pTarget);

  // iTargetTrackingClientStateObserver callbacks
  virtual void UpdateNavigation(
      const Navico::Protocol::NRP::tNavigationState* pNavigationState);
  virtual void UpdateAlarmSetup(
      const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* pAlarmSetup);
  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tTargetTrackingProperties* pProperties);

  //-------------------------------------------------------------------------
  //  Image Client
  //-------------------------------------------------------------------------
 private:
  int ConnectImageClient(const std::string& serialNumber, unsigned instance);
  void DisconnectImageClient();

 private:
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

  // flags to force cached data notification updates
  bool m_ForceSetup;
  bool m_ForceAdvancedSTCState;

  //-------------------------------------------------------------------------
  //  Target Tracking Client
  //-------------------------------------------------------------------------
 public:
  static const unsigned cMaxTargets = 10;

 private:
  int ConnectTargetClient(const std::string& serialNumber, unsigned instance);
  void DisconnectTargetClient();
  [[noreturn]] void updateboatstate() {
    union socketmsg {
      double double_msg[2];
      char char_msg[16];
    };
    const int recv_size = 16;
    const int send_size = 10;
    socketmsg _recvmsg;
    char send_buffer[send_size] = "socket";

    tcpclient _tcpclient("127.0.0.1", "9340");

    while (1) {
      _tcpclient.senddata(_recvmsg.char_msg, send_buffer, recv_size, send_size);
      std::cout << "gps: " << _recvmsg.double_msg[0] << " "
                << _recvmsg.double_msg[1] << std::endl;

      bool set_results = false;
      set_results = m_pTargetClient->SetBoatSpeed(
          Navico::Protocol::NRP::eSpeedType::eSpeedOverGround,
          static_cast<uint32_t>(_recvmsg.double_msg[1]),
          Navico::Protocol::NRP::eDirectionType::eHeadingTrue,
          static_cast<uint32_t>(_recvmsg.double_msg[0]));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      std::cout<<"set_results: "<<(set_results==true?"true":"false")<<std::endl;
//      if (set_results)
//        std::this_thread::sleep_for(std::chrono::milliseconds(500));
//      else
//        continue;
    }
  }

  signals :
      // signals from callbacks
  void UpdateTargetAlarmSetup_signal();
  void UpdateTargetTarget_signal(unsigned targetID);

 private slots:
  // slots for callbacks
  void TargetAcquire(double sample, double degrees);
  void UpdateTargetAlarmSetup_slot();
  void UpdateTargetTarget_slot(unsigned target);
  void MultiRadar_ConnectChanged(bool connect);

 private:
  // working variables
  Navico::Protocol::NRP::tNavigationState* m_pNavigationState;
  Navico::Protocol::NRP::tTargetTrackingClient* m_pTargetClient;
  Navico::Protocol::NRP::tTargetTrackingAlarmSetup* m_pTargetAlarmSetup;
  Navico::Protocol::NRP::tTargetTrackingProperties* m_pTargetProperties;
  Navico::Protocol::NRP::tTrackedTarget* m_pTargets;
  tTargetLocation* m_pTargetLocations;

  // flags to force cached data notification updates
  bool m_ForceTargetAlarmSetup;
  bool m_ForceTargetProperties;
  bool m_ForceNavigationState;

 private:
  tMultiRadar* m_pMultiRadar;
  tTabTargets* m_pTabTargets;
  tTabBScan* m_pTabBScan;
  tTabPPI* m_pTabPPI;

  Ui::GUIDemoClass ui;

  tOverlayManager m_OverlayManager;
  bool m_AlarmTypes[Navico::Protocol::NRP::cMaxGuardZones];

  const char g_Off[4] = "Off";
  const char g_On[3] = "On";
  const char g_Low[4] = "Low";
  const char g_MedLow[8] = "Med-Low";
  const char g_Medium[7] = "Medium";
  const char g_MedHigh[9] = "Med-High";
  const char g_High[5] = "High";

  const char* g_Severity1[1] = {g_Off};
  const char* g_Severity2[2] = {g_Off, g_On};
  const char* g_Severity3[3] = {g_Off, g_Low, g_High};
  const char* g_Severity4[4] = {g_Off, g_Low, g_Medium, g_High};
  const char* g_Severity5[5] = {g_Off, g_Low, g_MedLow, g_MedHigh, g_High};
  const char* g_Severity6[6] = {g_Off,    g_Low,     g_MedLow,
                                g_Medium, g_MedHigh, g_High};

  const char** g_Severities[6] = {g_Severity1, g_Severity2, g_Severity3,
                                  g_Severity4, g_Severity5, g_Severity6};

  const char* g_STCCurveStrs[3] = {"Calm", "Moderate", "Rough"};

  struct tConstStrings {
    unsigned nStrings;
    const char** ppStrings;

    tConstStrings(unsigned nStrs = 0, const char** ppStrs = nullptr)
        : nStrings(nStrs), ppStrings(ppStrs) {}

    template <int N>
    tConstStrings(const char* (&strs)[N]) : nStrings(N), ppStrings(strs) {}
  };

  std::string ToString(tConstStrings names) {
    std::string result;
    if (names.nStrings > 0) {
      result += names.ppStrings[0];
      for (unsigned index = 1; index < names.nStrings; ++index) {
        result += "|";
        result += names.ppStrings[index];
      }
    }
    return result;
  }

  std::string ToString(Navico::Protocol::NRP::eRadarErrorType type) {
    switch (type) {
      case Navico::Protocol::NRP::eErrorPersistenceCorrupt:
        return "Persistence Corrupt";
      case Navico::Protocol::NRP::eErrorZeroBearingFault:
        return "Zero Bearing Fault";
      case Navico::Protocol::NRP::eErrorBearingPulseFault:
        return "Bearing Pulse Fault";
      case Navico::Protocol::NRP::eErrorMotorNotRunning:
        return "Motor Not Running";
      case Navico::Protocol::NRP::eErrorCommsNotActive:
        return "CommsNotActive";
      case Navico::Protocol::NRP::eErrorMagnetronHeaterVoltage:
        return "Magnetron Heater Voltage";
      case Navico::Protocol::NRP::eErrorModulationVoltage:
        return "Modulation Voltage";
      case Navico::Protocol::NRP::eErrorTriggerFault:
        return "Trigger Fault";
      case Navico::Protocol::NRP::eErrorVideoFault:
        return "Video Fault";
      case Navico::Protocol::NRP::eErrorFanFault:
        return "Fan Fault";
      case Navico::Protocol::NRP::eErrorScannerConfigFault:
        return "Scanner Config Fault";
      case Navico::Protocol::NRP::eErrorPowerSupplyTransient:
        return "Power Supply Transient";
      case Navico::Protocol::NRP::eErrorScannerDetectFail:
        return "Scanner Detect Fail";
      case Navico::Protocol::NRP::eErrorPASoftOverheat:
        return "PA Soft-Overheat";
      case Navico::Protocol::NRP::eErrorPAHardOverheat:
        return "PA Hard-Overheat";
      case Navico::Protocol::NRP::eErrorGWDatapathError:
        return "GW Datapath Error";
      case Navico::Protocol::NRP::eErrorPSUOverheat:
        return "PSU Overheat";
      case Navico::Protocol::NRP::eErrorPSUVoltage:
        return "PSU Voltage";
      case Navico::Protocol::NRP::eErrorPSUPower:
        return "PSU Power";
      case Navico::Protocol::NRP::eErrorPSUHWFault:
        return "PSU HW-Fault";
      case Navico::Protocol::NRP::eErrorPSUCommsFault:
        return "PSU Comms-Fault";
      case Navico::Protocol::NRP::eErrorMechanicalFault:
        return "Mechanical Fault";
      case Navico::Protocol::NRP::eErrorLEDFault:
        return "LED Fault";
      case Navico::Protocol::NRP::eErrorScannerFail:
        return "Scanner Fail";
      case Navico::Protocol::NRP::eErrorRIFault:
        return "Radar-Interface Fault";
      case Navico::Protocol::NRP::eErrorLowBattery:
        return "Low Battery";
      case Navico::Protocol::NRP::eErrorMotorStall:
        return "Motor Stall";
      case Navico::Protocol::NRP::eErrorSafetyMode:
        return "Safety Mode";
      default:
        return "Radar Error" + std::to_string(type);
    }
  }

  tConstStrings SeverityStrings(uint8_t nValues) {
    int length = sizeof(g_Severities) / sizeof(*g_Severities);
    if (nValues > length) {
      return tConstStrings();
    }

    return tConstStrings(nValues, g_Severities[nValues - 1]);
  }

  tConstStrings StcCurveStrings() { return tConstStrings(3, g_STCCurveStrs); }

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
    switch (useMode) {
      case Navico::Protocol::NRP::eUseMode_Custom:
        return "Manual";
      case Navico::Protocol::NRP::eUseMode_Harbour:
        return "Harbour";
      case Navico::Protocol::NRP::eUseMode_Offshore:
        return "Offshore";
      case Navico::Protocol::NRP::eUseMode_Buoy:
        return "Buoy";
      case Navico::Protocol::NRP::eUseMode_Weather:
        return "Weather";
      case Navico::Protocol::NRP::eUseMode_Bird:
        return "Bird";
      case Navico::Protocol::NRP::eUseMode_Netfinder:
        return "Netfinder";
      case Navico::Protocol::NRP::eUseMode_SaRT:
        return "SaRT";
      case Navico::Protocol::NRP::eUseMode_Doppler:
        return "Doppler";
      case Navico::Protocol::NRP::eUseMode_RTE:
        return "RTE";
      default:
        return "Unknown " + std::to_string(useMode);
    }
  }  // UseModeToString

  //-----------------------------------------------------------------------------
  void ExtractFeatureControlEnum(
      const Navico::Protocol::NRP::tFeatureLevel& feature,
      tConstStrings names) {
    if (feature.supported == true) {
      std::cout << "[" << feature.maxLevel + 1 << "]"
                << " " << ToString(names);
    }
  }

  //-----------------------------------------------------------------------------
  void ExtractFeatureControlEnum(
      const Navico::Protocol::NRP::tFeatureLevel& feature) {
    ExtractFeatureControlEnum(feature, SeverityStrings(feature.maxLevel + 1));
  }

  void DefaultFeatureControlEnum(bool defaultState) {
    if (defaultState) {
      std::cout << "default\n";
    } else {
      std::cout << "default - unsupported\n";
    }
  }

  std::string ErrorToString(Navico::Protocol::eErrors error) {
    switch (error) {
      case Navico::Protocol::EOK:
        return "EOK";
      case Navico::Protocol::ELocked:
        return "ELocked";
      case Navico::Protocol::EPending:
        return "EPending";
      case Navico::Protocol::ETimedOut:
        return "ETimedOut";
      case Navico::Protocol::EBusy:
        return "EBusy";
      case Navico::Protocol::EBadSerialNumber:
        return "EBadSerialNumber";
      case Navico::Protocol::ENoUnlockKey:
        return "ENoUnlockKey";
      case Navico::Protocol::EBadUnlockKey:
        return "EBadUnlockKey";
      case Navico::Protocol::EWrongUnlockKey:
        return "EWrongUnlockKey";
      case Navico::Protocol::ENotRunning:
        return "ENotRunning";
      case Navico::Protocol::EUnknownRadar:
        return "EUnknownRadar";
      case Navico::Protocol::ENonStdAddress:
        return "ENonStdAddress";
      case Navico::Protocol::ECommsFailure:
        return "ECommsFailure";
      case Navico::Protocol::EThreadCreation:
        return "EThreadCreation";
      case Navico::Protocol::EBadParameter:
        return "EBadParameter";
      case Navico::Protocol::EUnused:
        return "EUnused";
      case Navico::Protocol::EBadUnlockLevel:
        return "EBadUnlockLevel";
      default:
        return "error" + std::to_string(error);
    }
  }

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
  }
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
