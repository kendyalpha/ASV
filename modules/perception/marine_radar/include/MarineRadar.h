/*
****************************************************************************
* MarineRadar.h:
* Marine radar for target tracking
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _MARINERADAR_H_
#define _MARINERADAR_H_

#include <ClientErrors.h>
#include <Feature.h>
#include <FeatureManager.h>
#include <ImageClient.h>
#include <ImageClientObserver.h>
#include <MultiRadarClient.h>
#include <NavRadarProtocol.h>
#include <PPIController.h>
#include <TargetTrackingClient.h>

#include "MultiRadar.h"

namespace ASV::perception {

class MarineRadar
    : public Navico::Protocol::NRP::iImageClientStateObserver,
      public Navico::Protocol::NRP::iImageClientSpokeObserver,
      public Navico::Protocol::NRP::iFeatureObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientStateObserver {
  enum GUARDZONE { eGuardZone1 = 0, eGuardZone2 };

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
        m_pTargetLocations(nullptr) {
    m_pImageClient = new Navico::Protocol::NRP::tImageClient();
    m_pTargetClient = new Navico::Protocol::NRP::tTargetTrackingClient();
    InitProtocolData();
    m_pMultiRadar = new MultiRadar();
  }
  ~MarineRadar() {}

 private:
  //-------------------------------------------------------------------------
  //  Observer Callbacks
  //-------------------------------------------------------------------------
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
  int ConnectImageClient(const std::string& serialNumber, unsigned instance) {
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
  void DisconnectImageClient() {
    if (m_pImageClient) {
      m_pImageClient->Disconnect();
      m_pImageClient->RemoveStateObserver(this);
      m_pImageClient->RemoveSpokeObserver(this);
    }
  }

  //-------------------------------------------------------------------------
  //  Target Tracking Client
  //-------------------------------------------------------------------------
  int ConnectTargetClient(const std::string& serialNumber, unsigned instance) {
    int error = Navico::Protocol::EOK + 1;
    if (m_pTargetClient) {
      m_pTargetClient->AddStateObserver(this);
      m_pTargetClient->AddTargetTrackingObserver(this);

      m_ForceTargetAlarmSetup = true;
      m_ForceTargetProperties = true;

      error = m_pTargetClient->Connect(serialNumber.c_str(), instance);

      // std::thread _socketthread(&GUIDemo::updateboatstate, this);
      // _socketthread.detach();
    }
    return error;
  }
  void DisconnectTargetClient() {
    if (m_pTargetClient) {
      m_pTargetClient->RemoveStateObserver(this);
      m_pTargetClient->RemoveTargetTrackingObserver(this);
      m_pTargetClient->Disconnect();
    }
  }

  void updateboatstate() {
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

      std::cout << "set_results: " << (set_results == true ? "true" : "false")
                << std::endl;
      //      if (set_results)
      //        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      //      else
      //        continue;
    }
  }

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
  tTargetLocation* m_pTargetLocations;

  // flags to force cached data notification updates
  bool m_ForceSetup;
  bool m_ForceAdvancedSTCState;
  bool m_ForceTargetAlarmSetup;
  bool m_ForceTargetProperties;
  bool m_ForceNavigationState;

  MultiRadar* m_pMultiRadar;
};

}  // namespace ASV::perception

#endif /* _MARINERADAR_H_ */