//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "MultiRadar.h"

//-----------------------------------------------------------------------------
//  tMultiRadar Implementation
//-----------------------------------------------------------------------------
tMultiRadar::tMultiRadar(Ui::GUIDemoClass& myUI, QObject* pParent)
    : QObject(pParent), ui(myUI) {
  ConnectControls(true, *this, *ui.groupMultiRadar);

  Navico::Protocol::tMultiRadarClient* pMultiRadarClient =
      Navico::Protocol::tMultiRadarClient::GetInstance();
  pMultiRadarClient->AddRadarListObserver(this);
  pMultiRadarClient->AddUnlockStateObserver(this);
  pMultiRadarClient->SetUnlockKeySupplier(this);
  pMultiRadarClient->Connect();
  pMultiRadarClient->QueryRadars();
}

//-----------------------------------------------------------------------------
tMultiRadar::~tMultiRadar() {
  ConnectControls(false, *this, *ui.groupMultiRadar);

  Navico::Protocol::tMultiRadarClient* pMultiRadarClient =
      Navico::Protocol::tMultiRadarClient::GetInstance();
  pMultiRadarClient->Disconnect();
  pMultiRadarClient->SetUnlockKeySupplier(nullptr);
  pMultiRadarClient->RemoveUnlockStateObserver(this);
  pMultiRadarClient->RemoveRadarListObserver(this);
}

//-----------------------------------------------------------------------------
void tMultiRadar::InitiateUnlock() {
  // prepare the unlock key
  uint8_t key_data[MAX_UNLOCKKEY_SIZE];
  std::string unlockKey =
      "FCA53C6FE25450CA93E4AF63B78769E659E56E2705AFAD443F1D6EDBEFB249FF2C7AFD"
      "7589122F80DE38FA32638C36F195816F5EE5C1257EFFED4A02537252FE";
  unsigned len = hexstring_to_uint8(unlockKey, key_data);
  std::cout << current_radar_serial_number << std::endl;
  Navico::Protocol::tMultiRadarClient::GetInstance()->UnlockRadar(
      current_radar_serial_number.c_str(), key_data, len, 0);
}

//-----------------------------------------------------------------------------
void tMultiRadar::SetConnectState(bool connected) {
  ui.pushMultiRadarConnect->setChecked(connected);
}

//-----------------------------------------------------------------------------
// Observer Callbacks and Handling
//-----------------------------------------------------------------------------
void tMultiRadar::UpdateRadarList(const char* /*pSerialNumber*/,
                                  iRadarListObserver::eAction /*action*/) {
  multi_radar_devices.clear();

  char radars[cMaxRadars][MAX_SERIALNUMBER_SIZE];

  unsigned numRadars =
      Navico::Protocol::tMultiRadarClient::GetInstance()->GetRadars(radars,
                                                                    cMaxRadars);
  if (numRadars > 0) {
    if (numRadars > cMaxRadars) numRadars = cMaxRadars;
    for (unsigned i = 0; i < numRadars; ++i) {
      int numImageServices = Navico::Protocol::tMultiRadarClient::GetInstance()
                                 ->GetImageStreamCount(radars[i]);
      if (numImageServices > 0) {
        std::string serialNumber(radars[i]);
        std::vector<std::string> ImageServicesList;
        for (int j = 0; j < numImageServices; ++j)
          ImageServicesList.push_back(serialNumber + std::string(1, 'A' + j));

        multi_radar_devices.insert(
            std::pair<unsigned, std::vector<std::string>>(i,
                                                          ImageServicesList));
      }
    }
    current_radar_serial_number = std::string(radars[current_radar_index]);
  } else
    multi_radar_devices.clear();

  //      for (auto const& x : multi_radar_devices)
  //      {
  //          std::cout << x.first  // string (key)
  //                    << ':';
  //          for(auto const& value: x.second ) {
  //              std::cout<<value<< ", ";
  //          }
  //                    std::cout<< std::endl ;
  //      }
}

//-----------------------------------------------------------------------------
int tMultiRadar::GetUnlockKey(const char* /*pSerialNumber*/,
                              const uint8_t* /*pLockID*/,
                              unsigned /*lockIDSize*/, uint8_t* /*pUnlockKey*/,
                              unsigned /*maxUnlockKeySize*/) {
  // can't return an unlock key immediately because we need to prompt the user
  return -1;
}

//-----------------------------------------------------------------------------
void tMultiRadar::UpdateUnlockState(const char* pSerialNumber, int _lockState) {
  std::string str_radar(pSerialNumber);
  str_radar = "Radar " + str_radar;

  if (_lockState > 0)
    std::cout << str_radar + " unlocked (level " + std::to_string(_lockState) +
                     ")\n";
  else if (_lockState == 0)
    std::cout << str_radar + " still locked\n";
  else
    std::cout << "Unlock of " + str_radar + " Failed\n";
}

//-----------------------------------------------------------------------------
// UI Event Handling
//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarConnect_clicked(bool checked) {
  emit ConnectChanged(checked);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarUnlock_clicked(bool) { InitiateUnlock(); }
