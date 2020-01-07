/*
****************************************************************************
* MultiRadar.h:
* Handling radars - listing, selecting and unlocking.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _MULTIRADAR_H_
#define _MULTIRADAR_H_

#include <MultiRadarClient.h>
#include <map>
#include <string>
#include <vector>

namespace ASV::messages {

class MultiRadar : public Navico::Protocol::iRadarListObserver,
                   public Navico::Protocol::iUnlockStateObserver,
                   public Navico::Protocol::iUnlockKeySupplier {
 public:
  MultiRadar() {
    Navico::Protocol::tMultiRadarClient* pMultiRadarClient =
        Navico::Protocol::tMultiRadarClient::GetInstance();
    pMultiRadarClient->AddRadarListObserver(this);
    pMultiRadarClient->AddUnlockStateObserver(this);
    pMultiRadarClient->SetUnlockKeySupplier(this);
    pMultiRadarClient->Connect();
    pMultiRadarClient->QueryRadars();
  }
  ~MultiRadar() {
    Navico::Protocol::tMultiRadarClient* pMultiRadarClient =
        Navico::Protocol::tMultiRadarClient::GetInstance();
    pMultiRadarClient->Disconnect();
    pMultiRadarClient->SetUnlockKeySupplier(nullptr);
    pMultiRadarClient->RemoveUnlockStateObserver(this);
    pMultiRadarClient->RemoveRadarListObserver(this);
  }

  // unlock the SDK
  int InitiateUnlock() {
    // prepare the unlock key
    uint8_t key_data[MAX_UNLOCKKEY_SIZE];
    std::string unlockKey =
        "FCA53C6FE25450CA93E4AF63B78769E659E56E2705AFAD443F1D6EDBEFB249FF2C7AFD"
        "7589122F80DE38FA32638C36F195816F5EE5C1257EFFED4A02537252FE";
    unsigned len = hexstring_to_uint8(unlockKey, key_data);
    return Navico::Protocol::tMultiRadarClient::GetInstance()->UnlockRadar(
        current_radar_serial_number.c_str(), key_data, len, 0);
  }

  // Return the text of the current selection
  std::string GetRadarSelection() {
    return multi_radar_devices[current_radar_index][0];
  }
  // Return the serial-number of the selected radar
  std::string GetRadarSerialNumber() { return current_radar_serial_number; }
  // Return the instance/range selected
  unsigned GetRadarInstance() { return current_radar_index; }

  // iRadarListObserver, iUnlockKeySupplier, iUnlockStateObserver - multi-device
  // callbacks
  virtual void UpdateRadarList(const char* /*pSerialNumber*/,
                               iRadarListObserver::eAction /*action*/) {
    multi_radar_devices.clear();

    constexpr unsigned cMaxRadars = 8;
    char radars[cMaxRadars][MAX_SERIALNUMBER_SIZE];

    unsigned numRadars =
        Navico::Protocol::tMultiRadarClient::GetInstance()->GetRadars(
            radars, cMaxRadars);
    if (numRadars > 0) {
      if (numRadars > cMaxRadars) numRadars = cMaxRadars;
      for (unsigned i = 0; i < numRadars; ++i) {
        int numImageServices =
            Navico::Protocol::tMultiRadarClient::GetInstance()
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
  }  // UpdateRadarList

  virtual int GetUnlockKey(const char* /*pSerialNumber*/,
                           const uint8_t* /*pLockID*/, unsigned /*lockIDSize*/,
                           uint8_t* /*pUnlockKey*/,
                           unsigned /*maxUnlockKeySize*/) {
    return -1;
  }  // GetUnlockKey

  virtual void UpdateUnlockState(const char* pSerialNumber, int _lockState) {
    std::string str_radar(pSerialNumber);
    str_radar = "Radar " + str_radar;

    if (_lockState > 0)
      std::cout << str_radar + " unlocked (level " +
                       std::to_string(_lockState) + ")\n";
    else if (_lockState == 0)
      std::cout << str_radar + " still locked\n";
    else
      std::cout << "Unlock of " + str_radar + " Failed\n";
  }  // UpdateUnlockState

 private:
  std::map<unsigned, std::vector<std::string>> multi_radar_devices;
  std::string current_radar_serial_number;
  unsigned current_radar_index = 0;

  unsigned hexstring_to_uint8(const std::string& hexText, uint8_t* pData) {
    std::size_t len = hexText.length();
    for (std::size_t i = 0; i < len; i += 2) {
      std::string byteString = hexText.substr(i, 2);
      pData[i / 2] =
          static_cast<uint8_t>(strtol(byteString.c_str(), nullptr, 16));
    }
    return static_cast<unsigned>(len / 2);
  }
};
}  // namespace ASV::messages

#endif /* _MULTIRADAR_H_ */