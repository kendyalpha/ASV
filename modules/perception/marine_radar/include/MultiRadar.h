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
#include <string>

namespace ASV::perception {

class MultiRadar : public Navico::Protocol::iRadarListObserver,
                   public Navico::Protocol::iUnlockStateObserver,
                   public Navico::Protocol::iUnlockKeySupplier {
 public:
  MultiRadar() {}
  virtual ~MultiRadar() = default;

  // Return the text of the current selection
  std::string GetRadarSelection() {}

  // Return the serial-number of the selected radar
  std::string GetRadarSerialNumber() {}
  // Return the instance/range selected
  unsigned GetRadarInstance();

  void InitiateUnlock();
  void SetConnectState(bool connected);

 private:
};

}  // namespace ASV::perception

#endif /* _MULTIRADAR_H_ */