/*
****************************************************************************
* SpokeProcessing.h:
* obstacle detection using spoke data from marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _SPOKEPROCESSING_H_
#define _SPOKEPROCESSING_H_

#include <algorithm>
#include <iostream>
#include <vector>

#include "SpokeProcessingData.h"

namespace ASV::perception {

class SpokeProcessing {
 public:
  SpokeProcessing() : max_num_obstacle(10) {}
  virtual ~SpokeProcessing() = default;
  SpokeProcessRTdata getSpokeProcessRTdata() const noexcept {
    return SpokeProcess_RTdata;
  }

 private:
  const AlarmZone Alarm_Zone;
  int max_num_obstacle;
  SpokeProcessRTdata SpokeProcess_RTdata;
  void generateAlarmSpoke() {}

  void findobstacle(const uint8_t *_spoke_array, int _array_size,
                    double _spoke_azimuth_deg, double _samplerange_m) {
    int element_index = std::distance(
        _array, std::find_if(_array, _array + _array_size, [](uint8_t element) {
          return element >= alarmzone.sensitivity_threhold;
        }));
    if (element_index < _array_size) _array[element_index] * _samplerange_m;
    std::cout << "element:" << element_index << std::endl;
    else std::cout << "element is not present in the given array!\n";
  }

  void update_num_obstacle(double _samplerange_m) {}

};  // end class SpokeProcessing

}  // namespace ASV::perception

#endif /* _SPOKEPROCESSING_H_ */
