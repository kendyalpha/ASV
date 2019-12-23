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
#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::perception {

class SpokeProcessing {
 public:
  explicit SpokeProcessing(const AlarmZone &_AlarmZone,
                           const SpokeProcessdata &_SpokeProcessdata)
      : Alarm_Zone(_AlarmZone), SpokeProcess_data(_SpokeProcessdata) {}
  virtual ~SpokeProcessing() = default;

  SpokeProcessing &DetectionOnSpoke(const uint8_t *_spoke_array,
                                    const std::size_t _array_size,
                                    const double _spoke_azimuth_deg,
                                    const double _samplerange_m,
                                    const double _vessel_x_m = 0.0,
                                    const double _vessel_y_m = 0.0,
                                    const double _vessel_theta_rad = 0.0) {
    find_surroundings_spoke(_spoke_array, _array_size, _spoke_azimuth_deg,
                            _samplerange_m, SpokeProcess_RTdata);
    InAlarmZone(SpokeProcess_RTdata);
    convert_surroundings_to_marine(_vessel_x_m, _vessel_y_m, _vessel_theta_rad,
                                   SpokeProcess_RTdata);
    return *this;
  }  // DetectionOnSpoke

  SpokeProcessRTdata getSpokeProcessRTdata() const noexcept {
    return SpokeProcess_RTdata;
  }  // getSpokeProcessRTdata

  double getsampletime() const noexcept {
    return SpokeProcess_data.sample_time;
  }

 private:
  const AlarmZone Alarm_Zone;
  const SpokeProcessdata SpokeProcess_data;
  SpokeProcessRTdata SpokeProcess_RTdata;

  // check if the surroundings are in the alarm zone
  void InAlarmZone(SpokeProcessRTdata &_SpokeProcessRTdata) {
    std::vector<double> surroundings_bearing_rad;
    std::vector<double> surroundings_range_m;

    std::size_t num_surroundings =
        _SpokeProcessRTdata.surroundings_bearing_rad.size();
    for (std::size_t i = 0; i != num_surroundings; ++i) {
      double s_range_m = _SpokeProcessRTdata.surroundings_range_m[i];
      double s_bearing_rad = _SpokeProcessRTdata.surroundings_bearing_rad[i];

      if ((s_range_m <= Alarm_Zone.end_range_m) &&
          (Alarm_Zone.start_range_m <= s_range_m) &&
          (std::abs(common::math::Normalizeheadingangle(
               s_bearing_rad - Alarm_Zone.center_bearing_rad)) <=
           0.5 * Alarm_Zone.width_bearing_rad)) {
        surroundings_bearing_rad.emplace_back(s_bearing_rad);
        surroundings_range_m.emplace_back(s_range_m);
      }
    }
    // update the surroundings in the alarm zone
    _SpokeProcessRTdata.surroundings_bearing_rad = surroundings_bearing_rad;
    _SpokeProcessRTdata.surroundings_range_m = surroundings_range_m;

  }  // InAlarmZone

  // convert the body-fixed coordinate to marine
  void convert_surroundings_to_marine(const double _vessel_x_m,
                                      const double _vessel_y_m,
                                      const double _vessel_theta_rad,
                                      SpokeProcessRTdata &_SpokeProcessRTdata) {
    double cvalue = std::cos(_vessel_theta_rad);
    double svalue = std::sin(_vessel_theta_rad);

    std::size_t num_surroundings =
        _SpokeProcessRTdata.surroundings_bearing_rad.size();

    std::vector<double> surroundings_x_m(num_surroundings, 0.0);
    std::vector<double> surroundings_y_m(num_surroundings, 0.0);

    for (std::size_t i = 0; i != num_surroundings; ++i) {
      double bearing_rad = _SpokeProcessRTdata.surroundings_bearing_rad[i];
      double range_m = _SpokeProcessRTdata.surroundings_range_m[i];
      double cvalue_plus = std::cos(_vessel_theta_rad + bearing_rad);
      double svalue_plus = std::sin(_vessel_theta_rad + bearing_rad);

      double xs = cvalue * SpokeProcess_data.radar_x -
                  svalue * SpokeProcess_data.radar_y + range_m * cvalue_plus +
                  _vessel_x_m;

      double ys = svalue * SpokeProcess_data.radar_x +
                  cvalue * SpokeProcess_data.radar_y + range_m * svalue_plus +
                  _vessel_y_m;

      surroundings_x_m[i] = xs;
      surroundings_y_m[i] = ys;
    }

    _SpokeProcessRTdata.surroundings_x_m = surroundings_x_m;
    _SpokeProcessRTdata.surroundings_y_m = surroundings_y_m;
  }  // convert_surroundings_to_marine

  // find all surroundings on one spoke data
  void find_surroundings_spoke(const uint8_t *_spoke_array,
                               const std::size_t _array_size,
                               const double _spoke_azimuth_deg,
                               const double _samplerange_m,
                               SpokeProcessRTdata &_SpokeProcessRTdata) {
    double _spoke_azimuth_rad = common::math::Normalizeheadingangle(
        common::math::Degree2Rad(_spoke_azimuth_deg));

    // find the index of all elements larger than threhold value
    auto surroundings_index = find_above_allelements_index<uint8_t>(
        _spoke_array, _array_size, Alarm_Zone.sensitivity_threhold);
    std::size_t num_surroundings = surroundings_index.size();
    std::vector<double> surroundings_bearing_rad(num_surroundings,
                                                 _spoke_azimuth_rad);
    std::vector<double> surroundings_range_m(num_surroundings, 0.0);

    double interception_empirical = 8.0;
    for (std::size_t i = 0; i != num_surroundings; ++i) {
      surroundings_range_m[i] =
          interception_empirical + _samplerange_m * (surroundings_index[i] + 1);
    }
    _SpokeProcessRTdata.surroundings_bearing_rad = surroundings_bearing_rad;
    _SpokeProcessRTdata.surroundings_range_m = surroundings_range_m;

  }  // find_surroundings_spoke

  template <class T>
  std::vector<T> find_above_allelements(const T *arr, const int array_size,
                                        const T _threhold) {
    std::vector<T> target;
    std::copy_if(arr, arr + array_size, std::back_inserter(target),
                 [=](T element) { return element >= _threhold; });
    return target;
  }  // find_above_allelements

  template <class T>
  std::vector<std::size_t> find_above_allelements_index(
      const T *arr, const std::size_t array_size, const T _threhold) {
    std::vector<std::size_t> results;
    for (std::size_t i = 0; i != array_size; ++i) {
      if (arr[i] >= _threhold) results.emplace_back(i);
    }
    return results;
  }  // find_above_allelements_index

};  // end class SpokeProcessing

}  // namespace ASV::perception

#endif /* _SPOKEPROCESSING_H_ */
