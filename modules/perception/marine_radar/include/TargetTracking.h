/*
****************************************************************************
* TargetTracking.h:
* Target tracking using marine radar, to get the position and velocity of
* targets
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _TARGETTRACKING_H_
#define _TARGETTRACKING_H_

#include <pyclustering/cluster/dbscan.hpp>
#include <pyclustering/utils/metric.hpp>

#include "common/math/Geometry/include/Miniball.hpp"
#include "common/math/miscellaneous/include/math_utils.h"
#include "common/timer/include/timecounter.h"

#include "RadarFiltering.h"
#include "TargetTrackingData.h"

namespace ASV::perception {

template <int max_num_target = 20>
class TargetTracking : public RadarFiltering {
  using T_Vectord = Eigen::Matrix<double, max_num_target, 1>;
  using T_Vectori = Eigen::Matrix<int, max_num_target, 1>;

 public:
  TargetTracking(const AlarmZone &_AlarmZone,
                 const SpokeProcessdata &_SpokeProcessdata,
                 const ClusteringData &_ClusteringData)
      : RadarFiltering(1, 1),
        Alarm_Zone(_AlarmZone),
        SpokeProcess_data(_SpokeProcessdata),
        Clustering_data(_ClusteringData),
        spoke_state(SPOKESTATE::OUTSIDE_ALARM_ZONE),
        TargetTracking_RTdata({
            T_Vectori::Zero(),  // targets_state
            T_Vectord::Zero(),  // targets_x
            T_Vectord::Zero(),  // targets_y
            T_Vectord::Zero(),  // targets_square_radius
            T_Vectord::Zero(),  // targets_vx
            T_Vectord::Zero(),  // targets_vy
            T_Vectord::Zero(),  // targets_CPA
            T_Vectord::Zero()   // targets_TCPA
        }) {}
  virtual ~TargetTracking() = default;

  TargetTracking &AutoTracking(const uint8_t *_spoke_array,
                               const std::size_t _array_size,
                               const double _spoke_azimuth_deg,
                               const double _samplerange_m,
                               const double _vessel_x_m = 0.0,
                               const double _vessel_y_m = 0.0,
                               const double _vessel_theta_rad = 0.0) {
    static double previous_spoke_azimuth_rad = 0;
    static bool previous_IsInAlarmAzimuth = false;
    static common::timecounter _timer;

    double _spoke_azimuth_rad = common::math::Normalizeheadingangle(
        common::math::Degree2Rad(_spoke_azimuth_deg));

    if (std::abs(common::math::Normalizeheadingangle(
            _spoke_azimuth_rad - previous_spoke_azimuth_rad)) > 0.008) {
      bool current_IsInAlarmAzimuth = IsInAlarmAzimuth(_spoke_azimuth_rad);
      if (current_IsInAlarmAzimuth) {  // in the alarm azimuth
        std::vector<double> surroundings_onespoke_bearing_rad;
        std::vector<double> surroundings_onespoke_range_m;
        std::vector<double> surroundings_onespoke_x_m;
        std::vector<double> surroundings_onespoke_y_m;

        find_surroundings_spoke(
            _spoke_array, _array_size, _spoke_azimuth_rad, _samplerange_m,
            surroundings_onespoke_bearing_rad, surroundings_onespoke_range_m);
        convert_surroundings_to_marine(
            _vessel_x_m, _vessel_y_m, _vessel_theta_rad,
            surroundings_onespoke_bearing_rad, surroundings_onespoke_range_m,
            surroundings_onespoke_x_m, surroundings_onespoke_y_m);

        // append the surroundings in the alarm zone
        SpokeProcess_RTdata.surroundings_bearing_rad.insert(
            SpokeProcess_RTdata.surroundings_bearing_rad.end(),
            surroundings_onespoke_bearing_rad.begin(),
            surroundings_onespoke_bearing_rad.end());
        SpokeProcess_RTdata.surroundings_range_m.insert(
            SpokeProcess_RTdata.surroundings_range_m.end(),
            surroundings_onespoke_range_m.begin(),
            surroundings_onespoke_range_m.end());
        SpokeProcess_RTdata.surroundings_x_m.insert(
            SpokeProcess_RTdata.surroundings_x_m.end(),
            surroundings_onespoke_x_m.begin(), surroundings_onespoke_x_m.end());
        SpokeProcess_RTdata.surroundings_y_m.insert(
            SpokeProcess_RTdata.surroundings_y_m.end(),
            surroundings_onespoke_y_m.begin(), surroundings_onespoke_y_m.end());

        // check the spoke azimuth to determine spoke state
        if (previous_IsInAlarmAzimuth)
          spoke_state = SPOKESTATE::IN_ALARM_ZONE;
        else {
          spoke_state = SPOKESTATE::ENTER_ALARM_ZONE;
          _timer.timeelapsed();
        }

      } else {                            // outside the alarm azimuth
        if (previous_IsInAlarmAzimuth) {  // leaving the alarm azimuth

          long int et_ms = _timer.timeelapsed();

          // start to cluster and miniball
          ClusteringAndMiniBall(SpokeProcess_RTdata.surroundings_x_m,
                                SpokeProcess_RTdata.surroundings_y_m,
                                TargetDetection_RTdata.target_x,
                                TargetDetection_RTdata.target_y,
                                TargetDetection_RTdata.target_square_radius);

          spoke_state = SPOKESTATE::LEAVE_ALARM_ZONE;
        } else {
          SpokeProcess_RTdata.surroundings_bearing_rad.clear();
          SpokeProcess_RTdata.surroundings_range_m.clear();
          SpokeProcess_RTdata.surroundings_x_m.clear();
          SpokeProcess_RTdata.surroundings_y_m.clear();

          spoke_state = SPOKESTATE::OUTSIDE_ALARM_ZONE;
        }
      }
    }  // check if two azimuth is different

    previous_spoke_azimuth_rad = _spoke_azimuth_rad;
    previous_IsInAlarmAzimuth = IsInAlarmAzimuth(previous_spoke_azimuth_rad);

    return *this;
  }  // AutoTracking

  TargetTracking &TestClustering(const std::vector<double> &_surroundings_x,
                                 const std::vector<double> &_surroundings_y) {
    ClusteringAndMiniBall(_surroundings_x, _surroundings_y,
                          TargetDetection_RTdata.target_x,
                          TargetDetection_RTdata.target_y,
                          TargetDetection_RTdata.target_square_radius);

    auto [CPA_X, CPA_Y, TCPA] = computeCPA(0, 0, 2, 0, 0, 4, 2, -2);
    std::cout << CPA_X << std::endl;
    std::cout << CPA_Y << std::endl;
    std::cout << TCPA << std::endl;

    return *this;
  }  // AutoTracking

  TargetDetectionRTdata getTargetDetectionRTdata() const noexcept {
    return TargetDetection_RTdata;
  }  // getTargetDetectionRTdata

  SpokeProcessRTdata getSpokeProcessRTdata() const noexcept {
    return SpokeProcess_RTdata;
  }  // getSpokeProcessRTdata

  SPOKESTATE getSpokeState() const noexcept {
    return spoke_state;
  }  // getSpokeState

  TargetTrackerRTdata<max_num_target> getTargetTrackerRTdata() const noexcept {
    return TargetTracking_RTdata;
  }  // getTargetTrackerRTdata

  double getsampletime() const noexcept {
    return SpokeProcess_data.sample_time;
  }  // getsampletime

  void setClusteringdata(double _p_radius,
                         std::size_t _p_minumum_neighbors = 2) {
    Clustering_data.p_radius = _p_radius;
    Clustering_data.p_minumum_neighbors = _p_minumum_neighbors;
  }  // setClusteringdata

 private:
  const AlarmZone Alarm_Zone;
  const SpokeProcessdata SpokeProcess_data;
  ClusteringData Clustering_data;

  SPOKESTATE spoke_state;
  TargetTrackerRTdata<max_num_target> TargetTracking_RTdata;
  SpokeProcessRTdata SpokeProcess_RTdata;
  TargetDetectionRTdata TargetDetection_RTdata;

  // calculate the CPA and TCPA, relative to boat A
  std::tuple<double, double, double> computeCPA(
      const double boatA_position_x, const double boatA_position_y,
      const double boatA_speed_x, const double boatA_speed_y,
      const double boatB_position_x, const double boatB_position_y,
      const double boatB_speed_x, const double boatB_speed_y) {
    double initial_delta_x = boatA_position_x - boatB_position_x;
    double initial_delta_y = boatA_position_y - boatB_position_y;
    double delta_speed_x = boatA_speed_x - boatB_speed_x;
    double delta_speed_y = boatA_speed_y - boatB_speed_y;
    double initial_square_distance =
        initial_delta_x * initial_delta_x + initial_delta_y * initial_delta_y;
    double initial_square_rela_speed =
        delta_speed_x * delta_speed_x + delta_speed_y * delta_speed_y;

    /** empiricial value **/
    double max_search_time = std::sqrt(2 * initial_square_distance /
                                       (initial_square_rela_speed + 1));
    int max_search_index = 100;
    double search_time_step = max_search_time / max_search_index;
    // empiricial value

    double min_distance = initial_square_distance;
    double CPA_x = 0;
    double CPA_y = 0;
    double TCPA = 0;
    for (int i = 0; i != max_search_index; ++i) {
      double search_time = i * search_time_step;
      double distance = (search_time * delta_speed_x + initial_delta_x) *
                            (search_time * delta_speed_x + initial_delta_x) +
                        (search_time * delta_speed_y + initial_delta_y) *
                            (search_time * delta_speed_y + initial_delta_y);

      if (distance < min_distance) {
        min_distance = distance;
        TCPA = search_time;
        CPA_x = boatA_position_x + boatA_speed_x * search_time;
        CPA_y = boatA_position_y + boatA_speed_y * search_time;
      }
    }

    return {CPA_x, CPA_y, TCPA};
  }  // computeCPA

  // clustering for all points and find the miniball around each sets of points
  void ClusteringAndMiniBall(const std::vector<double> &_surroundings_x,
                             const std::vector<double> &_surroundings_y,
                             std::vector<double> &_target_x,
                             std::vector<double> &_target_y,
                             std::vector<double> &_target_radius) {
    // clustering for all points
    std::shared_ptr<pyclustering::dataset> p_data =
        std::make_shared<pyclustering::dataset>();
    std::shared_ptr<pyclustering::clst::dbscan_data> ptr_output_result =
        std::make_shared<pyclustering::clst::dbscan_data>();

    std::size_t num_surroundings = _surroundings_x.size();
    p_data->resize(num_surroundings);
    for (std::size_t i = 0; i != num_surroundings; ++i) {
      p_data->at(i) = {_surroundings_x[i], _surroundings_y[i]};
    }

    pyclustering::clst::dbscan clustering_solver(
        Clustering_data.p_radius, Clustering_data.p_minumum_neighbors);
    clustering_solver.process(*p_data, *ptr_output_result);
    const pyclustering::clst::cluster_sequence &actual_clusters =
        ptr_output_result->clusters();

    std::size_t num_actual_clusters = actual_clusters.size();

    // miniball for each cluster
    _target_x.resize(num_actual_clusters);
    _target_y.resize(num_actual_clusters);
    _target_radius.resize(num_actual_clusters);

    for (std::size_t index = 0; index != num_actual_clusters; ++index) {
      auto cluster = actual_clusters.at(index);

      int d = 2;                       // dimension
      std::size_t n = cluster.size();  // number of points

      double **ap = new double *[n];
      for (std::size_t j = 0; j < n; ++j) {
        double *p = new double[d];
        p[0] = _surroundings_x[cluster.at(j)];
        p[1] = _surroundings_y[cluster.at(j)];
        ap[j] = p;
      }
      // create an instance of Miniball
      Miniball::Miniball<
          Miniball::CoordAccessor<double *const *, const double *> >
          mb(d, ap, ap + n);

      // output results
      const double *center = mb.center();
      _target_x[index] = center[0];
      _target_y[index] = center[1];
      _target_radius[index] = mb.squared_radius();

      // clean up
      for (std::size_t j = 0; j < n; ++j) delete[] ap[j];
      delete[] ap;
    }

  }  // ClusteringAndMiniBall

  // motion prediction for radar-detected target (Staight line assumption)
  void PredictMotion(
      const std::vector<double> &new_target_x,
      const std::vector<double> &new_target_y,
      const std::vector<double> &new_target_radius,
      TargetTrackerRTdata<max_num_target> &_TargetTracking_RTdata) {
    // _TargetTracking_RTdata.;

  }  // PredictMotion

  // check if spoke azimuth is the alarm zone, depending on azimuth
  bool IsInAlarmAzimuth(const double _current_spoke_azimuth_rad) {
    if (std::abs(common::math::Normalizeheadingangle(
            _current_spoke_azimuth_rad - Alarm_Zone.center_bearing_rad)) <=
        0.5 * Alarm_Zone.width_bearing_rad)
      return true;
    return false;
  }  // IsInAlarmAzimuth

  // convert the body-fixed coordinate to marine
  void convert_surroundings_to_marine(
      const double _vessel_x_m, const double _vessel_y_m,
      const double _vessel_theta_rad,
      const std::vector<double> &surroundings_bearing_rad,
      const std::vector<double> &surroundings_range_m,
      std::vector<double> &surroundings_x_m,
      std::vector<double> &surroundings_y_m) {
    double cvalue = std::cos(_vessel_theta_rad);
    double svalue = std::sin(_vessel_theta_rad);

    std::size_t num_surroundings = surroundings_bearing_rad.size();

    surroundings_x_m.resize(num_surroundings);
    surroundings_y_m.resize(num_surroundings);

    for (std::size_t i = 0; i != num_surroundings; ++i) {
      double bearing_rad = surroundings_bearing_rad[i];
      double range_m = surroundings_range_m[i];
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
  }  // convert_surroundings_to_marine

  // find all surroundings on one spoke data, depending on the given threhold
  void find_surroundings_spoke(
      const uint8_t *_spoke_array, const std::size_t _array_size,
      const double _spoke_azimuth_rad, const double _samplerange_m,
      std::vector<double> &surroundings_InAlarm_bearing_rad,
      std::vector<double> &surroundings_InAlarm_range_m) {
    // find the index of all elements larger than threhold value
    auto surroundings_index = find_above_allelements_index<uint8_t>(
        _spoke_array, _array_size, Alarm_Zone.sensitivity_threhold);
    std::size_t num_surroundings = surroundings_index.size();

    // check if the surroundings are in the alarm zone
    surroundings_InAlarm_bearing_rad.clear();
    surroundings_InAlarm_range_m.clear();

    for (std::size_t i = 0; i != num_surroundings; ++i) {
      // TODO: test the empirical
      static double interception_empirical = 8.0;
      double s_range_m =
          interception_empirical + _samplerange_m * (surroundings_index[i] + 1);
      double s_bearing_rad = _spoke_azimuth_rad;

      if ((s_range_m <= Alarm_Zone.end_range_m) &&
          (Alarm_Zone.start_range_m <= s_range_m)) {
        surroundings_InAlarm_bearing_rad.emplace_back(s_bearing_rad);
        surroundings_InAlarm_range_m.emplace_back(s_range_m);
      }
    }

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

};  // end class TargetTracking

}  // namespace ASV::perception

#endif /* _TARGETTRACKING_H_ */