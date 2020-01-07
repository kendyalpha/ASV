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
#include "TargetTrackingData.h"
#include "common/math/Geometry/include/Miniball.hpp"
#include "common/math/miscellaneous/include/math_utils.h"

#include "common/timer/include/timecounter.h"

namespace ASV::perception {
class TargetTracking {
 public:
  TargetTracking(const AlphaBetaData &_AlphaBeta_data,
                 const ClusteringData &_ClusteringData,
                 const AlarmZone &_AlarmZone,
                 const SpokeProcessdata &_SpokeProcessdata)
      : AlphaBeta_data(_AlphaBeta_data),
        Clustering_data(_ClusteringData),
        Alarm_Zone(_AlarmZone),
        SpokeProcess_data(_SpokeProcessdata) {}
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

      } else {                            // outside the alarm azimuth
        if (previous_IsInAlarmAzimuth) {  // leaving the alarm azimuth
          // start to cluster and miniball
          ClusteringAndMiniBall(SpokeProcess_RTdata.surroundings_x_m,
                                SpokeProcess_RTdata.surroundings_y_m,
                                TargetTracker_RTdata.target_x,
                                TargetTracker_RTdata.target_y,
                                TargetTracker_RTdata.target_square_radius);

          SpokeProcess_RTdata.surroundings_bearing_rad.clear();
          SpokeProcess_RTdata.surroundings_range_m.clear();
          SpokeProcess_RTdata.surroundings_x_m.clear();
          SpokeProcess_RTdata.surroundings_y_m.clear();
        }
      }
    }

    previous_spoke_azimuth_rad = _spoke_azimuth_rad;
    previous_IsInAlarmAzimuth = IsInAlarmAzimuth(previous_spoke_azimuth_rad);

    return *this;
  }  // AutoTracking

  TargetTracking &TestClustering(const std::vector<double> &_surroundings_x,
                                 const std::vector<double> &_surroundings_y) {
    ClusteringAndMiniBall(_surroundings_x, _surroundings_y,
                          TargetTracker_RTdata.target_x,
                          TargetTracker_RTdata.target_y,
                          TargetTracker_RTdata.target_square_radius);

    return *this;
  }  // AutoTracking

  TargetTrackerRTdata getTargetTrackerRTdata() const noexcept {
    return TargetTracker_RTdata;
  }

  SpokeProcessRTdata getSpokeProcessRTdata() const noexcept {
    return SpokeProcess_RTdata;
  }  // getSpokeProcessRTdata

  double getsampletime() const noexcept {
    return SpokeProcess_data.sample_time;
  }

 private:
  const AlphaBetaData AlphaBeta_data;
  const ClusteringData Clustering_data;
  const AlarmZone Alarm_Zone;
  const SpokeProcessdata SpokeProcess_data;
  SpokeProcessRTdata SpokeProcess_RTdata;
  TargetTrackerRTdata TargetTracker_RTdata;

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

  // alpha-beta filtering for 2d target tracking
  std::tuple<double, double, double, double> AlphaBetaFiltering(
      const double previous_target_x, const double previous_target_y,
      const double previous_target_vx, const double previous_target_vy,
      const double meas_x, const double meas_y) {
    double previous_position[2] = {previous_target_x, previous_target_y};
    double previous_velocity[2] = {previous_target_vx, previous_target_vy};
    double measurement[2] = {meas_x, meas_y};
    double position[2] = {};
    double velocity[2] = {};

    for (int i = 0; i != 2; ++i) {
      double xk = previous_position[i] +
                  AlphaBeta_data.sample_time * previous_velocity[i];
      double rk = measurement[i] - xk;
      position[i] = xk + AlphaBeta_data.alpha * rk;
      velocity[i] = previous_velocity[i] +
                    rk * AlphaBeta_data.beta / AlphaBeta_data.sample_time;
    }
    return {position[0], position[1], velocity[0], velocity[1]};
  }  // AlphaBetaFiltering

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