/*
****************************************************************************
* TargetTracking.h:
* Target tracking using marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _TARGETTRACKING_H_
#define _TARGETTRACKING_H_

#include "SpokeProcessingData.h"
#include "common/math/Geometry/include/Miniball.hpp"

namespace ASV::perception {
class TargetTracking {
 public:
  TargetTracking(const AlphaBetaData &_AlphaBeta_data)
      : AlphaBeta_data(_AlphaBeta_data) {}
  virtual ~TargetTracking() = default;

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

 private:
  const AlphaBetaData AlphaBeta_data;
};  // namespace ASV::perception

}  // namespace ASV::perception

#endif /* _TARGETTRACKING_H_ */