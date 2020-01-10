/*
****************************************************************************
* RadarFiltering.h:
* Filtering for the radar tracking
* targets
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _RADARFILTERING_H_
#define _RADARFILTERING_H_

#include <tuple>
#include "TargetTrackingData.h"

namespace ASV::perception {

class RadarFiltering {
 public:
  RadarFiltering(const AlphaBetaData &_AlphaBeta_Data)
      : AlphaBeta_data(_AlphaBeta_Data) {}
  virtual ~RadarFiltering() = default;

  void testFilter() {}
  void setSampleTime(double _sample_time) { sample_time = _sample_time; }

 private:
  double alpha;
  double beta;

  // alpha-beta filtering for target tracking
  std::tuple<double, double> AlphaBetaFiltering(const double previous_x,
                                                const double previous_v,
                                                const double meas,
                                                const double sample_time) {
    double x_bar = previous_x + sample_time * previous_v;
    double rk = meas - x_bar;
    double new_x = x_bar + rk * AlphaBeta_data.alpha;
    double new_v = previous_v + rk * AlphaBeta_data.beta / sample_time;

    return {new_x, new_v};
  }  // AlphaBetaFiltering

  std::tuple<double, double> VaryingAlphaBeta(const int K,
                                              const double sample_time) {
    double mdivide = (K + 1) * (K + 2);
    double alpha = (4 * K + 2) / mdivide;
    double beta = 6 / (sample_time * mdivide);
    return {alpha, beta};
  }  // VaryingAlphaBeta
};   // end class RadarFiltering
}  // namespace ASV::perception
#endif /* _RADARFILTERING_H_ */