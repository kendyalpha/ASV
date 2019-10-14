/*
***********************************************************************
* dubins_path.h:
* A dubins path class for finding analytical solutions to the problem of
* the shortest path.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DUBINS_PATH_H_
#define _DUBINS_PATH_H_

#include <cmath>
#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::planning {

class dubins_path {
 public:
  enum class DubinsPathType {
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
  };

  struct DubinsPath {
    /* the initial configuration */
    double qi[3];
    /* the lengths of the three segments */
    double param[3];
    /* model forward velocity / model angular velocity */
    double rho;
    /* the path type described */
    DubinsPathType type;
  };

  struct DubinsIntermediateResults {
    double alpha;  // alpha
    double beta;   // beta
    double d;      // d
    double sa;     // sin (alpha)
    double sb;     // sin (beta)
    double ca;     // cos (alpha)
    double cb;     // cos (beta)
    double c_ab;   // cos(alpha-beta)
    double d_sq;   // d*d
  };

  enum class SegmentType {
    L_SEG = 0,  // Left
    S_SEG = 1,  // Straight
    R_SEG = 2   // Right
  };

 public:
  dubins_path() {}

 private:
  // No error
  const int EDUBOK = 0;
  // Colocated configurations
  const int EDUBCOCONFIGS = 1;
  // Path parameterisitation error
  const int EDUBPARAM = 2;
  // the rho value is invalid
  const int EDUBBADRHO = 3;
  // no connection between configurations with this word
  const int EDUBNOPATH = 4;
  //
  const double EPSILON = 10e-10;
  /* The segment types for each of the Path types */
  const SegmentType DIRDATA[][3] = {
      {L_SEG, S_SEG, L_SEG}, {L_SEG, S_SEG, R_SEG}, {R_SEG, S_SEG, L_SEG},
      {R_SEG, S_SEG, R_SEG}, {R_SEG, L_SEG, R_SEG}, {L_SEG, R_SEG, L_SEG}};

  int dubins_LSL(const DubinsIntermediateResults& in, double* out) {
    double p_sq = 2 + in.d_sq - 2 * in.c_ab + 2 * in.d * (in.sa - in.sb);
    double tmp0 = in.d + in.sa - in.sb;
    double tmp1 = 0.0;
    if (p_sq >= 0) {
      tmp1 = std::atan2(in.cb - in.ca, tmp0);
      out[0] = common::math::Normalizeheadingangle(tmp1 - in.alpha);
      out[1] = std::sqrt(p_sq);
      out[2] = common::math::Normalizeheadingangle(in.beta - tmp1);
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_LSL

  int dubins_RSR(const DubinsIntermediateResults& in, double* out) {
    double tmp0 = in.d - in.sa + in.sb;
    double p_sq = 2 + in.d_sq - 2 * in.c_ab + 2 * in.d * (in.sb - in.sa);
    if (p_sq >= 0) {
      double tmp1 = std::atan2(in.ca - in.cb, tmp0);
      out[0] = common::math::Normalizeheadingangle(in.alpha - tmp1);
      out[1] = std::sqrt(p_sq);
      out[2] = common::math::Normalizeheadingangle(tmp1 - in.beta);
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_RSR

  int dubins_LSR(const DubinsIntermediateResults& in, double* out) {
    double p_sq = -2 + (in.d_sq) + (2 * in.c_ab) + (2 * in.d * (in.sa + in.sb));
    if (p_sq >= 0) {
      double p = std::sqrt(p_sq);
      double tmp0 = std::atan2((-in.ca - in.cb), (in.d + in.sa + in.sb)) -
                    std::atan2(-2.0, p);
      out[0] = common::math::Normalizeheadingangle(tmp0 - in.alpha);
      out[1] = p;
      // out[2] = common::math::Normalizeheadingangle(tmp0 - mod2pi(in.beta));
      out[2] = common::math::Normalizeheadingangle(tmp0 - in.beta);
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_LSR

  int dubins_RSL(const DubinsIntermediateResults& in, double* out) {
    double p_sq = -2 + in.d_sq + (2 * in.c_ab) - (2 * in.d * (in.sa + in.sb));
    if (p_sq >= 0) {
      double p = std::sqrt(p_sq);
      double tmp0 = std::atan2((in.ca + in.cb), (in.d - in.sa - in.sb)) -
                    std::atan2(2.0, p);
      out[0] = common::math::Normalizeheadingangle(in.alpha - tmp0);
      out[1] = p;
      out[2] = common::math::Normalizeheadingangle(in.beta - tmp0);
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_RSL

  int dubins_RLR(const DubinsIntermediateResults& in, double* out) {
    double tmp0 =
        (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sa - in.sb)) / 8.;
    double phi = std::atan2(in.ca - in.cb, in.d - in.sa + in.sb);
    if (std::abs(tmp0) <= 1) {
      double p =
          common::math::Normalizeheadingangle((2 * M_PI) - std::acos(tmp0));
      double t =
          common::math::Normalizeheadingangle(in->alpha - phi + mod2pi(p / 2.));
      out[0] = t;
      out[1] = p;
      out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_RLR

  int dubins_LRL(const DubinsIntermediateResults& in, double* out) {
    double tmp0 =
        (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sb - in.sa)) / 8.;
    double phi = std::atan2(in.ca - in.cb, in.d + in.sa - in.sb);
    if (std::abs(tmp0) <= 1) {
      double p = mod2pi(2 * M_PI - acos(tmp0));
      double t = mod2pi(-in->alpha - phi + p / 2.);
      out[0] = t;
      out[1] = p;
      out[2] = mod2pi(mod2pi(in->beta) - in->alpha - t + mod2pi(p));
      return EDUBOK;
    }
    return EDUBNOPATH;
  }  // dubins_LRL
};

}  // namespace ASV::planning

#endif /* _DUBINS_PATH_H_ */