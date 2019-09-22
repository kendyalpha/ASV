
/*
***********************************************************************
* math_utils.h: Math-related util functions
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include <cmath>

namespace ASV {

// restrict heading angle or delta heading to (-PI ~ PI)
// compute the delta heading to find the shortest way to rotate
double Normalizeheadingangle(double _heading) noexcept {
  double a = std::fmod(_heading + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}  // Normalizeheadingangle

}  // end namespace ASV

#endif /* _MATH_UTILS_H_ */