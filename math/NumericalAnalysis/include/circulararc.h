/*
 * circulararc.h
 *
 * A segment of a circle
 *
 * by Hu.ZH(CrossOcean.ai)
 *
 */

#ifndef _CIRCULARARC_H_
#define _CIRCULARARC_H_

#include <cmath>

class circulararc {
 public:
  circulararc(double _center_x = 0, double _center_y = 0, double _radius = 1)
      : center_x(_center_x), center_y(_center_y), radius(_radius) {}
  ~circulararc() = default;

  void compute(double _angle, double &_x, double &_y) {
    _x = radius * std::cos(_angle) + center_x;
    _y = radius * std::sin(_angle) + center_y;
  }

 private:
  double center_x;
  double center_y;
  double radius;
};

#endif /*_CIRCULARARC_H_*/