/*
***********************************************************************
* planner_util.h:
* utility function for planner
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _PLANNER_UTIL_H_
#define _PLANNER_UTIL_H_

#include "spline.h"

namespace ASV {

class quintic_polynomial final : public polynomialvalue<5> {
 public:
  quintic_polynomial(const Eigen::Matrix<double, 6, 1> &_a =
                         Eigen::Matrix<double, 6, 1>::Zero())
      : polynomialvalue(_a),
        start_x(0),
        start_vx(0),
        start_ax(0),
        end_x(0),
        end_vx(0),
        end_ax(0),
        T(0),
        x(Eigen::Vector3d::Zero()),
        b(Eigen::Vector3d::Zero()),
        A(Eigen::Matrix3d::Zero()) {}

  void update_startendposition(double _start_x, double _start_vx,
                               double _start_ax, double _end_x, double _end_vx,
                               double _end_ax, double _T) {
    start_x = _start_x;
    start_vx = _start_vx;
    start_ax = _start_ax;
    end_x = _end_x;
    end_vx = _end_vx;
    end_ax = _end_ax;
    T = _T;
    updatecoefficients();
  }  // update_startendposition

 private:
  double start_x;
  double start_vx;
  double start_ax;
  double end_x;
  double end_vx;
  double end_ax;
  double T;

  Eigen::Vector3d x;
  Eigen::Vector3d b;
  Eigen::Matrix3d A;

  void updatecoefficients() {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    A << T3, T4, T5,                                            // first row
        3 * T2, 4 * T3, 5 * T4,                                 // second row
        6 * T, 12 * T2, 20 * T3;                                // third row
    b << end_x - start_x - start_vx * T - 0.5 * start_ax * T2,  // first row
        end_vx - start_vx - start_ax * T,                       // second row
        end_ax - start_ax;                                      // third row
    x = A.householderQr().solve(b);
    a(0) = x(2);
    a(1) = x(1);
    a(2) = x(0);
    a(3) = 0.5 * start_ax;
    a(4) = start_vx;
    a(5) = start_x;
  }  // updatecoefficients

};  // quintic_polynomial

class quartic_polynomial final : public polynomialvalue<4> {
 public:
  quartic_polynomial(const Eigen::Matrix<double, 5, 1> &_a =
                         Eigen::Matrix<double, 5, 1>::Zero())
      : polynomialvalue(_a),
        start_x(0),
        start_vx(0),
        start_ax(0),
        end_vx(0),
        end_ax(0),
        T(0),
        x(Eigen::Vector2d::Zero()),
        b(Eigen::Vector2d::Zero()),
        A(Eigen::Matrix2d::Zero()) {}

  void update_startendposition(double _start_x, double _start_vx,
                               double _start_ax, double _end_vx, double _end_ax,
                               double _T) {
    start_x = _start_x;
    start_vx = _start_vx;
    start_ax = _start_ax;
    end_vx = _end_vx;
    end_ax = _end_ax;
    T = _T;
    updatecoefficients();
  }  // update_startendposition

 private:
  double start_x;
  double start_vx;
  double start_ax;
  double end_vx;
  double end_ax;
  double T;

  Eigen::Vector2d x;
  Eigen::Vector2d b;
  Eigen::Matrix2d A;

  void updatecoefficients() {
    double T2 = T * T;
    double T3 = T2 * T;
    A << 3 * T2, 4 * T3,                    // first row
        6 * T, 12 * T2;                     // second row
    b << end_vx - start_vx - start_ax * T,  // first row
        end_ax - start_ax;                  // third row
    x = A.householderQr().solve(b);
    a(0) = x(1);
    a(1) = x(0);
    a(2) = 0.5 * start_ax;
    a(3) = start_vx;
    a(4) = start_x;
  }  // updatecoefficients

};  // quartic_polynomial

}  // end namespace ASV

#endif /* _PLANNER_UTIL_H_ */