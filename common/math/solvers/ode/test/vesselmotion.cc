/*
***********************************************************************
* odesolver.cc:
* ordinary differential equation solver with support to Eigen
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <cmath>
#include "../include/odesolver.h"
#include "common/fileIO/include/utilityio.h"

using namespace boost::numeric::odeint;

struct vessel {
  vessel() {
    auto M_inv = M.inverse();
    A.block(2, 2, 3, 3) = -M_inv * D;
    B.block(2, 0, 3, 3) = M_inv;
  }

  template <typename State, typename Deriv>
  void operator()(const State& x, Deriv& dxdt, double t) const {
    Eigen::Matrix<double, 6, 1> _dx = A * x + B * u;

    for (int i = 0; i != 3; ++i) dxdt[i] = _dx[i];
  }

  void updateTransform(double _theta) {
    double cvalue = std::cos(_theta);
    double svalue = std::sin(_theta);
    Transform(0, 0) = cvalue;
    Transform(0, 1) = -svalue;
    Transform(1, 0) = svalue;
    Transform(1, 1) = cvalue;
    Transform(2, 2) = 1;
  }

  void updateA(double _theta) {
    updateTransform(_theta);
    A.block(0, 2, 3, 3) = Transform;
  }
  void updateu(const Eigen::Vector3d& _u) { u = _u; }

  Eigen::Matrix3d M =
      (Eigen::Matrix3d() << 100, 0, 0, 0, 100, 10, 10, 0, 200).finished();
  Eigen::Matrix3d D =
      (Eigen::Matrix3d() << 10, 0, 0, 0, 10, 0, 0, 0, 10).finished();

  Eigen::Matrix3d Transform = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();

  Eigen::Vector3d u = Eigen::Vector3d::Zero();
};

int main() {
  using state_type = Eigen::Matrix<double, 6, 1>;
  vessel sys;
  runge_kutta4<state_type, double, state_type, double, vector_space_algebra>
      rk4;
  state_type x = (state_type() << 0, 0, 1, 0, 0, 0).finished();
  Eigen::Matrix3d P =
      (Eigen::Matrix3d() << 10, 0, 0, 0, 10, 0, 0, 0, 10).finished();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();

  int total_step = 500;
  Eigen::MatrixXd save_x(total_step, 6);

  for (int i = 0; i != total_step; ++i) {
    sys.updateA(x(2));
    u = -P * x.head(3);
    sys.updateu(u);
    rk4.do_step(sys, x, 0.0, 0.1);
    save_x.row(i) = x.transpose();
  }
  ASV::write2csvfile("../data/x.csv", save_x);
}
