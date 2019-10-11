/*
***********************************************************************
* odesolver.cc:
* ordinary differential equation solver with support to Eigen
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <cmath>
#include "common/math/solvers/ode/include/odesolver.h"
#include "common/property/include/vesseldata.h"
namespace ASV {

struct vessel_simulator {
  // constructor
  vessel_simulator(const vessel& _vessel)
      : M(_vessel.AddedMass + _vessel.Mass),
        D(_vessel.Damping),
        A(Eigen::Matrix<double, 6, 6>::Zero()),
        B(Eigen::Matrix<double, 6, 3>::Zero()),
        u(Eigen::Vector3d::Zero()) {
    initializeAB();
  }

  template <typename State, typename Deriv>
  void operator()(const State& x, Deriv& dxdt, double t) const {
    Eigen::Matrix<double, 6, 1> _dx = A * x + B * u;
    for (int i = 0; i != 6; ++i) dxdt[i] = _dx[i];
  }
  // update the A matrix in state space model
  void updateA(double _theta) {
    // update the transformation matrix
    double cvalue = std::cos(_theta);
    double svalue = std::sin(_theta);
    Eigen::Matrix3d Transform = Eigen::Matrix3d::Identity();
    Transform(0, 0) = cvalue;
    Transform(0, 1) = -svalue;
    Transform(1, 0) = svalue;
    Transform(1, 1) = cvalue;

    //
    A.block(0, 3, 3, 3) = Transform;
  }

  // update the thrust
  void updateu(const Eigen::Vector3d& _u) { u = _u; }

  //
  void initializeAB() {
    auto M_inv = M.inverse();
    A.block(3, 3, 3, 3) = -M_inv * D;
    B.block(3, 0, 3, 3) = M_inv;
  }

  Eigen::Matrix3d M;
  Eigen::Matrix3d D;
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 3> B;
  Eigen::Vector3d u;
};

class simulator {
  using namespace boost::numeric::odeint;
  using state_type = Eigen::Matrix<double, 6, 1>;

 public:
  simulator() {}
  virtual ~simulator() = default;

  simulator_onestep(const Eigen::Vector3d& _u) {
    sys.updateA(x(2));
    sys.updateu(_u);
    rk4.do_step(sys, x, 0.0, 0.1);
  }

  state_type getX() const noexcept { return x; }

 private:
  vessel_simulator sys;
  runge_kutta4<state_type, double, state_type, double, vector_space_algebra>
      rk4;

  state_type x;
  double sample_time;
};

}  // namespace ASV

int main() {
  state_type x = (state_type() << 0, 1, 0.1, 0, 0, 0).finished();
  Eigen::Matrix3d P =
      (Eigen::Matrix3d() << 10, 0, 0, 0, 10, 0, 0, 0, 100).finished();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();

  int total_step = 5000;
  Eigen::MatrixXd save_x(total_step, 6);

  for (int i = 0; i != total_step; ++i) {
    sys.updateA(x(2));
    u = -P * x.head(3);
    sys.updateu(u);
    rk4.do_step(sys, x, 0.0, 0.1);
    std::cout << x << std::endl;
    save_x.row(i) = x.transpose();
  }
  ASV::write2csvfile("../data/x.csv", save_x);
}

#endif /* _SIMULATOR_H_ */