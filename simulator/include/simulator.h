/*
***********************************************************************
* simulator.h:
* 3DoF motion simulator for USV
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <cmath>
#include "common/math/miscellaneous/include/math_utils.h"
#include "common/math/solvers/ode/include/odesolver.h"
#include "simulatordata.h"

namespace ASV::simulation {

// vessel motion used in simulation
struct vessel_simulator {
  // constructor
  vessel_simulator(const common::vessel& _vessel)
      : M(_vessel.AddedMass + _vessel.Mass),
        D(_vessel.LinearDamping),
        A(Eigen::Matrix<double, 6, 6>::Zero()),
        B(Eigen::Matrix<double, 6, 3>::Zero()),
        u(Eigen::Vector3d::Zero()) {
    initializeAB();
  }

  template <typename State, typename Deriv>
  void operator()(const State& x, Deriv& dxdt, double t) const {
    Eigen::Matrix<double, 6, 1> _dx = A * x + B * u;
    for (int i = 0; i != 6; ++i) dxdt[i] = _dx[i];
    t = t;  // disable warning!
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

  // update the input
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
  Eigen::Vector3d u;  // input defined in the body-fixed coordinate frame
};

class simulator {
  using state_type = Eigen::Matrix<double, 6, 1>;

 public:
  simulator(double _sample_time, const common::vessel& _vessel,
            const state_type& _x = state_type::Zero())
      : sample_time(_sample_time),
        sys(_vessel),
        simulator_rtdata({common::STATETOGGLE::READY, _x}) {}
  virtual ~simulator() = default;

  simulator& simulator_onestep(
      double _desired_heading, const Eigen::Vector3d& _thrust,
      const Eigen::Vector3d& _seaload = Eigen::Vector3d::Zero()) {
    double _theta = 0.0;
    if (std::abs(common::math::Normalizeheadingangle(
            simulator_rtdata.X(2) - _desired_heading)) < M_PI / 36) {
      // use the fixed setpoint orientation to prevent measurement noise
      _theta = _desired_heading;
    } else {
      // if larger than 5 deg, we use the realtime orientation
      _theta = simulator_rtdata.X(2);
    }

    sys.updateA(_theta);              // update the transform matrix and A
    sys.updateu(_thrust + _seaload);  // update the input
    rk4.do_step(sys, simulator_rtdata.X, 0.0, sample_time);
    return *this;
  }

  void setX(const state_type& _x) { simulator_rtdata.X = _x; }
  state_type getX() const noexcept { return simulator_rtdata.X; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  double sample_time;  // second
  vessel_simulator sys;
  simulatorRTdata simulator_rtdata;

  // 4-order Runge-Kutta methods
  boost::numeric::odeint::runge_kutta4<
      state_type, double, state_type, double,
      boost::numeric::odeint::vector_space_algebra>
      rk4;
};

}  // namespace ASV::simulation

#endif /* _SIMULATOR_H_ */