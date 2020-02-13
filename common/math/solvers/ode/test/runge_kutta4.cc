/*
 [auto_generated]
 libs/numeric/odeint/test_external/eigen/runge_kutta4.cpp

 [begin_description]
 tba.
 [end_description]

 Copyright 2013 Karsten Ahnert
 Copyright 2013 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */

#ifdef BOOST_MSVC
#pragma warning(disable : 4996)
#endif

#include <iostream>
#include "../include/odesolver.h"
#include "common/plotting/include/gnuplot-iostream.h"

using namespace boost::numeric::odeint;

struct sys {
  template <class State, class Deriv>
  void operator()(const State &x, Deriv &dxdt, double t) const {
    dxdt[0] = -K / M * x[1] + T / M;
    dxdt[1] = x[0];
    t = t;
  }

  void setK(double _K) { K = _K; }
  void setT(double _T) { T = _T; }

  double K = 1;
  double M = 1;
  double T = 0;
};

int main() {
  typedef Eigen::Matrix<double, 2, 1> state_type;
  state_type x;
  sys mysys;
  x[0] = 0.0;
  x[1] = 3.0;

  runge_kutta4<state_type, double, state_type, double, vector_space_algebra>
      rk4;

  int total_step = 100;
  std::vector<std::pair<double, double> > xy_pts_A;

  for (int i = 0; i != total_step; ++i) {
    mysys.setT(1);
    rk4.do_step(mysys, x, 0.0, 0.1);
    xy_pts_A.push_back(std::make_pair(i, x(1)));
  }

  Gnuplot gp;
  gp << "set title 'ODE'\n";
  gp << "set xlabel 'sampling instant'\n";
  gp << "set ylabel 'x'\n";
  gp << "plot"
        " '-' with lines lw 2\n";
  gp.send1d(xy_pts_A);
}
