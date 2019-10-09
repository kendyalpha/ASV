/*
 [auto_generated]
 integrate.cpp

 [begin_description]
 tba.
 [end_description]

 Copyright 2009-2012 Karsten Ahnert
 Copyright 2009-2012 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */

#ifdef BOOST_MSVC
#pragma warning(disable : 4996)
#endif

#include "../include/odesolver.h"

struct lorenz {
  template <typename State, typename Deriv, typename Time>
  void operator()(const State& x, Deriv& dxdt, const Time& t) const {
    const Time sigma = 10.0;
    const Time R = 28.0;
    const Time b = 8.0 / 3.0;
    dxdt[0] = sigma * (x[1] - x[0]);
    dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
    dxdt[2] = -b * x[2] + x[0] * x[1];
  }
};

int main() {
  typedef Eigen::Matrix<double, 3, 1> state_type;
  state_type x;
  x[0] = 10.0;
  x[1] = 10.0;
  x[2] = 10.0;
  double t_start = 0.0, t_end = 1000.0, dt = 0.1;
  boost::numeric::odeint::integrate<double>(lorenz(), x, t_start, t_end, dt);

  std::vector<double> x2(3);
  x2[0] = 10.0;
  x2[1] = 10.0;
  x2[2] = 10.0;
  boost::numeric::odeint::integrate(lorenz(), x2, t_start, t_end, dt);

  // BOOST_CHECK_CLOSE(x[0], x2[0], 1.0e-13);
  // BOOST_CHECK_CLOSE(x[1], x2[1], 1.0e-13);
  // BOOST_CHECK_CLOSE(x[2], x2[2], 1.0e-13);
}
