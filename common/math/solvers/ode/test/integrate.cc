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

#include <boost/config.hpp>
#ifdef BOOST_MSVC
#pragma warning(disable : 4996)
#endif

#define BOOST_TEST_MODULE odeint_eigen_runge_kutta4

#include <boost/test/unit_test.hpp>

#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen_resize.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>

using namespace boost::unit_test;
using namespace boost::numeric::odeint;

struct sys {
  template <class State, class Deriv>
  void operator()(const State &x, Deriv &dxdt, double t) const {
    dxdt[0] = 1.0;
  }
};

int main() {
  typedef Eigen::Matrix<double, 1, 1> state_type;
  state_type x;
  x[0] = 10.0;
  runge_kutta4<state_type, double, state_type, double, vector_space_algebra>
      rk4;
  rk4.do_step(sys(), x, 0.0, 0.1);
}
