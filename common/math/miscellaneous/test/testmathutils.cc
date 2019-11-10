/*
***********************************************************************
* testmathutils.h: Test math utility function
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <boost/test/included/unit_test.hpp>
#include <iostream>
#include "../include/math_utils.h"

using namespace ASV::common::math;

BOOST_AUTO_TEST_CASE(NormalizeAngle) {
  BOOST_CHECK_CLOSE(Normalizeheadingangle(3.15), -3.13318, 1e-3);
  BOOST_CHECK_CLOSE(Normalizeheadingangle(3.14), 3.14, 1e-7);
  BOOST_CHECK_CLOSE(Normalizeheadingangle(-3.14), -3.14, 1e-7);
  BOOST_CHECK_CLOSE(Normalizeheadingangle(-2 * M_PI), 0, 1e-7);
  BOOST_CHECK_CLOSE(Normalizeheadingangle(5 * M_PI), -M_PI, 1e-7);
}

BOOST_AUTO_TEST_CASE(Cartesian2Pol) {
  double x = 1;
  double y = 3;
  auto [r, theta] = Cartesian2Polar(x, y);

  BOOST_CHECK_CLOSE(r, 3.16228, 1e-4);
  BOOST_CHECK_CLOSE(Rad2Degree(theta), 71.5651, 1e-4);
}
