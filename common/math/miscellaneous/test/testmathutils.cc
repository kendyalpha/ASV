#include <iostream>
#include "../include/math_utils.h"

using namespace ASV::common::math;

void testNormalizeAngle() {
  std::cout << "Result of 3.15 is " << Normalizeheadingangle(3.15) << std::endl;
  std::cout << "Result of 3.14 is " << Normalizeheadingangle(3.14) << std::endl;
  std::cout << "Result of -3.14 is " << Normalizeheadingangle(-3.14)
            << std::endl;
  std::cout << "Result of -2Pi is " << Normalizeheadingangle(-2 * M_PI)
            << std::endl;
  std::cout << "Result of 5Pi is " << Normalizeheadingangle(5 * M_PI)
            << std::endl;
}

void testCartesian2Polar() {
  double x = 1;
  double y = 3;
  auto [r, theta] = Cartesian2Polar(x, y);
  std::cout << "r: " << r << std::endl;
  std::cout << "theta: " << Rad2Degree(theta) << std::endl;
}

int main() { testCartesian2Polar(); }