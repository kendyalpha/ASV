#include <cmath>
#include <iostream>

double Normalizeheadingangle(double _heading) noexcept {
  double a = std::fmod(_heading + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

int main() { std::cout << Normalizeheadingangle(3.15) << std::endl; }