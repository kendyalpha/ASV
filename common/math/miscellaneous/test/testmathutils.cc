#include <iostream>
#include "../include/math_utils.h"
int main() {
  std::cout << "Result of 3.15 is " << ASV::Normalizeheadingangle(3.15)
            << std::endl;
  std::cout << "Result of 3.14 is " << ASV::Normalizeheadingangle(3.14)
            << std::endl;
  std::cout << "Result of -3.14 is " << ASV::Normalizeheadingangle(-3.14)
            << std::endl;
  std::cout << "Result of -2Pi is " << ASV::Normalizeheadingangle(-2 * M_PI)
            << std::endl;
  std::cout << "Result of 5Pi is " << ASV::Normalizeheadingangle(5 * M_PI)
            << std::endl;
}