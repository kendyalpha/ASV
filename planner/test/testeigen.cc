#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

int main() {
  Eigen::VectorXd a(5);
  a << -1, 2, 3, 4, 5;
  std::cout << a.minCoeff() << std::endl;
}