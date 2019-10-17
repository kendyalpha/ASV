/*
***********************************************************************
* testopenmp.cc:
* multiple threads using openmp
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <omp.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include <vector>
#include "common/timer/include/timecounter.h"

void testomp() {
  ASV::common::timecounter _timer;
  const int size = 2560000;
  std::vector<double> sinTable;
  sinTable.resize(size);
  double sum = 0.0;
#pragma omp parallel for num_threads(2)  // using two threads
  for (int n = 0; n < size; ++n) {
    std::vector<int> a(10000, 1);
    sinTable[n] = std::sin(2 * M_PI * n / size) * std::cos(M_PI * n + size) +
                  std::sin(M_PI * n / size);
  }
  // avoid thread race
  for (int n = 0; n < size; ++n) {
    sum += sinTable[n];
  }

  long int et = _timer.timeelapsed();
  std::cout << "It takes " << et << " to get results: " << sum << std::endl;
}

void withoutthread() {
  ASV::common::timecounter _timer;
  const int size = 2560000;
  std::vector<double> sinTable;
  sinTable.resize(size);
  double sum = 0.0;
  for (int n = 0; n < size; ++n) {
    std::vector<int> a(10000, 1);
    sinTable[n] = std::sin(2 * M_PI * n / size) * std::cos(M_PI * n + size) +
                  std::sin(M_PI * n / size);
    sum += sinTable[n];
  }

  long int et = _timer.timeelapsed();
  std::cout << "It takes " << et << " to get results: " << sum << std::endl;
}

int main() {
  std::thread t1(testomp);
  std::thread t2(withoutthread);
  t1.join();
  t2.join();
}