#include <omp.h>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <vector>
#include "common/timer/include/timecounter.h"
using namespace std;

const int thread_used = 4;

class sumup {
 public:
  sumup& test() {
    for (int i = 0; i != 100000000; ++i) sum += i;
    return *this;
  }
  int getsum() { return sum; }

 private:
  int sum = 0;
};

void runwithoutthread() {
  ASV::common::timecounter _timer;

  sumup _sumup();
  int sum = _sumup.test.getsum();

  long int et = _timer.timeelapsed();
  std::cout << "without thread: " << et << std::endl;
  std::cout << "results: " << sum << std::endl;
}

void runompthread() {
  ASV::common::timecounter _timer;

  std::thread t1(sumup., 0, 100000000 / 2);
  std::thread t2(sumup, 100000000 / 2, 100000000);

  t1.join();
  t2.join();

  long int et = _timer.timeelapsed();
  std::cout << "omp: " << et << std::endl;
  std::cout << "results: " << sum << std::endl;
}

int main() {
  // runwithoutthread();
  runompthread();

  return 0;
}