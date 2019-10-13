/*
*****************************************************************************
* testtimer.cc:
* unit test for timer in milliseconds
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "../include/timecounter.h"

int main() {
  using namespace ASV::common;
  timecounter _timer;
  for (int i = 0; i != 100; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    long int et = _timer.timeelapsed();
    std::cout << _timer.getUTCtime() << " " << et << std::endl;
  }
}
