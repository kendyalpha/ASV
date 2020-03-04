/*
*******************************************************************************
* testIMU.cc:
* unit test to receive data from IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <thread>
#include "../include/imu.h"
#include "common/timer/include/timecounter.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  ASV::messages::imu _imu(50);
  if (_imu.initializeIMU() >= 0) {
    ASV::common::timecounter _timer;

    while (1) {
      auto rtdata = _imu.receivedata().getimuRTdata();
      long int et = _timer.timeelapsed();
      std::cout << "sampling time: " << et << std::endl;
      std::cout << "status:" << rtdata.status << std::endl;
      std::cout << "Acc X:" << rtdata.acc_X << std::endl;
      std::cout << "Acc Y:" << rtdata.acc_Y << std::endl;
      std::cout << "Acc Z:" << rtdata.acc_Z << std::endl;
      std::cout << "Ang_vel_X:" << rtdata.Ang_vel_X << std::endl;
      std::cout << "Ang_vel_Y:" << rtdata.Ang_vel_Y << std::endl;
      std::cout << "Ang_vel_Z:" << rtdata.Ang_vel_Z << std::endl;
      std::cout << "Roll:" << rtdata.roll << std::endl;
      std::cout << "Pitch:" << rtdata.pitch << std::endl;
      std::cout << "Yaw:" << rtdata.yaw << std::endl;
    }
  }
  _imu.releaseIMU();
}