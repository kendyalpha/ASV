/*
*******************************************************************************
* run.cc:
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <cstdio>
#include "../include/threadloop.h"

using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  try {
    threadloop _threadloop;
    _threadloop.mainloop();

  } catch (const std::exception& e) {
    LOG(INFO) << "Shutting down.";
    std::cerr << "ERROR: std::logic_error" << e.what() << std::endl;
  } catch (int i) {
    LOG(INFO) << " stop signal  ";
    LOG(INFO) << "Shutting down.";
    // std::terminate();

  } catch (...) {
    // ensure destuctors of auto objects are called

    LOG(INFO) << "Shutting down.";
    std::cerr << "ERROR: std::logic_error\n";
  }
  return 0;
}

// 思源湖号，饮水思源，爱国荣校，蛤蛤蛤 +1