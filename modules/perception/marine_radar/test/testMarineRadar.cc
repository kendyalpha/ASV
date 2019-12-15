/*
****************************************************************************
* TestMarineRadar.cc:
* Marine radar for target tracking
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include "../include/MarineRadar.h"

using namespace ASV::perception;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  MarineRadar _MarineRadar;

  _MarineRadar.testMarineRadar();
}