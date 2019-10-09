//
// This file is part of Easylogging++ samples
//
// Revision 1.0
//

#include <thread>
#include "../include/easylogging++.h"

class testlog {
 public:
  testlog() {}
  ~testlog() {}

  void def() {
    for (int i = 0; i < 1000; ++i) {
      CLOG(INFO, "first") << "This is from first " << i;
    }
    CLOG(ERROR, "first") << "This is info log using performance logger";
  }
  void second() {
    for (int i = 0; i < 1000; ++i)
      CLOG(INFO, "second") << "This is from second" << i;
  }
};

void def() {
  testlog _testlog;
  _testlog.def();
}

void second() {
  testlog _testlog;
  _testlog.second();
}

int main(int, char**) {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  // el::Configurations confFromFile(
  //     "/home/scar1et/Coding/ASV/logging/default-logger.conf");

  // el::Loggers::setDefaultConfigurations(confFromFile, true);

  LOG(INFO) << "The program has started!";

  std::thread t1(def);
  std::thread t2(second);

  t1.join();
  t2.join();

  LOG(INFO) << "Shutting down.";
  return 0;
}
