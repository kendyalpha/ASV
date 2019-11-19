/*
*****************************************************************************
* testguilink.cc:
* unit test for gui communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include "../include/guilink.h"

using namespace ASV;

void test_gui_serial() {
  constexpr int num_thruster = 6;
  constexpr int dim_controlspace = 3;
  constexpr int num_battery = 3;

  // real time gui-link data
  messages::guilinkRTdata<num_thruster, num_battery> _guilinkRTdata{
      "",                                           // UTC_time
      messages::GUISTATUS::STANDBY,                 // guistutus_PC2gui
      messages::GUISTATUS::STANDBY,                 // guistutus_gui2PC
      common::LINKSTATUS::DISCONNECTED,             // linkstatus
      0,                                            // indicator_autocontrolmode
      0,                                            // indicator_windstatus
      0.0,                                          // latitude
      0.0,                                          // longitude
      Eigen::Matrix<double, 6, 1>::Zero(),          // State
      0.0,                                          // roll
      0.0,                                          // pitch
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_rotation
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<double, num_battery, 1>::Zero(),  // battery_voltage
      Eigen::Vector3d::Zero(),                        // setpoints
      0.0,                                            // desired_speed
      Eigen::Vector2d::Zero(),                        // startingpoint
      Eigen::Vector2d::Zero(),                        // endingpoint
      Eigen::Matrix<double, 2, 8>::Zero()             // waypoints
  };

  messages::guilink_serial<num_thruster, num_battery, dim_controlspace>
      _guiserial(_guilinkRTdata, 115200);
  common::timecounter _timer;
  while (1) {
    std::string pt_utc = _timer.getUTCtime();
    long int et = _timer.timeelapsed();
    std::cout << _timer.getUTCtime() << " " << et << std::endl;
    static int count = 0;
    count++;
    if (count > 400) count = 0;

    _guilinkRTdata.UTC_time = pt_utc;
    _guilinkRTdata.indicator_autocontrolmode = count;
    _guiserial.setguilinkRTdata(_guilinkRTdata).guicommunication();
    std::cout << "recv: " << _guiserial.getrecv_buffer() << std::endl;
    std::cout << "send: " << _guiserial.getsend_buffer() << std::endl;

    _guilinkRTdata = _guiserial.getguilinkRTdata();
  }
}  // test_gui_serial

void test_gui_socket() {}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  test_gui_serial();
}