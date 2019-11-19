/*
***********************************************************************
* timecounter.h: to give the precise elapsed time in milliseconds.
* Boost date_time is not a header-only library.  Please build the library
* and then add it.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TIMECOUNTER_H_
#define _TIMECOUNTER_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer/timer.hpp>

namespace ASV::common {

class timecounter {
  using PTIMER = boost::posix_time::ptime;
  using PCLOCK = boost::posix_time::microsec_clock;

 public:
  timecounter()
      : pt_start(PCLOCK::universal_time()), pt_UTC(PCLOCK::universal_time()){};

  // return the elapsed duration in milliseconds
  long int timeelapsed() {
    PTIMER pt_now(PCLOCK::universal_time());
    boost::posix_time::time_duration t_elapsed = pt_now - pt_start;
    pt_start = pt_now;
    return t_elapsed.total_milliseconds();
  }

  // return the elapsed duration in microseconds
  long int micro_timeelapsed() {
    PTIMER pt_now(PCLOCK::universal_time());
    boost::posix_time::time_duration t_elapsed = pt_now - pt_start;
    pt_start = pt_now;
    return t_elapsed.total_microseconds();
  }

  // return the UTC time (ISO)
  std::string getUTCtime() {
    pt_UTC = PCLOCK::universal_time();
    return to_iso_string(pt_UTC);
  }
  ~timecounter() {}

 private:
  PTIMER pt_start;
  PTIMER pt_UTC;

};  // end class timecounter

}  // namespace ASV::common

#endif /*_TIMECOUNTER_H_*/