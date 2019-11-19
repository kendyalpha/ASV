/*
***********************************************************************
* ConstraintChecker.h:
*
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _CONSTRAINTCHECKER_H_
#define _CONSTRAINTCHECKER_H_

namespace ASV::planning {
class ConstraintChecker {
 public:
  enum class Result {
    VALID = 0,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    LAT_VELOCITY_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND,
  };
  ConstraintChecker() = delete;
  static Result ValidTrajectory(const DiscretizedTrajectory& trajectory);
};
};  // namespace ASV::planning

#endif /* _CONSTRAINTCHECKER_H_ */