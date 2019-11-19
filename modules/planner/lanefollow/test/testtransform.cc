/*
***********************************************************************
* testtransform.cc:
* Utility test for transformation from Cartesian x,y coordinates
* to Frenet s,d coordinates
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include <iostream>
#include "../include/FrenetTrajectoryGenerator.h"

using namespace ASV;
int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  planning::LatticeData _latticedata{
      0.1,         // SAMPLE_TIME
      50.0 / 3.6,  // MAX_SPEED
      0.05,        // TARGET_COURSE_ARC_STEP
      7.0,         // MAX_ROAD_WIDTH
      1,           // ROAD_WIDTH_STEP
      7.0,         // MAXT
      6.0,         // MINT
      0.5,         // DT
      0.5,         // MAX_SPEED_DEVIATION
      0.1          // TRAGET_SPEED_STEP
  };

  planning::CartesianState Before_cartesianstate{
      1,            // x
      0,            // y
      -M_PI / 3.0,  // theta
      0.11,         // kappa
      2,            // speed
      0,            // dspeed
  };

  planning::CartesianState After_cartesianstate{
      0,   // x
      0,   // y
      0,   // theta
      0.,  // kappa
      0,   // speed
      0,   // dspeed
  };

  planning::FrenetState frenetstate{
      10,  // s
      0,   // s_dot
      0,   // s_ddot
      2,   // d
      0,   // d_dot
      0,   // d_ddot
      0,   // d_prime
      0    // d_pprime
  };

  planning::FrenetTrajectoryGenerator _trajectorygenerator(_latticedata);
  transformc2f(_trajectorygenerator, frenetstate, Before_cartesianstate);
  std::cout << "Results of Transformation from Cartesian to Frenet\n";
  std::cout << "s: " << frenetstate.s << std::endl;
  std::cout << "s_dot: " << frenetstate.s_dot << std::endl;
  std::cout << "s_ddot: " << frenetstate.s_ddot << std::endl;
  std::cout << "d: " << frenetstate.d << std::endl;
  std::cout << "d_dot: " << frenetstate.d_dot << std::endl;
  std::cout << "d_ddot: " << frenetstate.d_ddot << std::endl;
  std::cout << "d_prime: " << frenetstate.d_prime << std::endl;
  std::cout << "d_pprime: " << frenetstate.d_pprime << std::endl;

  transformf2c(_trajectorygenerator, frenetstate, After_cartesianstate);
  std::cout << "Results of Transformation from Frenet to Cartesian \n";
  std::cout << "x: " << After_cartesianstate.x << std::endl;
  std::cout << "y: " << After_cartesianstate.y << std::endl;
  std::cout << "theta: " << After_cartesianstate.theta << std::endl;
  std::cout << "kappa: " << After_cartesianstate.kappa << std::endl;
  std::cout << "speed: " << After_cartesianstate.speed << std::endl;
  std::cout << "dspeed: " << After_cartesianstate.dspeed << std::endl;

  LOG(INFO) << "Shutting down.";
}
