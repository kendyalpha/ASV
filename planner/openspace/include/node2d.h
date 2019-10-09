/*
***********************************************************************
* node2d.h:
* A two dimensional node class used for the holonomic with obstacles heuristic.
* Each node has a unique discrete position (x,y).
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>
#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

namespace ASV {

class Node2d {
 public:
 private:
  Eigen::Vector2d position;
};

}  // namespace ASV
#endif /* NODE2D_H */