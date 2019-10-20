/*
***********************************************************************
* collision_detection.h:
* The CollisionDetection class determines whether a given configuration
* q of the robot will result in a collision with the environment.
* It is supposed to return a boolean value that returns true for collisions
* and false in the case of a safe node.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "Node2D.h"
#include "Node3D.h"

namespace ASV::plannin {

class collision_detection {
 public:
 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};  // collision_detection

}  // namespace ASV::plannin

#endif /* _COLLISIONDETECTION_H_ */