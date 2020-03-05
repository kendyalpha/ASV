/*
***********************************************************************
* Node3D.h:
* The 3d node (x, y, theta) in the configuration space
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef NODE3D_H
#define NODE3D_H

#include <stdlib.h>
#include "common/math/miscellaneous/include/math_utils.h"
#include "openspacedata.h"

namespace ASV {
namespace planning {

enum class MOVEPRIMITIVE {
  FS = 0,  // move forward straight
  FL,      // move forward left
  FR,      // move forward right
  RS,      // move reverse straight
  RL,      // move reverse left
  RR       // move reverse right
};

enum class NODESTATUS { CLOSED = 0, OPEN };

class Node3D {
 public:
  /// Constructor for a node with the given arguments
  Node3D(double _x, double _y, double _theta, double _g, double _h,
         const Eigen::Vector3d& _dx, const Eigen::Vector3d& _dy,
         const Eigen::Vector3d& _dtheta,
         MOVEPRIMITIVE _primitive = MOVEPRIMITIVE::FS)
      : dx(_dx),
        dy(_dy),
        dtheta(_dtheta),
        x(_x),
        y(_y),
        theta(_theta),
        g(_g),
        h(_h),
        idx(-1),
        nodestatus(NODESTATUS::CLOSED),
        primitive(_primitive) {}
  virtual ~Node3D() = default;

  double getX() const noexcept { return x; }
  double getY() const noexcept { return y; }
  double getTheta() const noexcept { return theta; }
  double getG() const noexcept { return g; }
  double getH() const noexcept { return h; }
  int getIdx() const noexcept { return idx; }
  // get the total estimated cost
  double getC() const noexcept { return g + h; }
  // get the number associated with the motion primitive of the node
  MOVEPRIMITIVE getPrim() const noexcept { return primitive; }
  // determine whether the node is open
  NODESTATUS getnodestatus() const noexcept { return nodestatus; }

  void setX(double _x) { x = _x; }
  void setY(double _y) { y = _y; }
  void setTheta(double _theta) { theta = _theta; }
  void setG(double _g) { g = _g; }
  void setH(double _h) { h = _h; }
  /// set and get the index of the node in the 3D grid
  Node3D& setIdx(int _width, int _height) {
    idx = static_cast<int>(theta / DELTA_HEADING_RAD) * _width * _height +
          static_cast<int>(y) * _width + static_cast<int>(x);
    return *this;
  }
  // open the node
  void open() { nodestatus = NODESTATUS::OPEN; }
  // close the node
  void close() { nodestatus = NODESTATUS::CLOSED; }

  // Updates the cost-so-far for the node x' coming from its predecessor. It
  // also discovers the node.
  Node3D& updateG(const Node3D& _pred) {
    int prim = static_cast<int>(primitive);
    int pre_prim = static_cast<int>(_pred.primitive);
    // forward driving
    if (prim < 3) {
      // penalize turning
      if (pre_prim != prim) {
        // penalize change of direction
        if (pre_prim < 3)
          g += dx(0) * PENALTY_TURNING;
        else
          g += dx(0) * PENALTY_TURNING * PENALTY_COD;
      } else {
        g += dx(0);
      }
    }
    // reverse driving
    else {
      // penalize turning and reversing
      if (pre_prim != prim) {
        // penalize change of direction
        if (pre_prim < 3)
          g += dx(0) * PENALTY_TURNING * PENALTY_REVERSING * PENALTY_COD;
        else
          g += dx(0) * PENALTY_TURNING * PENALTY_REVERSING;
      } else {
        g += dx(0) * PENALTY_REVERSING;
      }
    }

    return *this;
  }

  /// Custom operator to compare nodes. Nodes are equal if their x and y
  /// position as well as heading is similar.
  bool operator==(const Node3D& rhs) const {
    return static_cast<int>(x) == static_cast<int>(rhs.x) &&
           static_cast<int>(y) == static_cast<int>(rhs.y) &&
           (std::abs(common::math::Normalizeheadingangle(theta - rhs.theta)) <=
            DELTA_HEADING_RAD);
  }

  // TODO: why abs and then square
  // RANGE CHECKING: Determines whether it is appropriate to find a analytical
  // solution.
  bool isInRange(const Node3D& goal) const {
    int random = rand() % 10 + 1; /* generate secret number between 1 and 10: */
    double _dx = std::abs(x - goal.x) / random;
    double _dy = std::abs(y - goal.y) / random;
    return (_dx * _dx) + (_dy * _dy) < DUBINS_SHOT_DISTANCE;
  }

  // TODO: 0~2pi ----> -pi~pi
  // GRID CHECKING: Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const {
    return (x >= 0) && (x < width) && (y >= 0) && (y < height) &&
           (theta >= -M_PI) && (theta <= M_PI);
  }

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  auto createSuccessor(const MOVEPRIMITIVE _prim) {
    double xSucc = 0.0;
    double ySucc = 0.0;
    double theta_Succ = 0.0;
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    int i = static_cast<int>(_prim);

    if (i < 3) {
      // calculate successor positions forward
      xSucc = x + dx(i) * cvalue - dy(i) * svalue;
      ySucc = y + dx(i) * svalue + dy(i) * cvalue;
      theta_Succ = common::math::Normalizeheadingangle(theta + dtheta(i));
    } else {
      // backwards
      xSucc = x - dx(i - 3) * cvalue - dy(i - 3) * svalue;
      ySucc = y - dx(i - 3) * svalue + dy(i - 3) * cvalue;
      theta_Succ = common::math::Normalizeheadingangle(theta - dtheta(i - 3));
    }

    return Node3D(xSucc, ySucc, theta_Succ, g, 0, dx, dy, dtheta, _prim);
  }

 private:
  /// Possible movements in the x direction
  const Eigen::Vector3d dx;
  /// Possible movements in the y direction
  const Eigen::Vector3d dy;
  /// Possible movements regarding heading theta
  const Eigen::Vector3d dtheta;

  double x;                 // the x position
  double y;                 // the y position
  double theta;             // heading
  double g;                 // the cost-so-far
  double h;                 // the cost-to-go
  int idx;                  // the index of the node in the 3D array
  NODESTATUS nodestatus;    // open or closed
  MOVEPRIMITIVE primitive;  // the motion primitive of the node

};  // class Node3D

}  // namespace planning

}  // end namespace ASV
#endif /* NODE3D_H */
