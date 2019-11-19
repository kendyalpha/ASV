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
namespace planning {

template <int dir = 8>  // Number of possible directions
class Node2D {
  using vectorni = Eigen::Matrix<int, dir, 1>;

 public:
  /// Constructor for a node with the given arguments
  Node2D(int _x, int _y, double _g, double _h, const vectorni& _dx,
         const vectorni& _dy)
      : dx(_dx),
        dy(_dy),
        x(_x),
        y(_y),
        g(_g),
        h(_h),
        idx(-1),
        o(false),
        c(false),
        d(false) {}
  virtual ~Node2D() = default;

  int getX() const noexcept { return x; }
  int getY() const noexcept { return y; }
  double getG() const noexcept { return g; }
  double getH() const noexcept { return h; }
  // get the total estimated cost
  double getC() const { return g + h; }
  /// get the index of the node in the 2D array
  int getIdx() const { return idx; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is discovered
  bool isDiscovered() const { return d; }

  void setX(int _x) { x = _x; }
  void setY(int _y) { y = _y; }
  void setG(double _g) { g = _g; }
  void setH(double _h) { h = _h; }
  /// set and get the index of the node in the 2D array
  Node2D& setIdx(int _width) {
    idx = y * _width + x;
    return *this;
  }
  // open the node
  void open() {
    o = true;
    c = false;
  }
  // close the node
  void close() {
    c = true;
    o = false;
  }
  // set the node neither open nor closed
  void reset() {
    c = false;
    o = false;
  }
  // discover the node
  void discover() { d = true; }

  // Updates the cost-so-far for the node x' coming from its predecessor. It
  // also discovers the node.
  Node2D& updateG(const Node2D<dir>& _pred) {
    g += movementCost(_pred);
    discover();
    return *this;
  }
  // Updates the cost-to-go for the node x' to the goal node.
  void updateH(const Node2D<dir>& _goal) { h = movementCost(_goal); }
  // The heuristic as well as the cost measure.
  double movementCost(const Node2D<dir>& _pred) const {
    return std::sqrt((x - _pred.x) * (x - _pred.x) +
                     (y - _pred.y) * (y - _pred.y));
  }

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y
  /// position is the same.
  bool operator==(const Node2D<dir>& rhs) const {
    return x == rhs.x && y == rhs.y;
  }

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 2D array.
  bool isOnGrid(const int width, const int height) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  // SUCCESSOR CREATION
  /// Creates a successor on a eight-connected grid.
  auto createSuccessor(const int i) {
    return Node2D<dir>(x + dx(i), y + dy(i), g, 0, dx, dy);
  }

 private:
  /// Possible movements in the x direction
  const vectorni dx;
  /// Possible movements in the y direction
  const vectorni dy;

  /// the x position
  int x;
  /// the y position
  int y;
  /// the cost-so-far (real value)
  double g;
  /// the cost-to-go (heuristic value)
  double h;
  /// the index of the node in the 2D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the discovered value
  bool d;
};  // Node2D

}  // namespace planning
}  // namespace ASV
#endif /* NODE2D_H */