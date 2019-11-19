/*
***********************************************************************
* testnode2d.cc:
* A two dimensional node class used for the holonomic with obstacles heuristic.
* Each node has a unique discrete position (x,y).
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <iostream>
#include "Node2D.h"
#include "Node3D.h"

using namespace ASV::planning;

void test_node2d() {
  int width = 90;
  int height = 90;
  constexpr int dir = 8;
  // possible movements
  Eigen::Matrix<int, dir, 1> dx, dy;
  dx << -1, -1, 0, 1, 1, 1, 0, -1;
  dy << 0, 1, 1, 1, 0, -1, -1, -1;

  Node2D<dir> nPred(2, 3, 0, 0, dx, dy);
  for (int i = 0; i != dir; ++i) {
    Node2D<dir> nSucc = nPred.createSuccessor(i);
    int iSucc = nSucc.setIdx(width).getIdx();
    // calculate new G value
    double newG = nSucc.updateG(nPred).getG();
    std::cout << "isucc: " << iSucc << std::endl;
    std::cout << "newG: " << newG << std::endl;
  }
}  //

void test_node3d() {
  int width = 90;
  int height = 90;
  constexpr int dir = 3;
  // possible movements
  Eigen::Matrix<double, dir, 1> dx, dy, dtheta;
  dx << 0.7068582, 0.705224, 0.705224;
  dy << 0, 0.0415893, -0.0415893;
  dtheta << 0, 0.1178097, -0.1178097;

  Node3D nPred(2, 3, 0, 0, 0, dx, dy, dtheta);
  for (int i = 0; i != 6; ++i) {
    Node3D nSucc = nPred.createSuccessor(static_cast<MOVEPRIMITIVE>(i));
    int iSucc = nSucc.setIdx(width, height).getIdx();
    // calculate new G value
    double newG = nSucc.updateG(nPred).getG();
    std::cout << "isucc: " << iSucc << std::endl;
    std::cout << "newG: " << newG << std::endl;
  }
}

int main() { test_node3d(); }