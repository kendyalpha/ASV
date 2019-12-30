/*
****************************************************************************
* testTargetTracking.cc:
* unit test for Target tracking using marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <iostream>
#include "../include/TargetTracking.h"
#include "common/fileIO/include/utilityio.h"

void test_2d_miniball() {
  int d = 2;    // dimension
  int n = 100;  // number of points

  std::srand(10);

  Eigen::MatrixXd save_points = Eigen::MatrixXd::Zero(n, d);
  Eigen::MatrixXd save_circle = Eigen::MatrixXd::Zero(d + 1, 1);

  // generate random points and store them in a 2-d array
  // ----------------------------------------------------
  double** ap = new double*[n];
  for (int i = 0; i < n; ++i) {
    double* p = new double[d];
    for (int j = 0; j < d; ++j) {
      p[j] = rand();
      save_points(i, j) = p[j];
    }
    ap[i] = p;
  }

  // create an instance of Miniball
  // ------------------------------
  Miniball::Miniball<Miniball::CoordAccessor<double* const*, const double*> >
      mb(d, ap, ap + n);

  // output results
  // --------------
  // center
  std::cout << "Center:\n  ";
  const double* center = mb.center();
  for (int i = 0; i < d; ++i, ++center) {
    std::cout << *center << " ";
    save_circle(i) = *center;
  }
  std::cout << std::endl;

  // squared radius
  std::cout << "Squared radius:\n  ";
  std::cout << mb.squared_radius() << std::endl;
  save_circle(d) = std::sqrt(mb.squared_radius());

  // number of support points
  std::cout << "Number of support points:\n  ";
  std::cout << mb.nr_support_points() << std::endl;

  // support points on the boundary determine the smallest enclosing ball
  std::cout << "Support point indices (numbers refer to the input order):\n  ";
  for (auto it = mb.support_points_begin(); it != mb.support_points_end();
       ++it) {
    std::cout << (*it) - ap << " ";  // 0 = first point
  }
  std::cout << std::endl;

  // relative error: by how much does the ball fail to contain all points?
  //                 tiny positive numbers come from roundoff and are ok
  std::cout << "Relative error:\n  ";
  double suboptimality;
  std::cout << mb.relative_error(suboptimality) << std::endl;

  // suboptimality: by how much does the ball fail to be the smallest
  //                enclosing ball of its support points? should be 0
  //                in most cases, but tiny positive numbers are again ok
  std::cout << "Suboptimality:\n  ";
  std::cout << suboptimality << std::endl;

  // validity: the ball is considered valid if the relative error is tiny
  //           (<= 10 times the machine epsilon) and the suboptimality is zero
  std::cout << "Validity:\n  ";
  std::cout << (mb.is_valid() ? "ok" : "possibly invalid") << std::endl;

  // computation time
  std::cout << "Computation time was " << mb.get_time() << " seconds\n";

  // clean up
  for (int i = 0; i < n; ++i) delete[] ap[i];
  delete[] ap;

  //

  // save data to csv file
  std::string _name("../../data/");
  ASV::common::write2csvfile(_name + "points.csv", save_points);
  ASV::common::write2csvfile(_name + "circle.csv", save_circle);
}

void testAlphaBetafiltering() {
  using namespace ASV::perception;
  AlphaBetaData AlphaBeta_Data{
      0.1,  // sample_time
      0.1,  // alpha
      0.1   // beta
  };
  TargetTracking Target_Tracking(AlphaBeta_Data);
  auto [postion_x, postion_y, velocity_vx, velocity_y] =
      Target_Tracking.AlphaBetaFiltering(1, 1, 1, 1, 1, 1);
}

int main() { test_2d_miniball(); }