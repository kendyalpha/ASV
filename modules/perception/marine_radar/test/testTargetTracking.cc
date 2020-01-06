/*
****************************************************************************
* testTargetTracking.cc:
* unit test for Target tracking using marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <pyclustering/cluster/dbscan.hpp>
#include <pyclustering/utils/metric.hpp>

#include <iostream>

#include "../include/TargetTracking.h"
#include "common/fileIO/include/utilityio.h"

void test_2d_miniball() {
  const int d = 2;  // dimension
  const int n = 5;  // number of points

  std::srand(10);

  Eigen::MatrixXd save_points = Eigen::MatrixXd::Zero(n, d);
  Eigen::MatrixXd save_circle = Eigen::MatrixXd::Zero(d + 1, 1);

  // generate random points and store them in a 2-d array
  // ----------------------------------------------------

  // double** ap = new double*[n];
  // for (int i = 0; i < n; ++i) {
  //   double* p = new double[d];
  //   for (int j = 0; j < d; ++j) {
  //     p[j] = rand();
  //     save_points(i, j) = p[j];
  //   }
  //   ap[i] = p;
  // }

  double test_x[n] = {0, 1, 2, 3, 4};
  double test_y[n] = {0, 1, 2, 3, 4};
  double** ap = new double*[n];
  for (int i = 0; i < n; ++i) {
    double* p = new double[d];
    p[0] = test_x[i];
    p[1] = test_y[i];
    save_points(i, 0) = p[0];
    save_points(i, 1) = p[1];

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

void testClustering() {
  double p_radius = 0.7;
  size_t p_neighbors = 3;

  pyclustering::dataset p_data = {
      {1.520060, 0.962128}, {2.022029, 0.808040}, {2.057351, 1.211364},
      {2.264624, 0.691987}, {1.833837, 0.873777}, {1.642185, 0.759126},
      {2.296886, 1.049943}, {1.651659, 1.479517}, {2.171928, 0.833745},
      {1.911756, 0.765536}, {2.037709, 0.885850}, {1.973467, 0.586479},
      {2.290475, 1.157447}, {1.561361, 1.167609}, {1.974000, 0.643743},
      {2.402489, 2.512324}, {1.989742, 2.578131}, {1.769925, 2.894308},
      {2.337243, 3.089108}, {1.763479, 3.078625}, {1.877453, 3.473692},
      {1.648136, 3.203372}, {1.983505, 3.397341}, {1.777163, 3.081633},
      {2.373442, 2.613905}, {2.125649, 3.245356}, {2.011568, 3.420420},
      {2.371343, 3.280969}, {1.866134, 2.656021}, {2.407069, 2.510251},
      {2.411449, 4.922576}, {1.529495, 4.624166}, {2.213014, 4.655022},
      {2.158573, 5.438162}, {2.453622, 5.487703}, {1.706733, 5.095323},
      {1.688246, 4.716194}, {1.648121, 5.033194}, {2.042832, 5.456132},
      {1.637036, 4.715025}, {2.389355, 4.519055}, {1.772783, 5.389859},
      {2.430609, 4.585668}, {1.835628, 4.928907}, {2.010313, 5.031169},
      {2.130630, 6.686596}, {1.908663, 7.473307}, {1.803819, 7.333536},
      {1.876055, 6.635525}, {1.624722, 7.233937}, {2.479029, 6.928866},
      {1.647363, 7.127566}, {2.093093, 6.662946}, {1.962334, 6.812723},
      {1.716604, 6.539181}, {1.829242, 6.924507}, {1.862751, 7.229120},
      {2.232196, 7.306841}, {1.973060, 6.695993}, {2.322847, 7.056129},
      {2.496003, 9.163995}, {2.252416, 8.745723}, {2.420608, 9.339556},
      {2.240567, 8.871278}, {1.980673, 8.529820}, {1.598466, 9.284379},
      {2.405742, 9.410697}, {1.785233, 9.384720}, {2.177794, 9.324454},
      {1.845079, 8.689630}, {2.375952, 8.684424}, {1.619341, 8.743305},
      {2.405635, 8.782045}, {1.905190, 9.464194}, {1.745843, 9.369996}};

  std::shared_ptr<pyclustering::clst::dbscan_data> ptr_output_result =
      std::make_shared<pyclustering::clst::dbscan_data>();
  pyclustering::clst::dbscan solver(p_radius, p_neighbors);
  solver.process(p_data, *ptr_output_result);
  const pyclustering::clst::cluster_sequence& actual_clusters =
      ptr_output_result->clusters();

  const std::vector<size_t> expected_clusters_length = {15, 15, 15, 15, 15};

  for (auto const& cluster : actual_clusters) {
    for (auto& value : cluster) {
      std::cout << value << " ";
    }
    std::cout << "\n";
  }
}

int main() {
  testClustering();
  // test_2d_miniball();
}