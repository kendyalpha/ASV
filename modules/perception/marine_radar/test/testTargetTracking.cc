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
#include "common/plotting/include/matplotlibcpp.h"

void generatecircle(const double center_x, const double center_y,
                    const double radius, std::vector<double>& _circle_x,
                    std::vector<double>& _circle_y) {
  const int n = 120;
  _circle_x.resize(n);
  _circle_y.resize(n);
  for (int i = 0; i < n; ++i) {
    double t = 2 * M_PI * i / n;
    _circle_x.at(i) = center_x + radius * std::cos(t);
    _circle_y.at(i) = center_y + radius * std::sin(t);
  }
}

void test_2d_miniball() {
  const int d = 2;    // dimension
  const int n = 100;  // number of points

  std::srand(10);

  // generate random points and store them in a 2-d array
  // ----------------------------------------------------
  std::vector<double> points_x;
  std::vector<double> points_y;

  double** ap = new double*[n];
  for (int i = 0; i < n; ++i) {
    double* p = new double[d];
    for (int j = 0; j < d; ++j) {
      p[j] = rand();
    }
    ap[i] = p;
    points_x.push_back(p[0]);
    points_y.push_back(p[1]);
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
  for (int i = 0; i < d; ++i) {
    std::cout << *(center + i) << " ";
  }
  std::cout << std::endl;

  // squared radius
  std::cout << "Squared radius:\n  ";
  std::cout << mb.squared_radius() << std::endl;

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

  // plotting
  // Set the size of output image = 1200x780 pixels
  matplotlibcpp::figure_size(800, 780);

  // Plot line from given x and y data. Color is selected automatically.
  matplotlibcpp::plot(points_x, points_y, ".");

  std::vector<double> circle_x;
  std::vector<double> circle_y;

  generatecircle(center[0], center[1], std::sqrt(mb.squared_radius()), circle_x,
                 circle_y);

  // Plot a red dashed line from given x and y data.
  matplotlibcpp::plot(circle_x, circle_y, "r-");
  matplotlibcpp::title("Smallest Closing Ball of Points");
  matplotlibcpp::show();
}  // test_2d_miniball

void testClustering() {
  double p_radius = 0.7;
  size_t p_neighbors = 3;

  std::vector<double> p_data_x = {
      1.520060, 2.022029, 2.057351, 2.264624, 1.833837, 1.642185, 2.296886,
      1.651659, 2.171928, 1.911756, 2.037709, 1.973467, 2.290475, 1.561361,
      1.974000, 2.402489, 1.989742, 1.769925, 2.337243, 1.763479, 1.877453,
      1.648136, 1.983505, 1.777163, 2.373442, 2.125649, 2.011568, 2.371343,
      1.866134, 2.407069, 2.411449, 1.529495, 2.213014, 2.158573, 2.453622,
      1.706733, 1.688246, 1.648121, 2.042832, 1.637036, 2.389355, 1.772783,
      2.430609, 1.835628, 2.010313, 2.130630, 1.908663, 1.803819, 1.876055,
      1.624722, 2.479029, 1.647363, 2.093093, 1.962334, 1.716604, 1.829242,
      1.862751, 2.232196, 1.973060, 2.322847, 2.496003, 2.252416, 2.420608,
      2.240567, 1.980673, 1.598466, 2.405742, 1.785233, 2.177794, 1.845079,
      2.375952, 1.619341, 2.405635, 1.905190, 1.745843};
  std::vector<double> p_data_y = {
      0.962128, 0.808040, 1.211364, 0.691987, 0.873777, 0.759126, 1.049943,
      1.479517, 0.833745, 0.765536, 0.885850, 0.586479, 1.157447, 1.167609,
      0.643743, 2.512324, 2.578131, 2.894308, 3.089108, 3.078625, 3.473692,
      3.203372, 3.397341, 3.081633, 2.613905, 3.245356, 3.420420, 3.280969,
      2.656021, 2.510251, 4.922576, 4.624166, 4.655022, 5.438162, 5.487703,
      5.095323, 4.716194, 5.033194, 5.456132, 4.715025, 4.519055, 5.389859,
      4.585668, 4.928907, 5.031169, 6.686596, 7.473307, 7.333536, 6.635525,
      7.233937, 6.928866, 7.127566, 6.662946, 6.812723, 6.539181, 6.924507,
      7.229120, 7.306841, 6.695993, 7.056129, 9.163995, 8.745723, 9.339556,
      8.871278, 8.529820, 9.284379, 9.410697, 9.384720, 9.324454, 8.689630,
      8.684424, 8.743305, 8.782045, 9.464194, 9.369996};

  std::shared_ptr<pyclustering::dataset> p_data =
      std::make_shared<pyclustering::dataset>();
  std::shared_ptr<pyclustering::clst::dbscan_data> ptr_output_result =
      std::make_shared<pyclustering::clst::dbscan_data>();

  for (std::size_t i = 0; i != p_data_x.size(); ++i) {
    p_data->push_back({p_data_x[i], p_data_y[i]});
  }

  pyclustering::clst::dbscan solver(p_radius, p_neighbors);
  solver.process(*p_data, *ptr_output_result);
  const pyclustering::clst::cluster_sequence& actual_clusters =
      ptr_output_result->clusters();

  // plotting
  // Set the size of output image = 1200x780 pixels
  matplotlibcpp::figure_size(800, 780);

  for (std::size_t index = 0; index != actual_clusters.size(); ++index) {
    std::vector<double> cluster_x;
    std::vector<double> cluster_y;

    for (auto& value : actual_clusters[index]) {
      cluster_x.push_back(p_data_x[value]);
      cluster_y.push_back(p_data_y[value]);
    }

    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::plot(cluster_x, cluster_y, ".");
  }

  matplotlibcpp::title("Clustering results");
  matplotlibcpp::show();

}  // testClustering

void testClusteringAndBall() {
  using namespace ASV::perception;
  AlphaBetaData AlphaBeta_Data{
      0.1,  // sample_time
      0.1,  // alpha
      0.1   // beta
  };
  ClusteringData Clustering_Data{
      0.7,  // p_radius
      3     // p_minumum_neighbors
  };

  std::vector<double> p_data_x = {
      1.520060, 2.022029, 2.057351, 2.264624, 1.833837, 1.642185, 2.296886,
      1.651659, 2.171928, 1.911756, 2.037709, 1.973467, 2.290475, 1.561361,
      1.974000, 2.402489, 1.989742, 1.769925, 2.337243, 1.763479, 1.877453,
      1.648136, 1.983505, 1.777163, 2.373442, 2.125649, 2.011568, 2.371343,
      1.866134, 2.407069, 2.411449, 1.529495, 2.213014, 2.158573, 2.453622,
      1.706733, 1.688246, 1.648121, 2.042832, 1.637036, 2.389355, 1.772783,
      2.430609, 1.835628, 2.010313, 2.130630, 1.908663, 1.803819, 1.876055,
      1.624722, 2.479029, 1.647363, 2.093093, 1.962334, 1.716604, 1.829242,
      1.862751, 2.232196, 1.973060, 2.322847, 2.496003, 2.252416, 2.420608,
      2.240567, 1.980673, 1.598466, 2.405742, 1.785233, 2.177794, 1.845079,
      2.375952, 1.619341, 2.405635, 1.905190, 1.745843};
  std::vector<double> p_data_y = {
      0.962128, 0.808040, 1.211364, 0.691987, 0.873777, 0.759126, 1.049943,
      1.479517, 0.833745, 0.765536, 0.885850, 0.586479, 1.157447, 1.167609,
      0.643743, 2.512324, 2.578131, 2.894308, 3.089108, 3.078625, 3.473692,
      3.203372, 3.397341, 3.081633, 2.613905, 3.245356, 3.420420, 3.280969,
      2.656021, 2.510251, 4.922576, 4.624166, 4.655022, 5.438162, 5.487703,
      5.095323, 4.716194, 5.033194, 5.456132, 4.715025, 4.519055, 5.389859,
      4.585668, 4.928907, 5.031169, 6.686596, 7.473307, 7.333536, 6.635525,
      7.233937, 6.928866, 7.127566, 6.662946, 6.812723, 6.539181, 6.924507,
      7.229120, 7.306841, 6.695993, 7.056129, 9.163995, 8.745723, 9.339556,
      8.871278, 8.529820, 9.284379, 9.410697, 9.384720, 9.324454, 8.689630,
      8.684424, 8.743305, 8.782045, 9.464194, 9.369996};

  TargetTracking Target_Tracking(AlphaBeta_Data, Clustering_Data);

  auto TargetTracker_RTdata =
      Target_Tracking.AutoTracking(p_data_x, p_data_y).getTargetTrackerRTdata();

  // plotting
  // Set the size of output image = 1200x780 pixels
  matplotlibcpp::figure_size(800, 780);

  for (std::size_t index = 0; index != TargetTracker_RTdata.target_x.size();
       ++index) {
    std::vector<double> circle_x;
    std::vector<double> circle_y;

    generatecircle(TargetTracker_RTdata.target_x[index],
                   TargetTracker_RTdata.target_y[index],
                   std::sqrt(TargetTracker_RTdata.target_square_radius[index]),
                   circle_x, circle_y);
    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::plot(circle_x, circle_y, "-");
  }
  matplotlibcpp::plot(p_data_x, p_data_y, ".");

  matplotlibcpp::title("Clustering and MiniBall results");
  matplotlibcpp::axis("equal");
  matplotlibcpp::show();

}  // testClusteringAndBall

int main() {
  testClusteringAndBall();
  // test_2d_miniball();
}