/*
***********************************************************************
* testFrenetTrajectoryGenerator.cc:
* Utility test for Frenet optimal trajectory generator
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#include "../include/LatticePlanner.h"
#include "common/plotting/include/gnuplot-iostream.h"
#include "common/timer/include/timecounter.h"

#include <random>

using namespace ASV;

// output figures or not
constexpr bool figure_output = true;

// illustrate the lattice planner at a time instant
void rtplotting_bestpath(
    Gnuplot &_gp, const double vessel_x, const double vessel_y,
    const double vessel_heading, const double vessel_speed,
    const std::vector<double> &_cart_obstacle_x,
    const std::vector<double> &_cart_obstacle_y, const Eigen::VectorXd &_ref_x,
    const Eigen::VectorXd &_ref_y, const Eigen::VectorXd &_cart_best_x,
    const Eigen::VectorXd &_cart_best_y,
    const std::vector<planning::Frenet_path> &_all_frenet_paths = {}) {
  static std::vector<double> vessel_profile_x({2.0, 1.1, -1.0, -1.0, 1.1});
  static std::vector<double> vessel_profile_y({0.0, 0.8, 0.8, -0.8, -0.8});

  double cvalue = std::cos(vessel_heading);
  double svalue = std::sin(vessel_heading);

  std::vector<std::pair<double, double> > xy_pts_A;

  double area = 17;
  _gp << "set xrange [" << vessel_y - area << ":" << vessel_y + area << "]\n";
  _gp << "set yrange [" << vessel_x - area << ":" << vessel_x + area << "]\n";

  // plot name
  if (figure_output) {
    static int figure_index = 0;
    figure_index++;
    _gp << "set output 'frenet_" + std::to_string(figure_index) + ".png'\n";
  }

  // lattice
  _gp << "plot ";
  for (int i = 0; i != _all_frenet_paths.size(); ++i) {
    xy_pts_A.clear();
    auto one_frenet_path = _all_frenet_paths[i];
    for (int j = 0; j != one_frenet_path.x.size(); ++j)
      xy_pts_A.push_back(
          std::make_pair(-one_frenet_path.y(j), one_frenet_path.x(j)));
    _gp << _gp.file1d(xy_pts_A)
        << "with line linetype 1 lw 0.5 lc rgb '#FDDBC7' notitle,";
  }

  // obstacle
  xy_pts_A.clear();
  for (std::size_t i = 0; i != _cart_obstacle_x.size(); ++i) {
    xy_pts_A.push_back(
        std::make_pair(-_cart_obstacle_y[i], _cart_obstacle_x[i]));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with points pt 9 ps 3 lc rgb '#2166AC' title 'obstacles', ";

  // best path
  xy_pts_A.clear();
  for (int i = 0; i != _cart_best_x.size(); ++i) {
    xy_pts_A.push_back(std::make_pair(-_cart_best_y(i), _cart_best_x(i)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'best path',";

  // reference line
  xy_pts_A.clear();
  for (int i = 0; i != _ref_x.size(); ++i) {
    xy_pts_A.push_back(std::make_pair(-_ref_y(i), _ref_x(i)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << "with line linetype 3 lw 2 lc rgb '#B2182B' title 'reference'\n";

  // vessel shape
  _gp << "set object 1 polygon from";
  for (std::size_t i = 0; i != vessel_profile_x.size(); ++i) {
    double x =
        vessel_x + cvalue * vessel_profile_x[i] - svalue * vessel_profile_y[i];
    double y =
        vessel_y + svalue * vessel_profile_x[i] + cvalue * vessel_profile_y[i];

    _gp << " " << y << ", " << x << " to";
  }
  _gp << " "
      << vessel_y + svalue * vessel_profile_x[0] + cvalue * vessel_profile_y[0]
      << ", "
      << vessel_x + cvalue * vessel_profile_x[0] - svalue * vessel_profile_y[0]
      << "\n";
  _gp << "set object 1 fc rgb 'blue' fillstyle solid 0.2 noborder\n";

  _gp.flush();

}  // rtplotting_bestpath

void rtplotting_speed(Gnuplot &_gp, const Eigen::VectorXd &_cart_best_speed) {
  std::vector<std::pair<double, double> > xy_pts_C;

  for (int i = 0; i != _cart_best_speed.size(); ++i) {
    xy_pts_C.push_back(std::make_pair(i, _cart_best_speed(i)));
  }
  // the second subplot
  _gp << "plot " << _gp.file1d(xy_pts_C)
      << " with lines lt 1 lw 2 lc rgb 'red' title 'speed'\n";
  _gp.flush();
}  // rtplotting_speed

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // trajectory generator
  Eigen::VectorXd marine_WX(5);
  Eigen::VectorXd marine_WY(5);
  // marine_WX << 0.0, 25, 50, 75, 100;
  // marine_WY << 0.0, -25, 0.0, 25, 0.0;
  marine_WX << 0.0, 10.0, 20.5, 35.0, 70.5;
  marine_WY << 0.0, 6.0, -5.0, -6.5, 0.0;

  std::vector<double> marine_surrounding_x{20.0, 30.0, 30.0, 35.0, 34.0, 50.0};
  std::vector<double> marine_surrounding_y{-10.0, -6.0, -8.0, -8.0, -8.0, -3.0};

  // std::vector<double> marine_surrounding_x{50, 50.0, 50.0, 50.0, 48, 50.0};
  // std::vector<double> marine_surrounding_y{-2.0, -1.0, 2.0, 1, 0, 3.0};

  std::vector<double> marine_surrounding_x_test{53, 54, 56, 58, 58, 60};
  std::vector<double> marine_surrounding_y_test{-2.0, -2.0, -2.0,
                                                -2.0, -10,  10};

  planning::LatticeData _latticedata{
      0.1,         // SAMPLE_TIME
      50.0 / 3.6,  // MAX_SPEED
      0.05,        // TARGET_COURSE_ARC_STEP
      7.0,         // MAX_ROAD_WIDTH
      1,           // ROAD_WIDTH_STEP
      5.0,         // MAXT
      4.0,         // MINT
      0.2,         // DT
      0.4,         // MAX_SPEED_DEVIATION
      0.2          // TRAGET_SPEED_STEP
  };

  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      2.0,   // MAX_ANG_ACCEL
      -2.0,  // MIN_ANG_ACCEL
      0.2,   // MAX_CURVATURE
      3,     // HULL_LENGTH
      1,     // HULL_WIDTH
      3.3    // ROBOT_RADIUS
  };

  // real time data
  planning::CartesianState Plan_cartesianstate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
      0,           // yaw_rate
      0            // yaw_accel
  };

  planning::CartesianState estimate_marinestate{
      0,            // x
      -1,           // y
      -0.2 * M_PI,  // theta
      0,            // kappa
      1,            // speed
      0,            // dspeed
      0,            // yaw_rate
      0             // yaw_accel
  };

  planning::LatticePlanner _trajectorygenerator(_latticedata, _collisiondata);
  _trajectorygenerator.regenerate_target_course(marine_WX, marine_WY);

  // timer
  common::timecounter _timer;

  // plotting
  Gnuplot gp1;
  Gnuplot gp2;

  if (figure_output)
    gp1 << "set terminal png size 1200, 1200 0\n";
  else
    gp1 << "set terminal x11 size 1200, 1200 0\n";

  gp1 << "set grid \n";
  gp1 << "set title 'Lattice generator'\n";
  gp1 << "set xlabel 'E (m)'\n";
  gp1 << "set ylabel 'N (m)'\n";

  gp2 << "set terminal x11 size 800, 400 1\n";
  gp2 << "set title 'best speed'\n";
  gp2 << "set xlabel 'Time (s)'\n";
  gp2 << "set ylabel 'speed (m/s)'\n";

  for (int i = 0; i != 500; ++i) {
    if (i < 10)
      _trajectorygenerator.setup_obstacle(marine_surrounding_x,
                                          marine_surrounding_y);
    else
      _trajectorygenerator.setup_obstacle(marine_surrounding_x_test,
                                          marine_surrounding_y_test);
    Plan_cartesianstate =
        _trajectorygenerator
            .trajectoryonestep(
                estimate_marinestate.x, estimate_marinestate.y,
                estimate_marinestate.theta, estimate_marinestate.kappa,
                estimate_marinestate.speed, estimate_marinestate.dspeed, 3)
            .getnextcartesianstate();

    estimate_marinestate = Plan_cartesianstate;

    std::tie(estimate_marinestate.y, estimate_marinestate.theta,
             estimate_marinestate.kappa) =
        common::math::Cart2Marine(Plan_cartesianstate.y,
                                  Plan_cartesianstate.theta,
                                  Plan_cartesianstate.kappa);

    auto all_frenet_paths = _trajectorygenerator.getallfrenetpaths();
    auto cart_rx = _trajectorygenerator.getCartRefX();
    auto cart_ry = _trajectorygenerator.getCartRefY();
    auto cart_bestX = _trajectorygenerator.getbestX();
    auto cart_bestY = _trajectorygenerator.getbestY();
    auto cart_bestspeed = _trajectorygenerator.getbestSpeed();
    auto cart_ob_x = _trajectorygenerator.getobstacle_x();
    auto cart_ob_y = _trajectorygenerator.getobstacle_y();

    if ((std::pow(estimate_marinestate.x - cart_rx(cart_rx.size() - 1), 2) +
         std::pow(estimate_marinestate.y + cart_ry(cart_ry.size() - 1), 2)) <=
        1.0) {
      std::cout << "goal\n";
      break;
    }
    rtplotting_bestpath(gp1, estimate_marinestate.x, estimate_marinestate.y,
                        estimate_marinestate.theta, estimate_marinestate.speed,
                        cart_ob_x, cart_ob_y, cart_rx, cart_ry, cart_bestX,
                        cart_bestY, all_frenet_paths);
    rtplotting_speed(gp2, cart_bestspeed);

    long int et = _timer.timeelapsed();
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "elapsed time of each calculation: " << et << std::endl;
  }
}
