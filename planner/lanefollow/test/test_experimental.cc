#include <sqlite_modern_cpp.h>
#include <iostream>
#include "../include/LatticePlanner.h"
#include "common/fileIO/include/utilityio.h"
#include "common/timer/include/timecounter.h"

using namespace ASV;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  double start_x = 3433822.79;
  double start_y = 350925.455;
  double end_x = 3433790.52;
  double end_y = 350986.45;

  Eigen::VectorXd marine_WX(3);
  Eigen::VectorXd marine_WY(3);
  Eigen::VectorXd marine_ob_x(1);
  Eigen::VectorXd marine_ob_y(1);
  marine_WX << start_x, 0.5 * (start_x + end_x), end_x;
  marine_WY << start_y, 0.5 * (start_y + end_y), end_y;
  marine_ob_x << 0.5 * (start_x + end_x);
  marine_ob_y << 0.5 * (start_y + end_y);

  planning::LatticeData _latticedata{
      0.2,   // SAMPLE_TIME
      4,     // MAX_SPEED
      0.05,  // TARGET_COURSE_ARC_STEP
      6,     // MAX_ROAD_WIDTH
      1,     // ROAD_WIDTH_STEP
      8.0,   // MAXT
      6.0,   // MINT
      0.5,   // DT
      0.6,   // MAX_SPEED_DEVIATION
      0.2    // TRAGET_SPEED_STEP
  };

  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      0.2,   // MAX_CURVATURE
      3,     // HULL_LENGTH
      1,     // HULL_WIDTH
      3      // ROBOT_RADIUS
  };

  // real time data
  planning::CartesianState Plan_cartesianstate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
  };

  planning::CartesianState estimate_marinestate{
      0,           // x
      -1,          // y
      M_PI / 3.0,  // theta
      0,           // kappa
      1,           // speed
      0,           // dspeed
  };

  planning::LatticePlanner _trajectorygenerator(_latticedata, _collisiondata);
  _trajectorygenerator.regenerate_target_course(marine_WX, marine_WY);
  _trajectorygenerator.setobstacle(marine_ob_x, marine_ob_y);

  sqlite::database read_db("experiment.db");
  sqlite::database test_db("test.db");

  std::string str =
      "CREATE TABLE PLAN"
      "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
      " DATETIME    TEXT       NOT NULL,"
      " theta       DOUBLE, "
      " kappa       DOUBLE, "
      " speed       DOUBLE);";
  test_db << str;

  // timer
  common::timecounter _timer;

  for (int i = 10; i != 4000; i++) {
    std::string input_str =
        "select state_x,state_y ,state_theta ,curvature ,speed ,dspeed from "
        "estimator where ID=" +
        std::to_string(i + 1) + ";";

    read_db << input_str >>
        std::tie(estimate_marinestate.x, estimate_marinestate.y,
                 estimate_marinestate.theta, estimate_marinestate.kappa,
                 estimate_marinestate.speed, estimate_marinestate.dspeed);
    // Lattice Planner
    Plan_cartesianstate = _trajectorygenerator
                              .trajectoryonestep(estimate_marinestate.x,  //
                                                 estimate_marinestate.y,  //
                                                 estimate_marinestate.theta,
                                                 estimate_marinestate.kappa,
                                                 estimate_marinestate.speed,
                                                 estimate_marinestate.dspeed, 3)
                              .getnextcartesianstate();

    // write data into DB
    std::string write_str =
        "INSERT INTO PLAN"
        "(DATETIME, theta, kappa, speed) "
        " VALUES(julianday('now'), " +
        std::to_string(Plan_cartesianstate.theta) + ", " +
        std::to_string(Plan_cartesianstate.kappa) + ", " +
        std::to_string(Plan_cartesianstate.speed) + ");";

    test_db << write_str;

    long int et = _timer.timeelapsed();
    // std::cout << et << std::endl;
  }
}