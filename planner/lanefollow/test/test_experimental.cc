#include <sqlite_modern_cpp.h>
#include <iostream>
#include "../include/LatticePlanner.h"
#include "common/fileIO/include/utilityio.h"

using namespace ASV;

template <int num_lowpass>
class lowpass {
  using vectorlp = Eigen::Matrix<double, num_lowpass, 1>;

 public:
  lowpass() : averagevector(vectorlp::Zero()) {}
  ~lowpass() {}

  // assign the same value to all elements of "averagevector"
  void setaveragevector(double _initialvalue) {
    averagevector = vectorlp::Constant(_initialvalue);
  }
  // low pass filtering using moving average method
  double movingaverage(double _newstep) {
    // pop_front
    vectorlp t_average = vectorlp::Zero();
    t_average.head(num_lowpass - 1) = averagevector.tail(num_lowpass - 1);
    // push back
    t_average(num_lowpass - 1) = _newstep;
    averagevector = t_average;
    // calculate the mean value
    return averagevector.mean();
  }

 private:
  vectorlp averagevector;
};  // end class lowpass

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  double start_x = 3433825.73;
  double start_y = 350927.165;
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
      8,     // MAX_ROAD_WIDTH
      2,     // ROAD_WIDTH_STEP
      10.0,  // MAXT
      8.0,   // MINT
      0.5,   // DT
      1,     // MAX_SPEED_DEVIATION
      0.5    // TRAGET_SPEED_STEP
  };

  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      0.2,   // MAX_CURVATURE
      3,     // HULL_LENGTH
      1,     // HULL_WIDTH
      2      // ROBOT_RADIUS
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
      " lp_theta    DOUBLE, "
      " lp_kappa    DOUBLE, "
      " speed       DOUBLE);";
  test_db << str;

  // low pass
  lowpass<50> lp_x;
  lowpass<50> lp_y;
  lowpass<10> lp_theta;
  lowpass<50> lp_kappa;
  lowpass<50> lp_speed;
  lowpass<200> lp_plan_theta;
  lowpass<200> lp_plan_kappa;

  lp_x.setaveragevector(start_x);
  lp_y.setaveragevector(start_y);
  lp_theta.setaveragevector(0);
  lp_kappa.setaveragevector(0);
  lp_speed.setaveragevector(0);
  lp_plan_theta.setaveragevector(0);
  lp_plan_kappa.setaveragevector(0);

  for (int i = 10; i != 3000; i++) {
    // read data from DB
    std::string input_str =
        "select state_x,state_y ,state_theta ,curvature ,speed ,dspeed from "
        "estimator where ID=" +
        std::to_string(i + 1) + ";";

    read_db << input_str >>
        std::tie(estimate_marinestate.x, estimate_marinestate.y,
                 estimate_marinestate.theta, estimate_marinestate.kappa,
                 estimate_marinestate.speed, estimate_marinestate.dspeed);

    double x = lp_x.movingaverage(estimate_marinestate.x);
    double y = lp_y.movingaverage(estimate_marinestate.y);
    double theta = lp_theta.movingaverage(estimate_marinestate.theta);
    double kappa = lp_kappa.movingaverage(estimate_marinestate.kappa);
    double speed = lp_speed.movingaverage(estimate_marinestate.speed);

    // Lattice Planner
    Plan_cartesianstate = _trajectorygenerator
                              .trajectoryonestep(estimate_marinestate.x,      //
                                                 estimate_marinestate.y,      //
                                                 estimate_marinestate.theta,  //
                                                 estimate_marinestate.kappa,  //
                                                 estimate_marinestate.speed,  //
                                                 estimate_marinestate.dspeed, 2)
                              .getnextcartesianstate();

    double plan_theta = lp_plan_theta.movingaverage(Plan_cartesianstate.theta);
    double plan_kappa = lp_plan_kappa.movingaverage(Plan_cartesianstate.kappa);

    // write data into DB
    std::string write_str =
        "INSERT INTO PLAN"
        "(DATETIME, theta, kappa, lp_theta, lp_kappa, speed) "
        " VALUES(julianday('now'), " +
        std::to_string(-Plan_cartesianstate.theta) + ", " +
        std::to_string(Plan_cartesianstate.kappa) + ", " +
        std::to_string(estimate_marinestate.theta) + ", " +
        std::to_string(plan_kappa) + ", " +
        std::to_string(Plan_cartesianstate.speed) + ");";

    test_db << write_str;
  }
}