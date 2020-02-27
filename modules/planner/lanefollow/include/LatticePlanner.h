/*
****************************************************************************
* LatticePlanner.h:
* Lattice planner including Frenet Lattices generator and collision checker
* define the data struct used in the controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _LATTICEPLANNER_H_
#define _LATTICEPLANNER_H_

#include "CollisionChecker.h"
#include "FrenetTrajectoryGenerator.h"

namespace ASV::planning {

class LatticePlanner : public FrenetTrajectoryGenerator,
                       public CollisionChecker {
 public:
  LatticePlanner(const LatticeData &_Latticedata,
                 const CollisionData &_CollisionData)
      : FrenetTrajectoryGenerator(_Latticedata),
        CollisionChecker(_CollisionData),
        sample_time(_Latticedata.SAMPLE_TIME),
        next_cartesianstate(CartesianState{
            0,            // x
            0,            // y
            -M_PI / 3.0,  // theta
            0,            // kappa
            1,            // speed
            0,            // dspeed
            0,            // yaw_rate
            0             // yaw_accel
        }) {}

  LatticePlanner &trajectoryonestep(double marine_x, double marine_y,
                                    double marine_theta, double marine_kappa,
                                    double marine_speed, double marine_a,
                                    double _targetspeed) {
    // generate lattice
    FrenetTrajectoryGenerator::Generate_Lattice(
        marine_x, marine_y, marine_theta, marine_kappa, marine_speed, marine_a,
        _targetspeed);

    // constraints and collision check
    auto t_frenet_paths =
        CollisionChecker::check_paths(FrenetTrajectoryGenerator::frenet_paths);

    if (t_frenet_paths.size() > 0) {
      // find minimum cost path
      best_path = findmincostpath(t_frenet_paths);
      // update the planning state
      updateNextCartesianStatus();
    } else
      CLOG(ERROR, "Frenet_Lattice") << "No best path!";

    return *this;
  }  // trajectoryonestep

  void regenerate_target_course(const Eigen::VectorXd &_marine_wx,
                                const Eigen::VectorXd &_marine_wy,
                                double initial_target_speed = 1) {
    double initial_theta = FrenetTrajectoryGenerator::regenerate_target_course(
        _marine_wx, _marine_wy);
    next_cartesianstate.x = 0;
    next_cartesianstate.y = 0;
    next_cartesianstate.theta = initial_theta;
    next_cartesianstate.kappa = 0;
    next_cartesianstate.speed = initial_target_speed;
    next_cartesianstate.dspeed = 0;
    next_cartesianstate.yaw_rate = 0;
    next_cartesianstate.yaw_accel = 0;

  }  // regenerate_target_course

  // consider the reference line and obstacle resolution
  void setup_obstacle(const std::vector<double> &_marine_surrounding_x,
                      const std::vector<double> &_marine_surrounding_y) {
    std::size_t size_of_surroundings = _marine_surrounding_x.size();
    if (size_of_surroundings == _marine_surrounding_y.size()) {
      // get the reference line
      auto _CartRefX = FrenetTrajectoryGenerator::getCartRefX();
      auto _CartRefY = FrenetTrajectoryGenerator::getCartRefY();

      // check if the surroundings are obstacles
      for (std::size_t i = 0; i != size_of_surroundings; ++i) {
        // convert to cart coordinate
        auto [surrounding_x, surrounding_y] = common::math::Marine2Cart(
            _marine_surrounding_x[i], _marine_surrounding_y[i]);
        CollisionChecker::IsObstacle(surrounding_x, surrounding_y, _CartRefX,
                                     _CartRefY);
      }

    } else
      return;

  }  // setup_obstacle

  // consider the reference line
  void setup_obstacle(const Eigen::VectorXi &_targets_state,
                      const Eigen::VectorXd &_targets_CPA_marine_x,
                      const Eigen::VectorXd &_targets_CPA_marine_y) {
    unsigned size_of_targets = _targets_state.size();
    // get the reference line
    auto _CartRefX = FrenetTrajectoryGenerator::getCartRefX();
    auto _CartRefY = FrenetTrajectoryGenerator::getCartRefY();
    //
    std::vector<double> new_surroundings_x;  // in the Cartesian coordinate
    std::vector<double> new_surroundings_y;  // in the Cartesian coordinate

    // check if the surroundings are obstacles
    for (unsigned i = 0; i != size_of_targets; ++i) {
      if (_targets_state(i) > 0) {
        // convert to cart coordinate
        auto [surrounding_x, surrounding_y] = common::math::Marine2Cart(
            _targets_CPA_marine_x(i), _targets_CPA_marine_y(i));
        if (!CollisionChecker::check_reference(surrounding_x, surrounding_y,
                                               _CartRefX, _CartRefY)) {
          new_surroundings_x.emplace_back(surrounding_x);
          new_surroundings_y.emplace_back(surrounding_y);
        }
      }
    }

    // update
    CollisionChecker::update_obstacles(new_surroundings_x, new_surroundings_y);
  }  // setup_obstacle

  CartesianState getnextcartesianstate() const noexcept {
    return next_cartesianstate;
  }
  Eigen::VectorXd getbestX() const noexcept { return best_path.x; }
  Eigen::VectorXd getbestY() const noexcept { return best_path.y; }
  Eigen::VectorXd getbestSpeed() const noexcept { return best_path.speed; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  const double sample_time;
  CartesianState next_cartesianstate;
  Frenet_path best_path;

  Frenet_path findmincostpath(const std::vector<Frenet_path> &_frenetpaths) {
    // find minimum cost path
    double mincost = std::numeric_limits<double>::max();
    Frenet_path _best_path = _frenetpaths[0];
    for (std::size_t i = 0; i != _frenetpaths.size(); i++) {
      if (mincost > _frenetpaths[i].cf) {
        mincost = _frenetpaths[i].cf;
        _best_path = _frenetpaths[i];
      }
    }
    return _best_path;
  }

  void updateNextCartesianStatus() {
    // The results of Frenet generation at "DT"
    // TODO: adjust the "index", which is empirical; For simulation without
    // considering controller, index = 1
    // double empirical_num = 0.1;
    // int index = static_cast<int>(empirical_num * frenetdata.HULL_LENGTH /
    //                              (_target_s_dot * frenetdata.SAMPLE_TIME));
    // int max_index = mincost_path.x.size() - 1;
    // if (index <= 1) index = 1;
    // if (index >= max_index) index = max_index;

    int index = 1;
    next_cartesianstate.x = best_path.x(index);
    next_cartesianstate.y = best_path.y(index);
    next_cartesianstate.theta = best_path.yaw(index);
    next_cartesianstate.kappa = best_path.kappa(index);
    next_cartesianstate.speed = best_path.speed(index);
    next_cartesianstate.dspeed = best_path.dspeed(index);
    next_cartesianstate.yaw_rate = best_path.yaw_rate(index);
    next_cartesianstate.yaw_accel = best_path.yaw_accel(index);

  }  // updateNextCartesianStatus

};  // end class LatticePlanner

}  // namespace ASV::planning

#endif /* _LATTICEPLANNER_H_ */
