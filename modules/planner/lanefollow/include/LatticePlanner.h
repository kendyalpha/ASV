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
            0             // dspeed
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

    // find minimum cost path
    best_path = findmincostpath(t_frenet_paths);

    updateNextCartesianStatus();

    return *this;
  }  // trajectoryonestep

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
    double mincost = 1e6;
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

  }  // updateNextCartesianStatus

};  // end class LatticePlanner

}  // namespace ASV::planning

#endif /* _LATTICEPLANNER_H_ */
