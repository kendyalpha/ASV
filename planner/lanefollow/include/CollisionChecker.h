/*
***********************************************************************
* CollisionChecker.h:
* Collision detection
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _COLLISIONCHECKER_H_
#define _COLLISIONCHECKER_H_

#include "LatticePlannerdata.h"
#include "common/logging/include/easylogging++.h"

namespace ASV::planning {
class CollisionChecker {
 public:
  CollisionChecker(const CollisionData &_CollisionData)
      : collisiondata(_CollisionData) {}
  virtual ~CollisionChecker() {}

  std::vector<Frenet_path> check_paths(
      const std::vector<Frenet_path> &_frenet_lattice) {
    std::vector<Frenet_path> t_roi_paths;
    std::vector<Frenet_path> t_greedy_roi_paths;
    std::vector<Frenet_path> tw_greedy_roi_paths;

    std::size_t count_collsion = 0;

    for (std::size_t i = 0; i != _frenet_lattice.size(); i++) {
      int results = check_collision(_frenet_lattice[i]);

      if (results == 2) {
        count_collsion++;
        continue;  // collision occurs
      } else if (results == 1)
        tw_greedy_roi_paths.push_back(
            _frenet_lattice[i]);  // emplace_back is better
      else {
        tw_greedy_roi_paths.push_back(_frenet_lattice[i]);
        t_greedy_roi_paths.push_back(_frenet_lattice[i]);
      }
    }
    // std::cout << t_greedy_roi_paths.size() << " " <<
    // tw_greedy_roi_paths.size()
    //           << std::endl;

    if (t_greedy_roi_paths.size() == 0) {
      t_greedy_roi_paths = tw_greedy_roi_paths;
      CLOG(ERROR, "Frenet") << "Collision will occur";  // TODO: Scenario switch
    }

    t_roi_paths = check_constraints(t_greedy_roi_paths);

    if (t_roi_paths.size() == 0) t_roi_paths = t_greedy_roi_paths;

    return t_roi_paths;
  }  // check_paths

  std::vector<Frenet_path> check_constraints(
      const std::vector<Frenet_path> &_frenet_lattice) {
    std::vector<Frenet_path> t_roi_paths;

    for (std::size_t i = 0; i != _frenet_lattice.size(); i++) {
      if (_frenet_lattice[i].speed.maxCoeff() > collisiondata.MAX_SPEED) {
        continue;  // max speed check
      }
      if ((_frenet_lattice[i].dspeed.maxCoeff() > collisiondata.MAX_ACCEL) ||
          (_frenet_lattice[i].dspeed.minCoeff() < collisiondata.MIN_ACCEL)) {
        continue;  // Max accel check
      }
      if ((_frenet_lattice[i].kappa.maxCoeff() > collisiondata.MAX_CURVATURE) ||
          (_frenet_lattice[i].kappa.minCoeff() <
           -collisiondata.MAX_CURVATURE)) {
        continue;  // Max curvature check
      }
      t_roi_paths.emplace_back(_frenet_lattice[i]);
    }
  }  // check_constraints

  void setobstacle(const Eigen::VectorXd &_obstacle_x,
                   const Eigen::VectorXd &_obstacle_y) noexcept {
    obstacle_x = _obstacle_x;
    obstacle_y = _obstacle_y;
  }  // setobstacle

 protected:
  int check_collision(const Frenet_path &_Frenet_path) {
    std::size_t max_n_obstacle = static_cast<std::size_t>(obstacle_x.size());
    std::size_t num_path_point =
        static_cast<std::size_t>(_Frenet_path.x.size());

    double min_dist = 10000;
    double min_radius = std::pow(collisiondata.ROBOT_RADIUS, 2);
    for (std::size_t i = 0; i != max_n_obstacle; i++) {
      for (std::size_t j = 0; j != num_path_point; j++) {
        double _dis = std::pow(_Frenet_path.x(j) - obstacle_x(i), 2) +
                      std::pow(_Frenet_path.y(j) - obstacle_y(i), 2);
        if (_dis < min_dist) min_dist = _dis;
      }
    }
    if (min_dist <= 0.5 * min_radius) return 2;
    if (min_dist <= min_radius)  // collision occurs
      return 1;
    return 0;
  }  // check_collision

  // obstacles
  Eigen::VectorXd obstacle_x;
  Eigen::VectorXd obstacle_y;

 private:
  CollisionData collisiondata;
};  // end class CollisionChecker
}  // namespace ASV::planning

#endif /* _COLLISIONCHECKER_H_ */