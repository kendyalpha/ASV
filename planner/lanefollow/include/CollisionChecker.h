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
    std::vector<Frenet_path> collision_free_roi_paths;
    std::vector<Frenet_path> sub_collision_free_roi_paths;
    std::vector<Frenet_path> constraint_free_paths =
        check_constraints(_frenet_lattice);

    for (std::size_t i = 0; i != constraint_free_paths.size(); i++) {
      int results = check_collision(constraint_free_paths[i]);

      if (results == 2) {
        continue;  // collision occurs
      } else if (results == 1)
        sub_collision_free_roi_paths.emplace_back(
            constraint_free_paths[i]);  // emplace_back is better
      else {
        sub_collision_free_roi_paths.emplace_back(constraint_free_paths[i]);
        collision_free_roi_paths.emplace_back(constraint_free_paths[i]);
      }
    }
    std::cout << collision_free_roi_paths.size() << " "
              << sub_collision_free_roi_paths.size() << std::endl;

    if (collision_free_roi_paths.size() == 0) {
      if (sub_collision_free_roi_paths.size() != 0) {
        collision_free_roi_paths = sub_collision_free_roi_paths;
        CLOG(ERROR, "Frenet Lattice")
            << "Reduce the collision radius";  // TODO: Scenario switch
      } else {
        collision_free_roi_paths = constraint_free_paths;
        CLOG(ERROR, "Frenet Lattice")
            << "Collision may occur";  // TODO: Scenario switch
      }
    }

    return collision_free_roi_paths;
  }  // check_paths

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

  std::vector<Frenet_path> check_constraints(
      const std::vector<Frenet_path> &_frenet_lattice) {
    std::vector<Frenet_path> constraint_free_paths;

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
      constraint_free_paths.emplace_back(_frenet_lattice[i]);
    }
    return constraint_free_paths;
  }

  // obstacles
  Eigen::VectorXd obstacle_x;
  Eigen::VectorXd obstacle_y;

 private:
  CollisionData collisiondata;
};  // end class CollisionChecker
}  // namespace ASV::planning

#endif /* _COLLISIONCHECKER_H_ */