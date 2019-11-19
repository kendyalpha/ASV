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
#include "modules/planner/common/include/planner_util.h"

namespace ASV::planning {
class CollisionChecker {
 public:
  CollisionChecker(const CollisionData &_CollisionData)
      : collisiondata(_CollisionData) {}
  virtual ~CollisionChecker() = default;

  std::vector<Frenet_path> check_paths(
      const std::vector<Frenet_path> &_frenet_lattice) {
    std::vector<Frenet_path> collision_free_roi_paths;
    std::vector<Frenet_path> sub_collision_free_roi_paths;
    std::vector<Frenet_path> constraint_free_paths = _frenet_lattice;
    // std::vector<Frenet_path> constraint_free_paths =
    //     check_constraints(_frenet_lattice);

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
    std::cout << constraint_free_paths.size() << " "
              << collision_free_roi_paths.size() << " "
              << sub_collision_free_roi_paths.size() << std::endl;

    if (collision_free_roi_paths.size() == 0) {
      if (sub_collision_free_roi_paths.size() != 0) {
        collision_free_roi_paths = sub_collision_free_roi_paths;
        CLOG(ERROR, "Frenet_Lattice")
            << "Reduce the collision radius";  // TODO: Scenario switch
      } else {
        collision_free_roi_paths = constraint_free_paths;
        CLOG(ERROR, "Frenet_Lattice")
            << "Collision may occur";  // TODO: Scenario switch
      }
    }

    return collision_free_roi_paths;
  }  // check_paths

  void setobstacle(const Eigen::VectorXd &_marine_obstacle_x,
                   const Eigen::VectorXd &_marine_obstacle_y) noexcept {
    auto cart_obstacle_y = common::math::Marine2Cart(_marine_obstacle_y);
    obstacle_x = _marine_obstacle_x;
    obstacle_y = cart_obstacle_y;
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

    std::size_t count_max_speed = 0;
    std::size_t count_max_accel = 0;
    std::size_t count_max_curvature = 0;

    for (std::size_t i = 0; i != _frenet_lattice.size(); i++) {
      if (_frenet_lattice[i].speed.maxCoeff() > collisiondata.MAX_SPEED) {
        count_max_speed++;
        continue;  // max speed check
      }
      if ((_frenet_lattice[i].dspeed.maxCoeff() > collisiondata.MAX_ACCEL) ||
          (_frenet_lattice[i].dspeed.minCoeff() < collisiondata.MIN_ACCEL)) {
        count_max_accel++;
        continue;  // Max accel check
      }
      if ((_frenet_lattice[i].kappa.maxCoeff() > collisiondata.MAX_CURVATURE) ||
          (_frenet_lattice[i].kappa.minCoeff() <
           -collisiondata.MAX_CURVATURE)) {
        count_max_curvature++;
        continue;  // Max curvature check
      }
      constraint_free_paths.emplace_back(_frenet_lattice[i]);
    }

    // std::cout << "max_speed " << count_max_speed << "max_accel "
    //           << count_max_accel << "MAX_CURVATURE: " << count_max_curvature
    //           << std::endl;
    return constraint_free_paths;
  }

  // obstacles
  Eigen::VectorXd obstacle_x;  // in the Cartesian coordinate
  Eigen::VectorXd obstacle_y;  // in the Cartesian coordinate

 private:
  CollisionData collisiondata;
};  // end class CollisionChecker
}  // namespace ASV::planning

#endif /* _COLLISIONCHECKER_H_ */