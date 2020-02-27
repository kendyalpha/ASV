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
    // std::vector<Frenet_path> constraint_free_paths = _frenet_lattice;
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

  std::vector<double> getobstacle_x() const noexcept { return obstacle_x; }
  std::vector<double> getobstacle_y() const noexcept { return obstacle_y; }
  std::vector<double> getprevious_obstacle_x() const noexcept {
    return previous_obstacle_x;
  }
  std::vector<double> getprevious_obstacle_y() const noexcept {
    return previous_obstacle_y;
  }

 protected:
  // check if the surroundings will block the reference line: if true, the
  // surroundings will be obstacles, otherwise not.
  void IsObstacle(double surrounding_x, double surrounding_y,
                  const Eigen::VectorXd &_ref_x,
                  const Eigen::VectorXd &_ref_y) {
    // check the reference line
    if (check_reference(surrounding_x, surrounding_y, _ref_x, _ref_y)) return;

    // obstacle resolution
    double obstacle_resolution = 0.1 * std::pow(collisiondata.ROBOT_RADIUS, 2);
    for (std::size_t i = 0; i != obstacle_x.size(); ++i) {
      double distance =
          (obstacle_x[i] - surrounding_x) * (obstacle_x[i] - surrounding_x) +
          (obstacle_y[i] - surrounding_y) * (obstacle_y[i] - surrounding_y);
      if (distance < obstacle_resolution)
        return;
      else
        continue;
    }

    obstacle_x.push_back(surrounding_x);
    obstacle_y.push_back(surrounding_y);

  }  // IsObstacle

  bool check_reference(double surrounding_x, double surrounding_y,
                       const Eigen::VectorXd &_ref_x,
                       const Eigen::VectorXd &_ref_y) {
    // check the reference line
    double max_reference_radius = 9 * std::pow(collisiondata.ROBOT_RADIUS, 2);
    double min_dist = std::numeric_limits<double>::max();

    for (unsigned i = 0; i != _ref_x.size(); ++i) {
      double distance =
          (_ref_x(i) - surrounding_x) * (_ref_x(i) - surrounding_x) +
          (_ref_y(i) - surrounding_y) * (_ref_y(i) - surrounding_y);
      if (distance < min_dist) min_dist = distance;
    }
    if (min_dist > max_reference_radius)  // out of reference line
      return true;
    return false;
  }  // check_reference

  void update_obstacles(const std::vector<double> &_new_obstacle_x,
                        const std::vector<double> &_new_obstacle_y) {
    previous_obstacle_x = obstacle_x;
    previous_obstacle_y = obstacle_y;
    obstacle_x = _new_obstacle_x;
    obstacle_y = _new_obstacle_y;

  }  // update_obstacles

 private:
  CollisionData collisiondata;
  // obstacles (including static and dynamic ones)
  std::vector<double> previous_obstacle_x;  // in the Cartesian coordinate
  std::vector<double> previous_obstacle_y;  // in the Cartesian coordinate
  std::vector<double> obstacle_x;           // in the Cartesian coordinate
  std::vector<double> obstacle_y;           // in the Cartesian coordinate

  int check_collision(const Frenet_path &_Frenet_path) {
    std::size_t num_path_point =
        static_cast<std::size_t>(_Frenet_path.x.size());

    double min_dist = std::numeric_limits<double>::max();
    double min_radius = std::pow(collisiondata.ROBOT_RADIUS, 2);
    for (std::size_t j = 0; j != num_path_point; j++) {
      for (std::size_t i = 0; i != obstacle_x.size(); i++) {
        double _dis = std::pow(_Frenet_path.x(j) - obstacle_x[i], 2) +
                      std::pow(_Frenet_path.y(j) - obstacle_y[i], 2);

        if (_dis < min_dist) min_dist = _dis;
      }
      for (std::size_t i = 0; i != previous_obstacle_x.size(); i++) {
        double _dis = std::pow(_Frenet_path.x(j) - previous_obstacle_x[i], 2) +
                      std::pow(_Frenet_path.y(j) - previous_obstacle_x[i], 2);

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
    std::size_t count_max_angular_accel = 0;
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
      if ((_frenet_lattice[i].yaw_accel.maxCoeff() >
           collisiondata.MAX_ANG_ACCEL) ||
          (_frenet_lattice[i].yaw_accel.minCoeff() <
           collisiondata.MIN_ANG_ACCEL)) {
        count_max_angular_accel++;
        continue;  // Max heading acceleration check
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
  }  // check_constraints

};  // end class CollisionChecker
}  // namespace ASV::planning

#endif /* _COLLISIONCHECKER_H_ */