/*
***********************************************************************
* FrenetTrajectoryGenerator.h:
* Frenet optimal trajectory generator
* This header file can be read by C++ compilers
*
* Ref: "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
* Frame", Moritz Werling et al.

* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _FRENETTRAJECTORYGENERATOR_H_
#define _FRENETTRAJECTORYGENERATOR_H_

#include "easylogging++.h"
#include "planner_util.h"
#include "plannerdata.h"

namespace ASV {

struct Frenet_path {
  Eigen::VectorXd t;
  Eigen::VectorXd d;
  Eigen::VectorXd d_dot;
  Eigen::VectorXd d_ddot;
  Eigen::VectorXd s;
  Eigen::VectorXd s_dot;
  Eigen::VectorXd s_ddot;
  Eigen::VectorXd d_prime;
  Eigen::VectorXd d_pprime;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd yaw;
  Eigen::VectorXd kappa;
  Eigen::VectorXd speed;
  Eigen::VectorXd dspeed;
  double cd;
  double cv;
  double cf;
};

class FrenetTrajectoryGenerator {
  enum FrenetScenarios {
    FOLLOWing = 1,
    MERGING = 2,
    STOPPING = 3,
    VELOCITYKEEPING = 4
  };

 public:
  FrenetTrajectoryGenerator(
      const Frenetdata &_Frenetdata,
      const Eigen::VectorXd &_wx = Eigen::VectorXd::LinSpaced(5, 0, 5),
      const Eigen::VectorXd &_wy = Eigen::VectorXd::LinSpaced(5, 0, 0))
      : frenetdata(_Frenetdata),
        n_di(0),
        n_Tj(0),
        tvk(0),
        target_Spline2D(_wx, _wy),
        next_cartesianstate(CartesianState{
            0,            // x
            0,            // y
            -M_PI / 3.0,  // theta
            0,            // kappa
            1,            // speed
            0,            // dspeed
        }),
        current_frenetstate(FrenetState{
            0,  // s
            0,  // s_dot
            0,  // s_ddot
            2,  // d
            0,  // d_dot
            0,  // d_ddot
            0,  // d_prime
            0   // d_pprime
        }) {
    setup_target_course();
    initialize_endcondition_FrenetLattice();
  }
  virtual ~FrenetTrajectoryGenerator() = default;

  FrenetTrajectoryGenerator &trajectoryonestep(double marine_x, double marine_y,
                                               double marine_theta,
                                               double marine_kappa,
                                               double marine_speed,
                                               double marine_a,
                                               double _targetspeed) {
    // static int i = 0;
    CartesianState _cart_state{marine_x,     marine_y,     marine_theta,
                               marine_kappa, marine_speed, marine_a};

    Marine2Cart(marine_y, marine_theta, marine_kappa, _cart_state.y,
                _cart_state.theta, _cart_state.kappa);
    Cart2Frenet(_cart_state, current_frenetstate);

    // std::cout << i << " After conversion\n";
    // std::cout << "speed: " << current_frenetstate.s_dot << std::endl;
    // std::cout << "d: " << current_frenetstate.d << std::endl;
    // std::cout << "d_dot: " << current_frenetstate.d_dot << std::endl;
    // std::cout << "d_ddot: " << current_frenetstate.d_ddot << std::endl;
    // std::cout << "s: " << current_frenetstate.s << std::endl;

    frenet_optimal_planning(current_frenetstate.d, current_frenetstate.d_dot,
                            current_frenetstate.d_ddot, current_frenetstate.s,
                            current_frenetstate.s_dot,
                            current_frenetstate.s_ddot, _targetspeed);

    // std::cout << " conversion\n ";
    // std::cout << "speed: " << mincost_path.speed(1) << std::endl;
    // std::cout << "d: " << mincost_path.d(1) << std::endl;
    // std::cout << "d_dot: " << mincost_path.d_dot(1) << std::endl;
    // std::cout << "d_ddot: " << mincost_path.d_ddot(1) << std::endl;
    // std::cout << "s: " << mincost_path.s(1) << std::endl;

    updateNextCartesianStatus();

    // static FrenetState frenetstate{
    //     0,  // s
    //     0,  // s_dot
    //     0,  // s_ddot
    //     0,  // d
    //     0,  // d_dot
    //     0,  // d_ddot
    //     0,  // d_prime
    //     0   // d_pprime
    // };

    // static double t_speed = 1;

    // frenet_optimal_planning(t_speed, frenetstate.d, frenetstate.d_dot,
    //                         frenetstate.d_ddot, frenetstate.s, _targetspeed);

    // t_speed = mincost_path.s_dot(1);
    // frenetstate.d = mincost_path.d(1);
    // frenetstate.d_dot = mincost_path.d_dot(1);
    // frenetstate.d_ddot = mincost_path.d_ddot(1);
    // frenetstate.s = mincost_path.s(1);

    // std::cout << "Without conversion\n ";
    // std::cout << "speed: " << mincost_path.s_dot(1) << std::endl;
    // std::cout << "d: " << mincost_path.d(1) << std::endl;
    // std::cout << "d_dot: " << mincost_path.d_dot(1) << std::endl;
    // std::cout << "d_ddot: " << mincost_path.d_ddot(1) << std::endl;
    // std::cout << "s: " << mincost_path.s(1) << std::endl;

    // ++i;
    return *this;
  }  // trajectoryonestep

  // setup a new targe course and re-generate it
  void regenerate_target_course(const Eigen::VectorXd &_marine_wx,
                                const Eigen::VectorXd &_marine_wy) {
    auto cart_wy = Marine2Cart(_marine_wy);
    target_Spline2D.reinterpolation(_marine_wx, cart_wy);  // cart_wy=marine_wx
    setup_target_course();
  }  // regenerate_target_course

  Eigen::VectorXd getCartRefX() const noexcept { return cart_RefX; }
  Eigen::VectorXd getCartRefY() const noexcept { return cart_RefY; }
  Eigen::VectorXd getRefHeading() const noexcept { return RefHeading; }
  Eigen::VectorXd getRefKappa() const noexcept { return RefKappa; }
  Eigen::VectorXd getbestX() const noexcept { return mincost_path.x; }
  Eigen::VectorXd getbestY() const noexcept { return mincost_path.y; }

  void setobstacle(const Eigen::VectorXd &_obstacle_x,
                   const Eigen::VectorXd &_obstacle_y) noexcept {
    obstacle_x = _obstacle_x;
    obstacle_y = _obstacle_y;
  }  // setobstacle

  CartesianState getnextcartesianstate() const noexcept {
    return next_cartesianstate;
  }

 private:
  // constant data in Frenet trajectory generator
  Frenetdata frenetdata;
  // end conditions of Frenet Lattice
  std::size_t n_di;
  std::size_t n_Tj;
  std::size_t n_tvk;
  Eigen::VectorXd di;
  Eigen::VectorXd Tj;
  Eigen::VectorXd tvk;
  // Frenet lattice
  std::vector<Frenet_path> frenet_paths;
  Frenet_path mincost_path;
  // center line
  Spline2D target_Spline2D;
  Eigen::VectorXd Frenet_s;        // arclength (m)
  Eigen::VectorXd cart_RefX;       // reference x (m)
  Eigen::VectorXd cart_RefY;       // reference y (m)
  Eigen::VectorXd RefHeading;      // reference yaw (rad)
  Eigen::VectorXd RefKappa;        // reference curvature
  Eigen::VectorXd RefKappa_prime;  // reference dk/ds

  // obstacles
  Eigen::VectorXd obstacle_x;
  Eigen::VectorXd obstacle_y;

  // real time data
  CartesianState next_cartesianstate;
  FrenetState current_frenetstate;
  // cost weights
  const double KJ = 0.1;
  const double KT = 0.1;
  const double KD = 1.0;
  const double KLAT = 1.0;
  const double KLON = 1.0;

  void frenet_optimal_planning(double _c_d, double _c_d_dot, double _c_d_ddot,
                               double _s, double _s_dot, double _s_ddot,
                               double _target_s_dot) {
    update_endcondition_FrenetLattice(_target_s_dot);

    calc_frenet_lattice(_c_d, _c_d_dot, _c_d_ddot, _s, _s_dot, _s_ddot,
                        _target_s_dot);

    auto t_frenet_paths = check_paths();

    // find minimum cost path
    mincost_path = findmincostpath(t_frenet_paths);

  }  // frenet_optimal_planning

  Frenet_path findmincostpath(const std::vector<Frenet_path> &_frenetpaths) {
    // find minimum cost path
    double mincost = 1e6;
    Frenet_path _best_path;
    for (std::size_t i = 0; i != _frenetpaths.size(); i++) {
      if (mincost > _frenetpaths[i].cf) {
        mincost = _frenetpaths[i].cf;
        _best_path = _frenetpaths[i];
      }
    }
    return _best_path;
  }

  void updateNextCartesianStatus(double _target_s_dot = 1.0) {
    // The results of Frenet generation at "DT"
    const int index = 8;

    // int index = static_cast<int>(frenetdata.HULL_LENGTH /
    //                              (_target_s_dot * frenetdata.SAMPLE_TIME));
    next_cartesianstate.x = mincost_path.x(index);
    next_cartesianstate.y = mincost_path.y(index);
    next_cartesianstate.theta = mincost_path.yaw(index);
    next_cartesianstate.kappa = mincost_path.kappa(index);
    next_cartesianstate.speed = mincost_path.speed(index);
    next_cartesianstate.dspeed = mincost_path.dspeed(index);

  }  // updateNextCartesianStatus

  std::vector<Frenet_path> check_paths() {
    std::vector<Frenet_path> t_roi_paths;
    std::vector<Frenet_path> t_greedy_roi_paths;

    std::size_t count_max_speed = 0;
    std::size_t count_max_accel = 0;
    std::size_t count_max_curvature = 0;
    std::size_t count_collsion = 0;

    for (std::size_t i = 0; i != frenet_paths.size(); i++) {
      if (!check_collision(frenet_paths[i])) {
        count_collsion++;
        continue;  // collision occurs
      }
      t_greedy_roi_paths.push_back(frenet_paths[i]);
    }

    for (std::size_t i = 0; i != t_greedy_roi_paths.size(); i++) {
      if (t_greedy_roi_paths[i].speed.maxCoeff() > frenetdata.MAX_SPEED) {
        count_max_speed++;
        continue;  // max speed check
      }
      if ((t_greedy_roi_paths[i].dspeed.maxCoeff() > frenetdata.MAX_ACCEL) ||
          (t_greedy_roi_paths[i].dspeed.minCoeff() < frenetdata.MIN_ACCEL)) {
        count_max_accel++;
        continue;  // Max accel check
      }
      if ((t_greedy_roi_paths[i].kappa.maxCoeff() > frenetdata.MAX_CURVATURE) ||
          (t_greedy_roi_paths[i].kappa.minCoeff() <
           -frenetdata.MAX_CURVATURE)) {
        count_max_curvature++;
        continue;  // Max curvature check
      }
      t_roi_paths.push_back(t_greedy_roi_paths[i]);
    }

    if (t_greedy_roi_paths.size() == 0)
      CLOG(ERROR, "Frenet") << "Collision will occur";  // TODO: Scenario switch

    if (t_roi_paths.size() == 0) t_roi_paths = t_greedy_roi_paths;
    // std::cout << frenet_paths.size() << std::endl;
    // std::cout << "Max speed: " << count_max_speed << std::endl;
    // std::cout << "Max accel: " << count_max_accel << std::endl;
    // std::cout << "Max cuvature: " << count_max_curvature << std::endl;
    // std::cout << "collision: " << count_collsion << std::endl;
    return t_roi_paths;
  }  // check_paths

  bool check_collision(const Frenet_path &_Frenet_path) {
    std::size_t max_n_obstacle = static_cast<std::size_t>(obstacle_x.size());
    std::size_t num_path_point =
        static_cast<std::size_t>(_Frenet_path.x.size());

    for (std::size_t i = 0; i != max_n_obstacle; i++) {
      for (std::size_t j = 0; j != num_path_point; j++) {
        double _dis = std::pow(_Frenet_path.x(j) - obstacle_x(i), 2) +
                      std::pow(_Frenet_path.y(j) - obstacle_y(i), 2);
        if (_dis <= std::pow(frenetdata.ROBOT_RADIUS, 2))  // collision occurs
          return false;
      }
    }
    return true;
  }  // check_collision

  // assume that target_spline2d is known, we can interpolate the spline2d to
  // obtain the associated (s,x,y,theta, kappa)
  void setup_target_course() {
    Eigen::VectorXd s = target_Spline2D.getarclength();
    std::size_t n =
        1 + static_cast<std::size_t>(s(s.size() - 1) /
                                     frenetdata.TARGET_COURSE_ARC_STEP);

    Frenet_s.resize(n);
    cart_RefX.resize(n);
    cart_RefY.resize(n);
    RefHeading.resize(n);
    RefKappa.resize(n);
    RefKappa_prime.resize(n);

    for (std::size_t i = 0; i != n; i++) {
      Frenet_s(i) = frenetdata.TARGET_COURSE_ARC_STEP * i;
      Eigen::Vector2d position = target_Spline2D.compute_position(Frenet_s(i));
      cart_RefX(i) = position(0);
      cart_RefY(i) = position(1);
      RefKappa(i) = target_Spline2D.compute_curvature(Frenet_s(i));
      RefHeading(i) = target_Spline2D.compute_yaw(Frenet_s(i));
      RefKappa_prime(i) = target_Spline2D.compute_dcurvature(Frenet_s(i));
    }
  }  // setup_target_course

  void initialize_endcondition_FrenetLattice() {
    n_di = static_cast<std::size_t>(
        2 * frenetdata.MAX_ROAD_WIDTH / frenetdata.ROAD_WIDTH_STEP + 1);
    n_Tj = static_cast<std::size_t>(
        (frenetdata.MAXT - frenetdata.MINT) / frenetdata.DT + 1);
    n_tvk = static_cast<std::size_t>(
        2 * frenetdata.MAX_SPEED_DEVIATION / frenetdata.TRAGET_SPEED_STEP + 1);

    n_di = static_cast<std::size_t>(
        2 * frenetdata.MAX_ROAD_WIDTH / frenetdata.ROAD_WIDTH_STEP + 1);
    di = Eigen::VectorXd::LinSpaced(n_di, -frenetdata.MAX_ROAD_WIDTH,
                                    frenetdata.MAX_ROAD_WIDTH);
    Tj = Eigen::VectorXd::LinSpaced(n_Tj, frenetdata.MINT, frenetdata.MAXT);
    tvk = Eigen::VectorXd::LinSpaced(n_tvk, 5 - frenetdata.MAX_SPEED_DEVIATION,
                                     5 + frenetdata.MAX_SPEED_DEVIATION);
  }  // initialize_frenet_paths

  void update_endcondition_FrenetLattice(double _target_s_dot) {
    tvk = Eigen::VectorXd::LinSpaced(
        n_tvk, _target_s_dot - frenetdata.MAX_SPEED_DEVIATION,
        _target_s_dot + frenetdata.MAX_SPEED_DEVIATION);
  }  // initialize_frenet_paths

  void calc_frenet_lattice(double _d,                   // current d(t)
                           double _d_dot,               // current d(d(t))/dt
                           double _d_ddot,              // current
                           double _s,                   // current arclength
                           double _s_dot,               // current ds/dt
                           double _s_ddot,              // current d(s_dot)/dt
                           double _target_s_dot,        // target speed,
                           double _target_s_ddot = 0.0  //
  ) {
    quintic_polynomial _quintic_polynomial;
    quartic_polynomial _quartic_polynomial;

    frenet_paths.clear();
    frenet_paths.reserve(n_di * n_Tj * n_tvk);

    // generate path to each offset goal
    for (std::size_t i = 0; i != n_di; i++) {
      // Lateral motion planning
      for (std::size_t j = 0; j != n_Tj; j++) {
        _quintic_polynomial.update_startendposition(_d, _d_dot, _d_ddot, di(i),
                                                    0.0, 0.0, Tj(j));
        std::size_t n_zero_Tj =
            static_cast<std::size_t>(std::ceil(Tj(j) / frenetdata.DT + 1));
        Eigen::VectorXd _t = Eigen::VectorXd::LinSpaced(n_zero_Tj, 0.0, Tj(j));
        Eigen::VectorXd t_d(n_zero_Tj);
        Eigen::VectorXd t_d_dot(n_zero_Tj);
        Eigen::VectorXd t_d_ddot(n_zero_Tj);
        Eigen::VectorXd t_d_dddot(n_zero_Tj);

        for (std::size_t ji = 0; ji != n_zero_Tj; ji++) {
          t_d(ji) = _quintic_polynomial.compute_order_derivative<0>(_t(ji));
          t_d_dot(ji) = _quintic_polynomial.compute_order_derivative<1>(_t(ji));
          t_d_ddot(ji) =
              _quintic_polynomial.compute_order_derivative<2>(_t(ji));
          t_d_dddot(ji) =
              _quintic_polynomial.compute_order_derivative<3>(_t(ji));
        }

        // Longitudinal motion planning (Velocity keeping)
        for (std::size_t k = 0; k != n_tvk; k++) {
          _quartic_polynomial.update_startendposition(
              _s, _s_dot, _s_ddot, tvk(k), _target_s_ddot, Tj(j));
          Eigen::VectorXd t_s(n_zero_Tj);
          Eigen::VectorXd t_s_dot(n_zero_Tj);
          Eigen::VectorXd t_s_ddot(n_zero_Tj);
          Eigen::VectorXd t_s_dddot(n_zero_Tj);
          Eigen::VectorXd t_d_prime(n_zero_Tj);
          Eigen::VectorXd t_d_pprime(n_zero_Tj);

          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            t_s(ki) = _quartic_polynomial.compute_order_derivative<0>(_t(ki));
            t_s_dot(ki) =
                _quartic_polynomial.compute_order_derivative<1>(_t(ki));
            t_s_ddot(ki) =
                _quartic_polynomial.compute_order_derivative<2>(_t(ki));
            t_s_dddot(ki) =
                _quartic_polynomial.compute_order_derivative<3>(_t(ki));
            t_d_prime(ki) = t_d_dot(ki) / t_s_dot(ki);
            t_d_pprime(ki) =
                (t_d_ddot(ki) * t_s_dot(ki) - t_s_ddot(ki) * t_d_dot(ki)) /
                std::pow(t_s_dot(ki), 3);
          }

          double Jp = t_d_dddot.squaredNorm();  // square of jerk
          double Js = t_s_dddot.squaredNorm();  // square of jerk

          // square of diff from target speed
          double _cd =
              KJ * Jp + KT * Tj(j) + KD * std::pow(t_d(n_zero_Tj - 1), 2);
          double _cv =
              KJ * Js + KT * Tj(j) +
              KD * std::pow((_target_s_dot - t_s_dot(n_zero_Tj - 1)), 2);
          double _cf = KLAT * _cd + KLON * _cv;

          // calc global positions;
          Eigen::VectorXd t_x(n_zero_Tj);       // global x
          Eigen::VectorXd t_y(n_zero_Tj);       // global y
          Eigen::VectorXd t_yaw(n_zero_Tj);     // heading
          Eigen::VectorXd t_kappa(n_zero_Tj);   // curvature
          Eigen::VectorXd t_speed(n_zero_Tj);   // speed
          Eigen::VectorXd t_dspeed(n_zero_Tj);  // dspeed

          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            CartesianState _cartesianstate{
                0,  // x
                0,  // y
                0,  // theta
                0,  // kappa
                0,  // speed
                0   // dspeed
            };
            Frenet2Cart(
                FrenetState{
                    t_s(ki),        // s
                    t_s_dot(ki),    // s_dot
                    t_s_ddot(ki),   // s_ddot
                    t_d(ki),        // d
                    t_d_dot(ki),    // d_dot
                    t_d_ddot(ki),   // d_ddot
                    t_d_prime(ki),  // d_prime
                    t_d_pprime(ki)  // d_pprime
                },
                _cartesianstate);

            t_x(ki) = _cartesianstate.x;
            t_y(ki) = _cartesianstate.y;
            t_yaw(ki) = _cartesianstate.theta;
            t_kappa(ki) = _cartesianstate.kappa;
            t_speed(ki) = _cartesianstate.speed;
            t_dspeed(ki) = _cartesianstate.dspeed;
          }

          // add to frenet_paths
          frenet_paths.push_back(Frenet_path{
              _t,          // vector of t
              t_d,         // vector of d
              t_d_dot,     // vector of dd/dt
              t_d_ddot,    // vector of d(d_dot)/dt
              t_s,         // vector of s
              t_s_dot,     // vector of ds/dt
              t_s_ddot,    // vector of d(s_dot)/dt
              t_d_prime,   // vector of dd/ds
              t_d_pprime,  // vector of d(d_prime)/ds
              t_x,         // vector of x;
              t_y,         // vector of y
              t_yaw,       // vector of yaw
              t_kappa,     // vector of kappa
              t_speed,     // vector of speed
              t_dspeed,    // vector of dspeed
              _cd,         // cd
              _cv,         // cv
              _cf          // cf
          });
        }
      }
    }
  }  // calc_frenet_lattice

  // find the closest point on the reference spline, given a position (x, y)
  std::size_t ClosestRefPoint(double _cart_vx, double _cart_vy,
                              const Eigen::VectorXd &_cart_rx,
                              const Eigen::VectorXd &_cart_ry) {
    static std::size_t n_closestrefpoint = 0;
    std::size_t previous_n_closestrefpoint = n_closestrefpoint;
    double mindistance = std::pow(_cart_vx - _cart_rx(n_closestrefpoint), 2) +
                         std::pow(_cart_vy - _cart_ry(n_closestrefpoint), 2);

    // TODO: decide iteration number
    std::size_t max_iteration =
        static_cast<std::size_t>(frenetdata.SAMPLE_TIME * frenetdata.MAX_SPEED /
                                 frenetdata.TARGET_COURSE_ARC_STEP);
    std::size_t max_index = static_cast<std::size_t>(_cart_rx.size());

    for (std::size_t i = 0; i != max_iteration; ++i) {
      std::size_t n_refpoint = previous_n_closestrefpoint + i;
      if (n_refpoint < max_index) {
        double t_distance = std::pow(_cart_vx - _cart_rx(n_refpoint), 2) +
                            std::pow(_cart_vy - _cart_ry(n_refpoint), 2);

        if (t_distance < mindistance) {
          mindistance = t_distance;
          n_closestrefpoint = n_refpoint;
        }
      } else
        continue;
    }

    // int n_closestrefpoint = 0;
    // double mindistance = 1e6;

    // for (int i = 0; i != _cart_rx.size(); ++i) {
    //   double t_distance = std::pow(_cart_vx - _cart_rx(i), 2) +
    //                       std::pow(_cart_vy - _cart_ry(i), 2);
    //   if (t_distance < mindistance) {
    //     mindistance = t_distance;
    //     n_closestrefpoint = i;
    //   } else
    //     continue;
    // }

    return n_closestrefpoint;
  }  // ClosestRefPoint

  // assume that we know the nearest arclength
  void Cart2Frenet(const CartesianState &_cartstate_v,
                   FrenetState &_frenetstate) {
    // compute the closest point on the center line
    std::size_t n_closestrefpoint =
        ClosestRefPoint(_cartstate_v.x, _cartstate_v.y, cart_RefX, cart_RefY);
    // arclength on center line
    _frenetstate.s = Frenet_s(n_closestrefpoint);
    // curvature of center line
    double ref_kappa = RefKappa(n_closestrefpoint);
    // dk/ds of center line
    double ref_kappa_prime = RefKappa_prime(n_closestrefpoint);
    // heading of center line
    double ref_heading = RefHeading(n_closestrefpoint);

    // delta theta: (TODO: | delta_theta | < pi/2 )
    double delta_theta = _cartstate_v.theta - ref_heading;
    if (std::abs(delta_theta) >= (0.5 * M_PI))
      CLOG(ERROR, "Frenet") << "extreme situations in heading";
    double cos_dtheta = std::cos(delta_theta);
    double sin_dtheta = std::sin(delta_theta);
    double tan_dtheta = std::tan(delta_theta);
    // d
    _frenetstate.d =
        std::cos(ref_heading) *
            (_cartstate_v.y - cart_RefY(n_closestrefpoint)) -
        std::sin(ref_heading) * (_cartstate_v.x - cart_RefX(n_closestrefpoint));
    // TODO: larger than zero
    double one_minus_kappa_r_d = 1 - ref_kappa * _frenetstate.d;
    if (one_minus_kappa_r_d <= 0)
      CLOG(ERROR, "Frenet") << "extreme situations in Lateral error";
    // ds/dt
    _frenetstate.s_dot = _cartstate_v.speed * cos_dtheta / one_minus_kappa_r_d;
    // dd/ds
    _frenetstate.d_prime = one_minus_kappa_r_d * tan_dtheta;
    // dd/dt
    _frenetstate.d_dot = _cartstate_v.speed * sin_dtheta;
    // d(d_prime)/ds
    const double kappa_r_d_prime =
        ref_kappa_prime * _frenetstate.d + ref_kappa * _frenetstate.d_prime;
    const double delta_theta_prime =
        _cartstate_v.kappa * one_minus_kappa_r_d / cos_dtheta - ref_kappa;
    _frenetstate.d_pprime =
        -kappa_r_d_prime * tan_dtheta +
        delta_theta_prime * one_minus_kappa_r_d / std::pow(cos_dtheta, 2);
    // d(s_dot) / dt
    _frenetstate.s_ddot =
        (_cartstate_v.dspeed * cos_dtheta -
         _frenetstate.s_dot * _frenetstate.s_dot *
             (_frenetstate.d_prime * delta_theta_prime - kappa_r_d_prime)) /
        one_minus_kappa_r_d;
    // d(d_dot) / dt
    _frenetstate.d_ddot =
        _frenetstate.d_pprime * _frenetstate.s_dot * _frenetstate.s_dot +
        _frenetstate.d_prime * _frenetstate.s_ddot;
  }  // Cart2Frenet

  // Transform from Frenet s,d coordinates to Cartesian x,y
  void Frenet2Cart(const FrenetState &_frenetstate,
                   CartesianState &_cartstate_v) {
    // calc global positions;
    double ref_heading = target_Spline2D.compute_yaw(_frenetstate.s);
    double ref_kappa = target_Spline2D.compute_curvature(_frenetstate.s);
    double ref_kappa_prime = target_Spline2D.compute_dcurvature(_frenetstate.s);

    auto _cart_position = CalculateCartesianPoint(
        ref_heading, _frenetstate.d,
        target_Spline2D.compute_position(_frenetstate.s));
    _cartstate_v.x = _cart_position(0);
    _cartstate_v.y = _cart_position(1);

    // TODO: larger than zero
    double one_minus_kappa_r_d = 1 - ref_kappa * _frenetstate.d;
    if (one_minus_kappa_r_d <= 0) CLOG(ERROR, "Frenet") << "extreme situations";
    // speed
    _cartstate_v.speed =
        std::sqrt(std::pow(_frenetstate.s_dot * one_minus_kappa_r_d, 2) +
                  std::pow(_frenetstate.d_dot, 2));

    // theta
    const double tan_delta_theta = _frenetstate.d_prime / one_minus_kappa_r_d;
    const double delta_theta =
        std::atan2(_frenetstate.d_prime, one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);
    _cartstate_v.theta = Normalizeheadingangle(delta_theta + ref_heading);

    // kappa
    const double kappa_r_d_prime =
        ref_kappa_prime * _frenetstate.d + ref_kappa * _frenetstate.d_prime;
    _cartstate_v.kappa =
        ((kappa_r_d_prime * tan_delta_theta + _frenetstate.d_pprime) *
             cos_delta_theta * cos_delta_theta / one_minus_kappa_r_d +
         ref_kappa) *
        cos_delta_theta / one_minus_kappa_r_d;
    // a
    const double delta_theta_prime =
        _cartstate_v.kappa * one_minus_kappa_r_d / cos_delta_theta - ref_kappa;

    _cartstate_v.dspeed =
        _frenetstate.s_ddot * one_minus_kappa_r_d / cos_delta_theta +
        _frenetstate.s_dot * _frenetstate.s_dot *
            (delta_theta_prime * _frenetstate.d_prime - kappa_r_d_prime) /
            cos_delta_theta;

  }  // Frenet2Cart

  double CalculateTheta(const double rtheta, const double rkappa,
                        const double d, const double d_prime) {
    return Normalizeheadingangle(rtheta + std::atan2(d_prime, 1 - d * rkappa));
  }
  double CalculateTheta(const double rtheta, const double d_dot,
                        const double vx) {
    return Normalizeheadingangle(rtheta + std::asin(d_dot / vx));
  }

  Eigen::Vector2d CalculateCartesianPoint(
      const double _rtheta, const double _d,
      const Eigen::Vector2d &_ref_position) {
    double _x = _ref_position(0) - _d * std::sin(_rtheta);
    double _y = _ref_position(1) + _d * std::cos(_rtheta);
    return (Eigen::Vector2d() << _x, _y).finished();
  }

 public:
  friend void transformf2c(FrenetTrajectoryGenerator &, const FrenetState &,
                           CartesianState &);
  friend void transformc2f(FrenetTrajectoryGenerator &, FrenetState &,
                           const CartesianState &);

};  // FrenetTrajectoryGenerator

void transformf2c(FrenetTrajectoryGenerator &_tg,
                  const FrenetState &_frenetstate, CartesianState &_cartstate) {
  _tg.Frenet2Cart(_frenetstate, _cartstate);
}
void transformc2f(FrenetTrajectoryGenerator &_tg, FrenetState &_frenetstate,
                  const CartesianState &_cartstate) {
  _tg.Cart2Frenet(_cartstate, _frenetstate);
}

}  // end namespace ASV

#endif /* _FRENETTRAJECTORYGENERATOR_H_*/