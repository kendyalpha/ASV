/*
***********************************************************************
* trajectorygenerator.h:
* Frenet optimal trajectory generator
* This header file can be read by C++ compilers
*
* Ref: "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
* Frame", Moritz Werling et al.

* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TRAJECTORYGENERATOR_H_
#define _TRAJECTORYGENERATOR_H_

#include "easylogging++.h"
#include "planner_util.h"
#include "plannerdata.h"

namespace ASV {

constexpr double TARGET_COURSE_ARC_STEP = 0.05;  //[m]
constexpr double SAMPLE_TIME = 0.1;              //[s]
constexpr double MAX_SPEED = 30;                 //[m/s]

struct Frenet_path {
  Eigen::VectorXd t;
  Eigen::VectorXd d;
  Eigen::VectorXd d_dot;
  Eigen::VectorXd d_ddot;
  Eigen::VectorXd d_dddot;
  Eigen::VectorXd s;
  Eigen::VectorXd s_dot;
  Eigen::VectorXd s_ddot;
  Eigen::VectorXd s_dddot;
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

struct parameters {
  double MAX_SPEED = 50.0 / 3.6;     // maximum speed [m/s]
  double MAX_ACCEL = 2.0;            // maximum acceleration [m/ss]
  double MAX_CURVATURE = 1.0;        // maximum curvature [1/m]
  double MAX_ROAD_WIDTH = 7.0;       // maximum road width [m]
  double D_ROAD_W = 1.0;             // road width sampling length [m]
  double DT = 0.2;                   // time tick [s]
  double MAXT = 5.0;                 // max prediction time [s]
  double MINT = 4.0;                 // min prediction time [s]
  double DS = 0.2;                   // [m]
  double MAXS = 5.0;                 // max arclength [m]
  double MINS = 4.0;                 // min arclength [m]
  double TARGET_SPEED = 30.0 / 3.6;  // target speed [m/s]
  double D_T_S = 5.0 / 3.6;          // target speed sampling length[m / s]
  double N_S_SAMPLE = 1;             // sampling number of target speed
  double ROBOT_RADIUS = 2.0;         // robot radius[m]

  // cost weights
  double KJ = 0.1;
  double KT = 0.1;
  double KD = 1.0;
  double KLAT = 1.0;
  double KLON = 1.0;
};

class trajectorygenerator {
  friend void transformf2c(trajectorygenerator &, const FrenetState &,
                           CartesianState &);
  friend void transformc2f(trajectorygenerator &, FrenetState &,
                           const CartesianState &);

 public:
  trajectorygenerator(const Eigen::VectorXd &_wx, const Eigen::VectorXd &_wy)
      : target_Spline2D(_wx, _wy) {
    setup_target_course();
    initialize_frenet_paths();
  }

  void trajectoryonestep() {
    frenet_optimal_planning(cartesianstate.speed, frenetstate.d,
                            frenetstate.d_dot, frenetstate.d_ddot,
                            frenetstate.s);
    updatecurrentstatus();
  }  // trajectoryonestep

  // setup a new targe course and re-generate it
  void regenerate_target_course(const Eigen::VectorXd &_wx,
                                const Eigen::VectorXd &_wy) {
    target_Spline2D.reinterpolation(_wx, _wy);
    setup_target_course();
  }  // regenerate_target_course

  Eigen::VectorXd getCartRefX() const noexcept { return cart_RefX; }
  Eigen::VectorXd getCartRefY() const noexcept { return cart_RefY; }
  Eigen::VectorXd getRefHeading() const noexcept { return RefHeading; }
  Eigen::VectorXd getRefKappa() const noexcept { return RefKappa; }
  Eigen::VectorXd getbestX() const noexcept { return _best_path.x; }
  Eigen::VectorXd getbestY() const noexcept { return _best_path.y; }

  double getcyaw() const noexcept { return _best_path.yaw(1); }

  void setobstacle(const Eigen::VectorXd &_obstacle_x,
                   const Eigen::VectorXd &_obstacle_y) noexcept {
    obstacle_x = _obstacle_x;
    obstacle_y = _obstacle_y;
  }  // setobstacle

  virtual ~trajectorygenerator() = default;

 private:
  std::vector<Frenet_path> frenet_paths;
  Frenet_path _best_path;
  std::size_t n_di;
  std::size_t n_Tj;
  std::size_t n_Sj;
  std::size_t n_tvk;
  Eigen::VectorXd di;
  Eigen::VectorXd Tj;
  Eigen::VectorXd tvk;

  Eigen::VectorXd obstacle_x;
  Eigen::VectorXd obstacle_y;

  // center line
  Spline2D target_Spline2D;
  Eigen::VectorXd Frenet_s;        // arclength (m)
  Eigen::VectorXd cart_RefX;       // reference x (m)
  Eigen::VectorXd cart_RefY;       // reference y (m)
  Eigen::VectorXd RefHeading;      // reference yaw (rad)
  Eigen::VectorXd RefKappa;        // reference curvature
  Eigen::VectorXd RefKappa_prime;  // reference dk/ds

  CartesianState cartesianstate{
      0,  // x
      0,  // y
      0,  // theta
      0,  // kappa
      3,  // speed
      0,  // dspeed
  };
  FrenetState frenetstate{
      0,  // s
      0,  // s_dot
      0,  // s_ddot
      2,  // d
      0,  // d_dot
      0,  // d_ddot
      0,  // d_prime
      0   // d_pprime
  };

  parameters _para;

  void frenet_optimal_planning(double _c_speed, double _c_d, double _c_d_d,
                               double _c_d_dd, double _s0) {
    calc_frenet_paths(_c_speed, _c_d, _c_d_d, _c_d_dd, _s0);

    auto t_frenet_paths = check_paths();

    // find minimum cost path
    double mincost = t_frenet_paths[0].cf;
    _best_path = t_frenet_paths[0];
    for (std::size_t i = 1; i != t_frenet_paths.size(); i++) {
      if (mincost > t_frenet_paths[i].cf) {
        mincost = t_frenet_paths[i].cf;
        _best_path = t_frenet_paths[i];
      }
    }
  }  // frenet_optimal_planning

  void updatecurrentstatus() {
    // TODO: cart2frenet

    cartesianstate.speed = _best_path.s_dot(1);
    frenetstate.d = _best_path.d(1);
    frenetstate.d_dot = _best_path.d_dot(1);
    frenetstate.d_ddot = _best_path.d_ddot(1);
    frenetstate.s = _best_path.s(1);

  }  // updatecurrentstatus

  std::vector<Frenet_path> check_paths() {
    std::vector<Frenet_path> t_frenet_paths;

    int count_max_speed = 0;
    int count_max_accel = 0;
    int count_max_curvature = 0;
    int count_collsion = 0;

    for (std::size_t i = 0; i != frenet_paths.size(); i++) {
      // std::cout << frenet_paths[i].c.transpose() << std::endl;

      if (frenet_paths[i].speed.maxCoeff() > _para.MAX_SPEED) {
        count_max_speed++;
        continue;  // max speed check
      }
      if ((frenet_paths[i].dspeed.maxCoeff() > _para.MAX_ACCEL) ||
          (frenet_paths[i].dspeed.minCoeff() < -_para.MAX_ACCEL)) {
        count_max_accel++;
        continue;  // Max accel check
      }
      if ((frenet_paths[i].kappa.maxCoeff() > _para.MAX_CURVATURE) ||
          (frenet_paths[i].kappa.minCoeff() < -_para.MAX_CURVATURE)) {
        count_max_curvature++;
        continue;  // Max curvature check
      }

      if (!check_collision(frenet_paths[i])) {
        count_collsion++;
        continue;  // collision occurs
      }
      t_frenet_paths.push_back(frenet_paths[i]);
    }
    // std::cout << frenet_paths.size() << std::endl;
    // std::cout << "Max speed: " << count_max_speed << std::endl;
    // std::cout << "Max accel: " << count_max_accel << std::endl;
    // std::cout << "Max cuvature: " << count_max_curvature << std::endl;
    // std::cout << "collision: " << count_collsion << std::endl;

    return t_frenet_paths;
  }  // check_paths

  bool check_collision(const Frenet_path &_Frenet_path) {
    for (int i = 0; i != obstacle_x.size(); i++) {
      for (int j = 0; j != _Frenet_path.x.size(); j++) {
        double _dis = std::pow(_Frenet_path.x(j) - obstacle_x(i), 2) +
                      std::pow(_Frenet_path.y(j) - obstacle_y(i), 2);
        if (_dis <= std::pow(_para.ROBOT_RADIUS, 2))  // collision occurs
          return false;
      }
    }
    return true;
  }  // check_collision

  // assume that target_spline2d is known, we can interpolate the spline2d to
  // obtain the associated (s,x,y,theta, kappa)
  void setup_target_course() {
    Eigen::VectorXd s = target_Spline2D.getarclength();
    int n = 1 + static_cast<int>(s(s.size() - 1) / TARGET_COURSE_ARC_STEP);

    Frenet_s.resize(n);
    cart_RefX.resize(n);
    cart_RefY.resize(n);
    RefHeading.resize(n);
    RefKappa.resize(n);
    RefKappa_prime.resize(n);

    for (int i = 0; i != n; i++) {
      Frenet_s(i) = TARGET_COURSE_ARC_STEP * i;
      Eigen::Vector2d position = target_Spline2D.compute_position(Frenet_s(i));
      cart_RefX(i) = position(0);
      cart_RefY(i) = position(1);
      RefKappa(i) = target_Spline2D.compute_curvature(Frenet_s(i));
      RefHeading(i) = target_Spline2D.compute_yaw(Frenet_s(i));
      RefKappa_prime(i) = target_Spline2D.compute_dcurvature(Frenet_s(i));
    }
  }  // setup_target_course

  void initialize_frenet_paths() {
    n_di =
        static_cast<std::size_t>(2 * _para.MAX_ROAD_WIDTH / _para.D_ROAD_W + 1);
    n_Tj = static_cast<std::size_t>((_para.MAXT - _para.MINT) / _para.DT + 1);
    n_Sj = static_cast<std::size_t>((_para.MAXT - _para.MINT) / _para.DT + 1);
    n_tvk = static_cast<std::size_t>(2 * _para.N_S_SAMPLE + 1);
    di = Eigen::VectorXd::LinSpaced(n_di, -_para.MAX_ROAD_WIDTH,
                                    _para.MAX_ROAD_WIDTH);
    Tj = Eigen::VectorXd::LinSpaced(n_Tj, _para.MINT, _para.MAXT);
    tvk = Eigen::VectorXd::LinSpaced(
        n_tvk, _para.TARGET_SPEED - _para.D_T_S * _para.N_S_SAMPLE,
        _para.TARGET_SPEED + _para.D_T_S * _para.N_S_SAMPLE);
  }  // initialize_frenet_paths

  void calc_frenet_paths(double _speed,   // current speed
                         double _d,       // current d(t)
                         double _d_dot,   // current d(d(t))/dt
                         double _d_ddot,  // current
                         double _s        // current arclength
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
            static_cast<std::size_t>(std::ceil(Tj(j) / _para.DT + 1));
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
          _quartic_polynomial.update_startendposition(_s, _speed, 0.0, tvk(k),
                                                      0.0, Tj(j));
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
          double _cd = _para.KJ * Jp + _para.KT * Tj(j) +
                       _para.KD * std::pow(t_d(n_zero_Tj - 1), 2);
          double _cv =
              _para.KJ * Js + _para.KT * Tj(j) +
              _para.KD *
                  std::pow((_para.TARGET_SPEED - t_s_dot(n_zero_Tj - 1)), 2);
          double _cf = _para.KLAT * _cd + _para.KLON * _cv;

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
            // add to frenet_paths
            FrenetState _frenetstate{
                t_s(ki),        // s
                t_s_dot(ki),    // s_dot
                t_s_ddot(ki),   // s_ddot
                t_d(ki),        // d
                t_d_dot(ki),    // d_dot
                t_d_ddot(ki),   // d_ddot
                t_d_prime(ki),  // d_prime
                t_d_pprime(ki)  // d_pprime
            };
            Frenet2Cart(_frenetstate, _cartesianstate);

            t_x(ki) = _cartesianstate.x;
            t_y(ki) = _cartesianstate.y;
            t_yaw(ki) = _cartesianstate.theta;
            t_kappa(ki) = _cartesianstate.kappa;
            t_speed(ki) = _cartesianstate.speed;
            t_dspeed(ki) = _cartesianstate.dspeed;
          }

          // add to frenet_paths
          Frenet_path _frenet_path{
              _t,          // vector of t
              t_d,         // vector of d
              t_d_dot,     // vector of dd/dt
              t_d_ddot,    // vector of d(d_dot)/dt
              t_d_dddot,   // vector of d(d_ddot)/dt
              t_s,         // vector of s
              t_s_dot,     // vector of ds/dt
              t_s_ddot,    // vector of d(s_dot)/dt
              t_s_dddot,   // vector of d(s_ddot)/dt
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
          };
          frenet_paths.push_back(_frenet_path);
        }
      }
    }
  }  // calc_frenet_paths

  // find the closest point on the reference spline, given a position (x, y)
  int ClosestRefPoint(double _cart_vx, double _cart_vy,
                      const Eigen::VectorXd &_cart_rx,
                      const Eigen::VectorXd &_cart_ry) {
    static int n_closestrefpoint = 0;
    int previous_n_closestrefpoint = n_closestrefpoint;
    double mindistance = std::pow(_cart_vx - _cart_rx(n_closestrefpoint), 2) +
                         std::pow(_cart_vy - _cart_ry(n_closestrefpoint), 2);

    // TODO: decide iteration number
    int max_iteration =
        static_cast<int>(SAMPLE_TIME * MAX_SPEED / TARGET_COURSE_ARC_STEP);

    for (int i = 0; i != max_iteration; ++i) {
      int n_refpoint = previous_n_closestrefpoint + i;
      if ((0 <= n_refpoint) && (n_refpoint < _cart_rx.size())) {
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
    int n_closestrefpoint =
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
      CLOG(ERROR, "Frenet") << "extreme situations";
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
    if (one_minus_kappa_r_d <= 0) CLOG(ERROR, "Frenet") << "extreme situations";
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

};  // trajectorygenerator

void transformf2c(trajectorygenerator &_tg, const FrenetState &_frenetstate,
                  CartesianState &_cartstate) {
  _tg.Frenet2Cart(_frenetstate, _cartstate);
}
void transformc2f(trajectorygenerator &_tg, FrenetState &_frenetstate,
                  const CartesianState &_cartstate) {
  _tg.Cart2Frenet(_cartstate, _frenetstate);
}

}  // end namespace ASV

#endif /* _TRAJECTORYGENERATOR_H_*/