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

#include <limits>
#include "LatticePlannerdata.h"
#include "common/logging/include/easylogging++.h"
#include "modules/planner/common/include/planner_util.h"

namespace ASV::planning {

class FrenetTrajectoryGenerator {
  enum FrenetScenarios {
    FOLLOWing = 1,
    MERGING = 2,
    STOPPING = 3,
    VELOCITYKEEPING = 4
  };

 public:
  FrenetTrajectoryGenerator(
      const LatticeData &_Latticedata,
      const Eigen::VectorXd &_wx = Eigen::VectorXd::LinSpaced(5, 0, 5),
      const Eigen::VectorXd &_wy = Eigen::VectorXd::LinSpaced(5, 0, 0))
      : latticedata(_Latticedata),
        n_di(0),
        n_Tj(0),
        tvk(0),
        target_Spline2D(_wx, _wy),
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

  FrenetTrajectoryGenerator &Generate_Lattice(double marine_x, double marine_y,
                                              double marine_theta,
                                              double marine_kappa,
                                              double marine_speed,
                                              double marine_a,
                                              double _targetspeed) {
    // convert cartesian coordinate to Frenet coodinate
    CartesianState _cart_state{marine_x,     marine_y,     marine_theta,
                               marine_kappa, marine_speed, marine_a};

    std::tie(_cart_state.y, _cart_state.theta, _cart_state.kappa) =
        common::math::Marine2Cart(marine_y, marine_theta, marine_kappa);
    current_frenetstate = Cart2Frenet(_cart_state);

    // update the condition
    update_endcondition_FrenetLattice(_targetspeed);

    //
    calc_frenet_lattice(current_frenetstate.d, current_frenetstate.d_dot,
                        current_frenetstate.d_ddot, current_frenetstate.s,
                        current_frenetstate.s_dot, current_frenetstate.s_ddot,
                        _targetspeed);

    return *this;
  }  // Generate_Lattice

  // setup a new targe course and re-generate it
  void regenerate_target_course(const Eigen::VectorXd &_marine_wx,
                                const Eigen::VectorXd &_marine_wy) {
    auto cart_wy = common::math::Marine2Cart(_marine_wy);
    auto cart_wx = _marine_wx;

    // ensure the # of waypoints is larger than 2
    if (cart_wx.size() == 2) {
      cart_wx = (Eigen::VectorXd(3) << cart_wx(0),
                 0.5 * (cart_wx(0) + cart_wx(1)), cart_wx(1))
                    .finished();

      cart_wy = (Eigen::VectorXd(3) << cart_wy(0),
                 0.5 * (cart_wy(0) + cart_wy(1)), cart_wy(1))
                    .finished();
    }
    target_Spline2D.reinterpolation(cart_wx, cart_wy);
    setup_target_course();
  }  // regenerate_target_course

  Eigen::VectorXd getCartRefX() const noexcept { return cart_RefX; }
  Eigen::VectorXd getCartRefY() const noexcept { return cart_RefY; }
  Eigen::VectorXd getRefHeading() const noexcept { return RefHeading; }
  Eigen::VectorXd getRefKappa() const noexcept { return RefKappa; }

 protected:
  // Frenet lattice
  std::vector<Frenet_path> frenet_paths;

 private:
  // constant data in Frenet trajectory generator
  LatticeData latticedata;
  // end conditions of Frenet Lattice
  std::size_t n_di;
  std::size_t n_Tj;
  std::size_t n_tvk;
  Eigen::VectorXd di;  // in the Frenet coordinate
  Eigen::VectorXd Tj;
  Eigen::VectorXd tvk;
  // center line
  Spline2D target_Spline2D;
  Eigen::VectorXd Frenet_s;    // arclength (m)
  Eigen::VectorXd cart_RefX;   // reference x (m) in the Cartesian coordinate
  Eigen::VectorXd cart_RefY;   // reference y (m) in the Cartesian coordinate
  Eigen::VectorXd RefHeading;  // reference yaw (rad) in Cartesian coordinate
  Eigen::VectorXd RefKappa;    // reference curvature in Cartesian coordinate
  Eigen::VectorXd RefKappa_prime;  // reference dk/ds in Cartesian coordinate

  // real time data
  FrenetState current_frenetstate;  // in the Frenet coordinate
  // cost weights
  const double KJ = 0.1;
  const double KT = 0.1;
  const double KD = 4;
  const double KLAT = 1;
  const double KLON = 10;

  // assume that target_spline2d is known, we can interpolate the spline2d to
  // obtain the associated (s,x,y,theta, kappa)
  void setup_target_course() {
    Eigen::VectorXd s = target_Spline2D.getarclength();
    std::size_t n =
        1 + static_cast<std::size_t>(s(s.size() - 1) /
                                     latticedata.TARGET_COURSE_ARC_STEP);

    Frenet_s.resize(n);
    cart_RefX.resize(n);
    cart_RefY.resize(n);
    RefHeading.resize(n);
    RefKappa.resize(n);
    RefKappa_prime.resize(n);

    for (std::size_t i = 0; i != n; i++) {
      Frenet_s(i) = latticedata.TARGET_COURSE_ARC_STEP * i;
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
        2 * latticedata.MAX_ROAD_WIDTH / latticedata.ROAD_WIDTH_STEP + 1);
    n_Tj = static_cast<std::size_t>(
        (latticedata.MAXT - latticedata.MINT) / latticedata.DT + 1);
    n_tvk = static_cast<std::size_t>(2 * latticedata.MAX_SPEED_DEVIATION /
                                         latticedata.TRAGET_SPEED_STEP +
                                     1);

    di = Eigen::VectorXd::LinSpaced(n_di, -latticedata.MAX_ROAD_WIDTH,
                                    latticedata.MAX_ROAD_WIDTH);
    Tj = Eigen::VectorXd::LinSpaced(n_Tj, latticedata.MINT, latticedata.MAXT);
    tvk = Eigen::VectorXd::LinSpaced(n_tvk, 5 - latticedata.MAX_SPEED_DEVIATION,
                                     5 + latticedata.MAX_SPEED_DEVIATION);
  }  // initialize_frenet_paths

  void update_endcondition_FrenetLattice(double _target_s_dot) {
    tvk = Eigen::VectorXd::LinSpaced(
        n_tvk, _target_s_dot - latticedata.MAX_SPEED_DEVIATION,
        _target_s_dot + latticedata.MAX_SPEED_DEVIATION);
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
            static_cast<std::size_t>(std::ceil(Tj(j) / latticedata.DT + 1));
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

          // std::cout << "Jp: " << Jp << " Js: " << Js << " Tj: " << Tj(j)
          //           << " JD_d: " << std::pow(t_d(n_zero_Tj - 1), 2) << "
          //           JD_s: "
          //           << std::pow((_target_s_dot - t_s_dot(n_zero_Tj - 1)), 2)
          //           << std::endl;

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
            auto _cartesianstate = Frenet2Cart(FrenetState{
                t_s(ki),        // s
                t_s_dot(ki),    // s_dot
                t_s_ddot(ki),   // s_ddot
                t_d(ki),        // d
                t_d_dot(ki),    // d_dot
                t_d_ddot(ki),   // d_ddot
                t_d_prime(ki),  // d_prime
                t_d_pprime(ki)  // d_pprime
            });

            t_x(ki) = _cartesianstate.x;
            t_y(ki) = _cartesianstate.y;
            t_yaw(ki) = _cartesianstate.theta;
            t_kappa(ki) = _cartesianstate.kappa;
            t_speed(ki) = _cartesianstate.speed;
            t_dspeed(ki) = _cartesianstate.dspeed;
          }

          // add to frenet_paths
          frenet_paths.emplace_back(Frenet_path{
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
  int ClosestRefPoint(double _cart_vx, double _cart_vy,
                      const Eigen::VectorXd &_cart_rx,
                      const Eigen::VectorXd &_cart_ry) {
    // static std::size_t n_closestrefpoint = 0;
    // std::size_t previous_n_closestrefpoint = n_closestrefpoint;
    // double mindistance = std::pow(_cart_vx - _cart_rx(n_closestrefpoint), 2)
    // +
    //                      std::pow(_cart_vy - _cart_ry(n_closestrefpoint), 2);

    // // TODO: decide iteration number
    // std::size_t max_iteration = static_cast<std::size_t>(
    //     latticedata.SAMPLE_TIME * latticedata.MAX_SPEED /
    //     latticedata.TARGET_COURSE_ARC_STEP);
    // std::size_t max_index = static_cast<std::size_t>(_cart_rx.size());

    // for (std::size_t i = 0; i != max_iteration; ++i) {
    //   std::size_t n_refpoint = previous_n_closestrefpoint + i;
    //   if (n_refpoint < max_index) {
    //     double t_distance = std::pow(_cart_vx - _cart_rx(n_refpoint), 2) +
    //                         std::pow(_cart_vy - _cart_ry(n_refpoint), 2);

    //     if (t_distance < mindistance) {
    //       mindistance = t_distance;
    //       n_closestrefpoint = n_refpoint;
    //     }
    //   } else
    //     continue;
    // }

    int n_closestrefpoint = 0;
    double mindistance = std::numeric_limits<double>::max();

    for (int i = 0; i != _cart_rx.size(); ++i) {
      double t_distance = std::pow(_cart_vx - _cart_rx(i), 2) +
                          std::pow(_cart_vy - _cart_ry(i), 2);
      if (t_distance < mindistance) {
        mindistance = t_distance;
        n_closestrefpoint = i;
      } else
        continue;
    }

    return n_closestrefpoint;
  }  // ClosestRefPoint

  // assume that we know the nearest arclength
  FrenetState Cart2Frenet(const CartesianState &_cartstate_v) {
    FrenetState _frenetstate;
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

    return _frenetstate;
  }  // Cart2Frenet

  // Transform from Frenet s,d coordinates to Cartesian x,y
  CartesianState Frenet2Cart(const FrenetState &_frenetstate) {
    CartesianState _cartstate_v;
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
    _cartstate_v.theta =
        common::math::Normalizeheadingangle(delta_theta + ref_heading);

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

    return _cartstate_v;
  }  // Frenet2Cart

  double CalculateTheta(const double rtheta, const double rkappa,
                        const double d, const double d_prime) {
    return common::math::Normalizeheadingangle(
        rtheta + std::atan2(d_prime, 1 - d * rkappa));
  }
  double CalculateTheta(const double rtheta, const double d_dot,
                        const double vx) {
    return common::math::Normalizeheadingangle(rtheta + std::asin(d_dot / vx));
  }  // CalculateTheta

  Eigen::Vector2d CalculateCartesianPoint(
      const double _rtheta, const double _d,
      const Eigen::Vector2d &_ref_position) {
    double _x = _ref_position(0) - _d * std::sin(_rtheta);
    double _y = _ref_position(1) + _d * std::cos(_rtheta);
    return (Eigen::Vector2d() << _x, _y).finished();
  }  // CalculateCartesianPoint

 public:  // unit test for private function
  CartesianState Frenet2Cart_TEST(const FrenetState &_frenetstate) {
    return Frenet2Cart(_frenetstate);
  }
  FrenetState Cart2Frenet_TEST(const CartesianState &_cartstate) {
    return Cart2Frenet(_cartstate);
  }

};  // FrenetTrajectoryGenerator

}  // namespace ASV::planning

#endif /* _FRENETTRAJECTORYGENERATOR_H_*/