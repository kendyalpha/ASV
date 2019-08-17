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

#include "spline.h"

struct Frenet_path {
  Eigen::VectorXd t;
  Eigen::VectorXd d;
  Eigen::VectorXd d_d;
  Eigen::VectorXd d_dd;
  Eigen::VectorXd d_ddd;
  Eigen::VectorXd s;
  Eigen::VectorXd s_d;
  Eigen::VectorXd s_dd;
  Eigen::VectorXd s_ddd;
  double cd;
  double cv;
  double cf;

  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd yaw;
  Eigen::VectorXd ds;
  Eigen::VectorXd c;
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
  double DS = 0.2;                   //  [m]
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

struct currentstatus {
  double c_speed = 10.0 / 3.6;  // current speed (m/s)
  double c_d = 2.0;             // current lateral position (m)
  double c_d_d = 0.0;           // current lateral speed (m/s)
  double c_d_dd = 0.0;          // current latral acceleration (m/s)
  double s0 = 0.0;              // current course position
};

class quintic_polynomial final : public polynomialvalue<5> {
 public:
  quintic_polynomial(const Eigen::Matrix<double, 6, 1> &_a =
                         Eigen::Matrix<double, 6, 1>::Zero())
      : polynomialvalue(_a),
        start_x(0),
        start_vx(0),
        start_ax(0),
        end_x(0),
        end_vx(0),
        end_ax(0),
        T(0),
        x(Eigen::Vector3d::Zero()),
        b(Eigen::Vector3d::Zero()),
        A(Eigen::Matrix3d::Zero()) {}

  void update_startendposition(double _start_x, double _start_vx,
                               double _start_ax, double _end_x, double _end_vx,
                               double _end_ax, double _T) {
    start_x = _start_x;
    start_vx = _start_vx;
    start_ax = _start_ax;
    end_x = _end_x;
    end_vx = _end_vx;
    end_ax = _end_ax;
    T = _T;
    updatecoefficients();
  }

 private:
  double start_x;
  double start_vx;
  double start_ax;
  double end_x;
  double end_vx;
  double end_ax;
  double T;

  Eigen::Vector3d x;
  Eigen::Vector3d b;
  Eigen::Matrix3d A;

  void updatecoefficients() {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    A << T3, T4, T5,                                            // first row
        3 * T2, 4 * T3, 5 * T4,                                 // second row
        6 * T, 12 * T2, 20 * T3;                                // third row
    b << end_x - start_x - start_vx * T - 0.5 * start_ax * T2,  // first row
        end_vx - start_vx - start_ax * T,                       // second row
        end_ax - start_ax;                                      // third row
    x = A.householderQr().solve(b);
    a(0) = x(2);
    a(1) = x(1);
    a(2) = x(0);
    a(3) = 0.5 * start_ax;
    a(4) = start_vx;
    a(5) = start_x;
  }
};

class quartic_polynomial final : public polynomialvalue<4> {
 public:
  quartic_polynomial(const Eigen::Matrix<double, 5, 1> &_a =
                         Eigen::Matrix<double, 5, 1>::Zero())
      : polynomialvalue(_a),
        start_x(0),
        start_vx(0),
        start_ax(0),
        end_vx(0),
        end_ax(0),
        T(0),
        x(Eigen::Vector2d::Zero()),
        b(Eigen::Vector2d::Zero()),
        A(Eigen::Matrix2d::Zero()) {}

  void update_startendposition(double _start_x, double _start_vx,
                               double _start_ax, double _end_vx, double _end_ax,
                               double _T) {
    start_x = _start_x;
    start_vx = _start_vx;
    start_ax = _start_ax;
    end_vx = _end_vx;
    end_ax = _end_ax;
    T = _T;
    updatecoefficients();
  }

 private:
  double start_x;
  double start_vx;
  double start_ax;
  double end_vx;
  double end_ax;
  double T;

  Eigen::Vector2d x;
  Eigen::Vector2d b;
  Eigen::Matrix2d A;

  void updatecoefficients() {
    double T2 = T * T;
    double T3 = T2 * T;
    A << 3 * T2, 4 * T3,                    // first row
        6 * T, 12 * T2;                     // second row
    b << end_vx - start_vx - start_ax * T,  // first row
        end_ax - start_ax;                  // third row
    x = A.householderQr().solve(b);
    a(0) = x(1);
    a(1) = x(0);
    a(2) = 0.5 * start_ax;
    a(3) = start_vx;
    a(4) = start_x;
  }
};

class trajectorygenerator {
 public:
  trajectorygenerator(const Eigen::VectorXd &_wx, const Eigen::VectorXd &_wy)
      : waypoints_x(_wx), waypoints_y(_wy), target_Spline2D(_wx, _wy) {
    generate_target_course();
    initialize_frenet_paths();
  }

  void trajectoryonestep() {
    frenet_optimal_planning(_currentstatus.c_speed, _currentstatus.c_d,
                            _currentstatus.c_d_d, _currentstatus.c_d_dd,
                            _currentstatus.s0);
    updatecurrentstatus();
  }  // trajectoryonestep

  Eigen::VectorXd getrx() const noexcept { return rx; }
  Eigen::VectorXd getry() const noexcept { return ry; }
  Eigen::VectorXd getryaw() const noexcept { return ryaw; }
  Eigen::VectorXd getrk() const noexcept { return rk; }
  Eigen::VectorXd getbestX() const noexcept { return _best_path.x; }
  Eigen::VectorXd getbestY() const noexcept { return _best_path.y; }

  double getcyaw() const noexcept { return _best_path.yaw(1); }

  void setobstacle(const Eigen::VectorXd &_obstacle_x,
                   const Eigen::VectorXd &_obstacle_y) noexcept {
    obstacle_x = _obstacle_x;
    obstacle_y = _obstacle_y;
  }

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

  Eigen::VectorXd waypoints_x;
  Eigen::VectorXd waypoints_y;
  Eigen::VectorXd obstacle_x;
  Eigen::VectorXd obstacle_y;

  Spline2D target_Spline2D;
  Eigen::VectorXd rx;    // reference x (m)
  Eigen::VectorXd ry;    // reference y (m)
  Eigen::VectorXd ryaw;  // reference yaw (rad)
  Eigen::VectorXd rk;    // reference curvature ()

  currentstatus _currentstatus;
  parameters _para;

  void generate_target_course() {
    Eigen::VectorXd s = target_Spline2D.getarclength();

    double step = 0.05;
    int n = 1 + static_cast<int>(s(s.size() - 1) / step);
    rx.resize(n);
    ry.resize(n);
    ryaw.resize(n);
    rk.resize(n);

    double is = 0.0;
    for (int i = 0; i != n; i++) {
      is = step * i;
      Eigen::Vector2d position = target_Spline2D.compute_position(is);
      rx(i) = position(0);
      ry(i) = position(1);
      rk(i) = target_Spline2D.compute_curvature(is);
      ryaw(i) = target_Spline2D.compute_yaw(is);
    }
  }  // generate_target_course

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
    _currentstatus.c_speed = _best_path.s_d(1);
    _currentstatus.c_d = _best_path.d(1);
    _currentstatus.c_d_d = _best_path.d_d(1);
    _currentstatus.c_d_dd = _best_path.d_dd(1);
    _currentstatus.s0 = _best_path.s(1);
  }  // updatecurrentstatus

  std::vector<Frenet_path> check_paths() {
    std::vector<Frenet_path> t_frenet_paths;

    int count_max_speed = 0;
    int count_max_accel = 0;
    int count_max_curvature = 0;
    int count_collsion = 0;
    for (std::size_t i = 0; i != frenet_paths.size(); i++) {
      // std::cout << frenet_paths[i].c.transpose() << std::endl;

      if (frenet_paths[i].s_d.maxCoeff() > _para.MAX_SPEED) {
        count_max_speed++;
        continue;  // max speed check
      }
      if ((frenet_paths[i].s_dd.maxCoeff() > _para.MAX_ACCEL) ||
          (frenet_paths[i].s_dd.minCoeff() < -_para.MAX_ACCEL)) {
        count_max_accel++;
        continue;  // Max accel check
      }
      if ((frenet_paths[i].c.maxCoeff() > _para.MAX_CURVATURE) ||
          (frenet_paths[i].c.minCoeff() < -_para.MAX_CURVATURE)) {
        count_max_curvature++;
        continue;  // Max curvature check
      }

      if (!check_collision(frenet_paths[i])) {
        count_collsion++;
        continue;  // collision occurs
      }
      t_frenet_paths.push_back(frenet_paths[i]);
    }

    // std::cout << "Max speed: " << count_max_speed << std::endl;
    // std::cout << "Max accel: " << count_max_accel << std::endl;
    // std::cout << "Max cuvature: " << count_max_curvature << std::endl;
    // std::cout << "collision: " << count_collsion << std::endl;

    return t_frenet_paths;
  }  // check_paths

  bool check_collision(const Frenet_path &_Frenet_path) {
    for (int i = 0; i != obstacle_x.size(); i++) {
      for (int j = 0; j != _Frenet_path.x.size(); j++) {
        double d = std::pow(_Frenet_path.x(j) - obstacle_x(i), 2) +
                   std::pow(_Frenet_path.y(j) - obstacle_y(i), 2);
        if (d <= std::pow(_para.ROBOT_RADIUS, 2))  // collision occurs
          return false;
      }
    }
    return true;
  }  // check_collision

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

  void calc_frenet_paths(double _c_speed,  // current speed
                         double _c_d,      // current d(t)
                         double _c_d_d,    // current d(d(t))/dt
                         double _c_d_dd,   // current
                         double _s0        // current arclength
  ) {
    quintic_polynomial _quintic_polynomial;
    quartic_polynomial _quartic_polynomial;

    frenet_paths.clear();
    frenet_paths.reserve(n_di * n_Tj * n_tvk);

    // generate path to each offset goal
    for (std::size_t i = 0; i != n_di; i++) {
      // Lateral motion planning
      for (std::size_t j = 0; j != n_Tj; j++) {
        _quintic_polynomial.update_startendposition(_c_d, _c_d_d, _c_d_dd,
                                                    di(i), 0.0, 0.0, Tj(j));
        std::size_t n_zero_Tj =
            static_cast<std::size_t>(std::ceil(Tj(j) / _para.DT + 1));
        Eigen::VectorXd _t = Eigen::VectorXd::LinSpaced(n_zero_Tj, 0.0, Tj(j));
        Eigen::VectorXd _d(n_zero_Tj);
        Eigen::VectorXd _d_d(n_zero_Tj);
        Eigen::VectorXd _d_dd(n_zero_Tj);
        Eigen::VectorXd _d_ddd(n_zero_Tj);

        for (std::size_t ji = 0; ji != n_zero_Tj; ji++) {
          _d(ji) = _quintic_polynomial.compute_order_derivative<0>(_t(ji));
          _d_d(ji) = _quintic_polynomial.compute_order_derivative<1>(_t(ji));
          _d_dd(ji) = _quintic_polynomial.compute_order_derivative<2>(_t(ji));
          _d_ddd(ji) = _quintic_polynomial.compute_order_derivative<3>(_t(ji));
        }

        // Longitudinal motion planning (Velocity keeping)
        for (std::size_t k = 0; k != n_tvk; k++) {
          _quartic_polynomial.update_startendposition(_s0, _c_speed, 0.0,
                                                      tvk(k), 0.0, Tj(j));
          Eigen::VectorXd _s(n_zero_Tj);
          Eigen::VectorXd _s_d(n_zero_Tj);
          Eigen::VectorXd _s_dd(n_zero_Tj);
          Eigen::VectorXd _s_ddd(n_zero_Tj);
          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            _s(ki) = _quartic_polynomial.compute_order_derivative<0>(_t(ki));
            _s_d(ki) = _quartic_polynomial.compute_order_derivative<1>(_t(ki));
            _s_dd(ki) = _quartic_polynomial.compute_order_derivative<2>(_t(ki));
            _s_ddd(ki) =
                _quartic_polynomial.compute_order_derivative<3>(_t(ki));
          }
          double Jp = _d_ddd.squaredNorm();  // square of jerk
          double Js = _s_ddd.squaredNorm();  // square of jerk

          // square of diff from target speed
          double _cd = _para.KJ * Jp + _para.KT * Tj(j) +
                       _para.KD * std::pow(_d(n_zero_Tj - 1), 2);
          double _cv =
              _para.KJ * Js + _para.KT * Tj(j) +
              _para.KD *
                  std::pow((_para.TARGET_SPEED - _s_d(n_zero_Tj - 1)), 2);
          double _cf = _para.KLAT * _cd + _para.KLON * _cv;

          // calc global positions;
          Eigen::VectorXd _x(n_zero_Tj);      // global x
          Eigen::VectorXd _y(n_zero_Tj);      // global y
          Eigen::VectorXd _yaw(n_zero_Tj);    // heading
          Eigen::VectorXd _ds(n_zero_Tj);     // arc length
          Eigen::VectorXd _c(n_zero_Tj - 1);  // curvature
          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            Eigen::Vector2d ref_position =
                target_Spline2D.compute_position(_s(ki));
            double ref_yaw = target_Spline2D.compute_yaw(_s(ki));
            _x(ki) = ref_position(0) + _d(ki) * std::cos(ref_yaw + 0.5 * M_PI);
            _y(ki) = ref_position(1) + _d(ki) * std::sin(ref_yaw + 0.5 * M_PI);
          }
          // calc yaw and ds
          for (std::size_t ki = 0; ki != (n_zero_Tj - 1); ki++) {
            double dx = _x(ki + 1) - _x(ki);
            double dy = _y(ki + 1) - _y(ki);
            _yaw(ki) = std::atan2(dy, dx);
            _ds(ki) = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          }
          _yaw(n_zero_Tj - 1) = _yaw(n_zero_Tj - 2);
          _ds(n_zero_Tj - 1) = _ds(n_zero_Tj - 2);
          // calc curvature (Attention !!!!)
          for (std::size_t ki = 0; ki != (n_zero_Tj - 1); ki++) {
            _c(ki) = (_yaw(ki + 1) - _yaw(ki)) / _ds(ki);
          }

          // add to frenet_paths
          Frenet_path _frenet_path{
              _t,      //  t
              _d,      // d
              _d_d,    // d_d
              _d_dd,   // d_dd
              _d_ddd,  // d_ddd
              _s,      // s
              _s_d,    // s_d
              _s_dd,   // s_dd
              _s_ddd,  // s_ddd
              _cd,     // cd
              _cv,     // cv
              _cf,     // cf
              _x,      // x;
              _y,      // y
              _yaw,    // yaw
              _ds,     // ds
              _c       // c
          };
          frenet_paths.push_back(_frenet_path);
        }
      }
    }
  }  // calc_frenet_paths

  void calc_frenet_paths_lowspeed(
      double _c_speed,  // current speed
      double _c_d,      // current d(s)
      double _c_d_d,    // current d(s) (1st derivative)
      double _c_d_dd,   // current d(s) (2nd derivative)
      double _s0        //
  ) {
    quintic_polynomial _quintic_polynomial;
    quartic_polynomial _quartic_polynomial;

    frenet_paths.clear();
    frenet_paths.reserve(n_di * n_Tj * n_tvk);

    // generate path to each offset goal
    for (std::size_t i = 0; i != n_di; i++) {
      // Lateral motion planning
      for (std::size_t j = 0; j != n_Tj; j++) {
        std::size_t n_zero_Tj =
            static_cast<std::size_t>(std::ceil(Tj(j) / _para.DT + 1));
        Eigen::VectorXd _t = Eigen::VectorXd::LinSpaced(n_zero_Tj, 0.0, Tj(j));

        // Longitudinal motion planning (Velocity keeping)
        for (std::size_t k = 0; k != n_tvk; k++) {
          _quartic_polynomial.update_startendposition(_s0, _c_speed, 0.0,
                                                      tvk(k), 0.0, Tj(j));
          Eigen::VectorXd _s(n_zero_Tj);
          Eigen::VectorXd _s_d(n_zero_Tj);
          Eigen::VectorXd _s_dd(n_zero_Tj);
          Eigen::VectorXd _s_ddd(n_zero_Tj);
          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            _s(ki) = _quartic_polynomial.compute_order_derivative<0>(_t(ki));
            _s_d(ki) = _quartic_polynomial.compute_order_derivative<1>(_t(ki));
            _s_dd(ki) = _quartic_polynomial.compute_order_derivative<2>(_t(ki));
            _s_ddd(ki) =
                _quartic_polynomial.compute_order_derivative<3>(_t(ki));
          }
          // lateral movement (Low speed trajectories)
          Eigen::VectorXd _d(n_zero_Tj);
          Eigen::VectorXd _d_d(n_zero_Tj);
          Eigen::VectorXd _d_dd(n_zero_Tj);
          Eigen::VectorXd _d_ddd(n_zero_Tj);

          _quintic_polynomial.update_startendposition(_c_d, _c_d_d, _c_d_dd,
                                                      di(i), 0.0, 0.0, Tj(j));
          for (std::size_t ji = 0; ji != n_zero_Tj; ji++) {
            _d(ji) = _quintic_polynomial.compute_order_derivative<0>(_s(ji));
            _d_d(ji) = _quintic_polynomial.compute_order_derivative<1>(_s(ji));
            _d_dd(ji) = _quintic_polynomial.compute_order_derivative<2>(_s(ji));
            _d_ddd(ji) =
                _quintic_polynomial.compute_order_derivative<3>(_s(ji));
          }

          double Jp = _d_ddd.squaredNorm();  // square of jerk
          double Js = _s_ddd.squaredNorm();  // square of jerk

          // square of diff from target speed
          double _cd = _para.KJ * Jp + _para.KT * (_s(n_zero_Tj - 1) - s0) +
                       _para.KD * std::pow(_d(n_zero_Tj - 1), 2);
          double _cv =
              _para.KJ * Js + _para.KT * Tj(j) +
              _para.KD *
                  std::pow((_para.TARGET_SPEED - _s_d(n_zero_Tj - 1)), 2);
          double _cf = _para.KLAT * _cd + _para.KLON * _cv;

          // calc global positions;
          Eigen::VectorXd _x(n_zero_Tj);      // global x
          Eigen::VectorXd _y(n_zero_Tj);      // global y
          Eigen::VectorXd _yaw(n_zero_Tj);    // heading
          Eigen::VectorXd _ds(n_zero_Tj);     // arc length
          Eigen::VectorXd _c(n_zero_Tj - 1);  // curvature
          for (std::size_t ki = 0; ki != n_zero_Tj; ki++) {
            Eigen::Vector2d ref_position =
                target_Spline2D.compute_position(_s(ki));
            double ref_yaw = target_Spline2D.compute_yaw(_s(ki));
            _x(ki) = ref_position(0) + _d(ki) * std::cos(ref_yaw + 0.5 * M_PI);
            _y(ki) = ref_position(1) + _d(ki) * std::sin(ref_yaw + 0.5 * M_PI);
          }
          // calc yaw and ds
          for (std::size_t ki = 0; ki != (n_zero_Tj - 1); ki++) {
            double dx = _x(ki + 1) - _x(ki);
            double dy = _y(ki + 1) - _y(ki);
            _yaw(ki) = std::atan2(dy, dx);
            _ds(ki) = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          }
          _yaw(n_zero_Tj - 1) = _yaw(n_zero_Tj - 2);
          _ds(n_zero_Tj - 1) = _ds(n_zero_Tj - 2);
          // calc curvature (Attention !!!!)
          for (std::size_t ki = 0; ki != (n_zero_Tj - 1); ki++) {
            _c(ki) = (_yaw(ki + 1) - _yaw(ki)) / _ds(ki);
          }

          // add to frenet_paths
          Frenet_path _frenet_path{
              _t,      //  t
              _d,      // d
              _d_d,    // d_d
              _d_dd,   // d_dd
              _d_ddd,  // d_ddd
              _s,      // s
              _s_d,    // s_d
              _s_dd,   // s_dd
              _s_ddd,  // s_ddd
              _cd,     // cd
              _cv,     // cv
              _cf,     // cf
              _x,      // x;
              _y,      // y
              _yaw,    // yaw
              _ds,     // ds
              _c       // c
          };
          frenet_paths.push_back(_frenet_path);
        }
      }
    }
  }
};

#endif /* _TRAJECTORYGENERATOR_H_*/