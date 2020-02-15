/*
*******************************************************************************
* testthrust.cc:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/thrustallocation.h"
#include "common/fileIO/include/utilityio.h"
#include "common/plotting/include/gnuplot-iostream.h"

using namespace ASV::control;
using namespace ASV::common;

// illustrate the results using gnuplot
void plotTAresults(const Eigen::MatrixXd &plot_u,
                   const Eigen::MatrixXi &plot_rotation,
                   const Eigen::MatrixXd &plot_alpha,
                   const Eigen::MatrixXi &plot_alpha_deg,
                   const Eigen::MatrixXd &plot_Balphau,
                   const Eigen::MatrixXd &plot_tau) {
  Gnuplot gp;
  std::vector<std::pair<double, double> > xy_pts_A;
  std::vector<std::pair<double, double> > xy_pts_B;

  int m = plot_u.rows();  // # of thrusters
  int totalstep = plot_u.cols();

  // the first window: estimated and desired thrust
  gp << "set terminal x11 size 1000, 1200 0\n";
  gp << "set multiplot layout 3, 1 title 'Comparision of estimated and desired "
        "force' font ',14'\n";
  std::vector<std::string> label_names = {"F_x(N)", "F_y(N)", "M_z(N*m)"};

  for (int i = 0; i != 3; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel '" << label_names[i] << "'\n";
    gp << "plot"
          " '-' with lines lt 1 lw 2 lc rgb 'violet' title 'estimated',"
          " '-' with lines lt 2 lw 2 lc rgb 'black' title 'desired'\n";
    xy_pts_A.clear();
    xy_pts_B.clear();
    for (int j = 0; j != totalstep; ++j) {
      xy_pts_A.push_back(std::make_pair(j, plot_Balphau(i, j)));
      xy_pts_B.push_back(std::make_pair(j, plot_tau(i, j)));
    }
    gp.send1d(xy_pts_A);
    gp.send1d(xy_pts_B);
  }
  gp << "unset multiplot\n";

  // the second window: rotation
  gp << "set term x11 1\n";
  gp << "set multiplot layout " << m << ", 1 title 'rotation' font ',14'\n";

  for (int i = 0; i != m; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel 'rpm" << i << "'\n";
    gp << "plot"
          " '-' with lines lt 1 lw 2 lc rgb 'black'\n";

    xy_pts_A.clear();
    for (int j = 0; j != totalstep; ++j) {
      xy_pts_A.push_back(std::make_pair(j, plot_rotation(i, j)));
    }
    gp.send1d(xy_pts_A);
  }

  gp << "unset multiplot\n";

  // the third window: u
  gp << "set term x11 2\n";
  gp << "set multiplot layout " << m << ", 1 title 'u' font ',14'\n";

  for (int i = 0; i != m; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel 'u" << i << "(N)'\n";
    gp << "plot"
          " '-' with lines lt 1 lw 2 lc rgb 'black'\n";

    xy_pts_A.clear();
    for (int j = 0; j != totalstep; ++j) {
      xy_pts_A.push_back(std::make_pair(j, plot_u(i, j)));
    }
    gp.send1d(xy_pts_A);
  }

  gp << "unset multiplot\n";

  // the third window: alpha
  gp << "set term x11 3\n";
  gp << "set multiplot layout " << m << ", 1 title 'alpha' font ',14'\n";

  for (int i = 0; i != m; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel 'alpha" << i << "(deg)'\n";
    gp << "plot"
          " '-' with lines lt 1 lw 2 lc rgb 'violet' title 'alpha',"
          " '-' with lines lt 2 lw 2 lc rgb 'black' title 'alpha-deg'\n";
    xy_pts_A.clear();
    xy_pts_B.clear();
    for (int j = 0; j != totalstep; ++j) {
      xy_pts_A.push_back(std::make_pair(j, (180 / M_PI) * plot_alpha(i, j)));
      xy_pts_B.push_back(std::make_pair(j, plot_alpha_deg(i, j)));
    }
    gp.send1d(xy_pts_A);
    gp.send1d(xy_pts_B);
  }
  gp << "unset multiplot\n";
}  // plotTAresults

// test thrust allocation for 4 propellers (fully actuated)
void testonestepthrustallocation() {
  const int m = 4;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::FULLYACTUATED;

  std::vector<int> index_thrusters{1, 1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      500,             // Q_surge
      500,             // Q_sway
      1000,            // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back({
      1.9,   // lx
      0,     // ly
      1e-6,  // K_positive
      2e-6,  // K_negative
      50,    // max_delta_rotation
      1000,  // max_rotation
      1,     // max_thrust_positive
      2      // max_thrust_negative
  });
  v_tunnelthrusterdata.push_back({
      1,     // lx
      0,     // ly
      1e-6,  // K_positive
      2e-6,  // K_negative
      50,    // max_delta_rotation
      1000,  // max_rotation
      1,     // max_thrust_positive
      2      // max_thrust_negative
  });

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,         // lx
      -0.216,         // ly
      2e-5,           // K
      10,             // max_delta_rotation
      1000,           // max rotation
      10,             // min_rotation
      0.1277,         // max_delta_alpha
      M_PI / 6,       // max_alpha
      -7 * M_PI / 6,  // min_alpha
      20,             // max_thrust
      2e-3            // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,        // lx
      0.216,         // ly
      2e-5,          // K
      10,            // max_delta_rotation
      1000,          // max rotation
      10,            // min_rotation
      0.1277,        // max_delta_alpha
      7 * M_PI / 6,  // max_alpha
      -M_PI / 6,     // min_alpha
      20,            // max_thrust
      2e-3           // min_thrust
  });
  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                                      // state_toggle
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),                    // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  // command_u
      // vectormi()::Zero(),
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300)
          .finished(),  // command_rotation
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
          .finished(),                   // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),  // command_alpha_deg
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  // feedback_u
      // vectormi()::Zero(),                    // feedback_rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
          .finished(),                  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()  // feedback_alpha_deg

  };

  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);

  std::cout << _controllerRTdata.command_alpha << std::endl;
  std::cout << "upper_delta_alpha: " << _thrustallocation.getupper_delta_alpha()
            << std::endl;
  std::cout << "lower_delta_alpha: " << _thrustallocation.getlower_delta_alpha()
            << std::endl;
  std::cout << "upper_delta_u: " << _thrustallocation.getupper_delta_u()
            << std::endl;
  std::cout << "lower_delta_u: " << _thrustallocation.getlower_delta_u()
            << std::endl;
  std::cout << "Q: " << _thrustallocation.getQ() << std::endl;
  std::cout << "Omega: " << _thrustallocation.getOmega() << std::endl;
  std::cout << "Q_deltau: " << _thrustallocation.getQ_deltau() << std::endl;
  std::cout << "g_deltau: " << _thrustallocation.getg_deltau() << std::endl;
  std::cout << "d_rho: " << _thrustallocation.getd_rho() << std::endl;
  std::cout << "B_alpha: " << _thrustallocation.getB_alpha() << std::endl;
  std::cout << "d_Balpha_u: " << _thrustallocation.getd_Balpha_u() << std::endl;
  std::cout << "lx: " << _thrustallocation.getlx() << std::endl;

  Eigen::MatrixXd vvv(2, 3);
  vvv.setZero();
  write2csvfile("csvfile.csv", vvv);
}

// test thrust allocation for 3 propellers (fully actuated)
void test_multiplethrusterallocation() {
  // set the parameters in the thrust allocation
  const int m = 3;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::FULLYACTUATED;

  std::vector<int> index_thrusters{1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      500,             // Q_surge
      500,             // Q_sway
      500,             // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back(
      {1.9, 0, 3.7e-7, 1.7e-7, 50, 3000, 3.33, 1.53});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,            // lx
      -0.216,            // ly
      2e-5,              // K
      20,                // max_delta_rotation
      1000,              // max rotation
      10,                // min_rotation
      0.1277,            // max_delta_alpha
      M_PI * 175 / 180,  // max_alpha
      M_PI / 18,         // min_alpha
      20,                // max_thrust
      0.002              // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,             // lx
      0.216,              // ly
      2e-5,               // K
      20,                 // max_delta_rotation
      1000,               // max rotation
      10,                 // min_rotation
      0.1277,             // max_delta_alpha
      -M_PI / 18,         // max_alpha
      -M_PI * 175 / 180,  // min_alpha
      20,                 // max_thrust
      0.002               // min_thrust
  });

  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::DYNAMICPOSITION);

  // data saved for validation and viewer
  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(m, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = -1.0 * sin(angle) + 0.2 * std::rand() / RAND_MAX;
  }
  save_tau.block(1, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 0) +
                                 0.00 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(1, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, -0) +
                                   0.00 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(0) = 2 * Eigen::MatrixXd::Constant(1, totalstep, 1) +
                    0.01 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // update feedback
    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;

    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // save variables
    save_u.col(i) = _controllerRTdata.command_u;
    save_alpha.col(i) = _controllerRTdata.command_alpha;
    save_alpha_deg.col(i) = _controllerRTdata.command_alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.command_rotation;
  }

  plotTAresults(save_u, save_rotation, save_alpha, save_alpha_deg, save_Balphau,
                save_tau);
}

// test thrust allocation for twin-fixed propeller (underactuated)
void test_twinfixed() {
  // set the parameters in the thrust allocation
  const int m = 2;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::UNDERACTUATED;

  std::vector<int> index_thrusters{4, 4};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      10,              // Q_surge
      10,              // Q_sway
      20,              // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;
  v_twinfixeddata.reserve(num_twinfixed);
  v_twinfixeddata.push_back({
      -1.9,    // lx
      -0.7,    // ly
      0.0083,  // K_positive
      0.0036,  // K_negative
      50,      // max_delta_rotation
      10,      // max_delta_rotation_p2n
      220,     // max rotation
      20,      // max_thrust_positive
      0.002    // max_thrust_negative
  });
  v_twinfixeddata.push_back({
      -1.9,    // lx
      0.7,     // ly
      0.0083,  // K_positive
      0.0036,  // K_negative
      50,      // max_delta_rotation
      10,      // max_delta_rotation_p2n
      220,     // max rotation
      20,      // max_thrust_positive
      0.002    // max_thrust_negative
  });

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::MANEUVERING);

  // data saved for validation and viewer
  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(m, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 180; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 20 * sin(angle) + 0.6 * std::rand() / RAND_MAX;
  }
  save_tau.block(0, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 50) +
                                 0.1 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(0, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, -40) +
                                   0.1 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(1) = 0.00 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // update feedback
    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;

    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // save variables
    save_u.col(i) = _controllerRTdata.command_u;
    save_alpha.col(i) = _controllerRTdata.command_alpha;
    save_alpha_deg.col(i) = _controllerRTdata.command_alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.command_rotation;
  }

  plotTAresults(save_u, save_rotation, save_alpha, save_alpha_deg, save_Balphau,
                save_tau);
}

void testrudder() {
  // set the parameters in the thrust allocation
  constexpr int m = 1;
  constexpr int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::UNDERACTUATED;

  std::vector<int> index_thrusters{3};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      500,             // Q_surge
      500,             // Q_sway
      1000,            // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);

  std::vector<ruddermaindata> v_ruddermaindata;
  v_ruddermaindata.reserve(num_mainrudder);
  v_ruddermaindata.push_back({
      -1.893,  // lx
      0,       // ly
      2e-5,    // K
      0.0126,  // Cy
      20,      // max_delta_rotation (no less than 1)
      500,     // max rotation
      1,       // min_rotation
      50,      // max_thrust (effective thrust)
      2e-4,    // min_thrust
      M_PI,    // max_alpha
      -M_PI,   // min_alpha
      5,       // max_delta_varphi (no less than 1)
      30,      // max_varphi
      -30      // min_varphi
  });
  // v_ruddermaindata.push_back({
  //     -1.893,  // lx
  //     0,       // ly
  //     2e-5,    // K
  //     0.0126,  // Cy
  //     20,      // max_delta_rotation
  //     500,     // max rotation
  //     1,       // min_rotation
  //     0,       // max_thrust
  //     2e-5,    // min_thrust
  //     M_PI,    // max_alpha
  //     -M_PI,   // min_alpha
  //     1,       // max_delta_varphi
  //     30,      // max_varphi
  //     -30      // min_varphi
  // });
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::MANEUVERING);

  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(m, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 0.5 * sin(angle) + 0.1 * std::rand() / RAND_MAX;
  }
  save_tau.block(0, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 0.3) +
                                 0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(0, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 0.3) +
                                   0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(1) = 0.01 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);

    // update feedback
    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;
    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);

    // save variables
    save_u.col(i) = _controllerRTdata.command_u;
    save_alpha.col(i) = _controllerRTdata.command_alpha;
    save_alpha_deg.col(i) = _controllerRTdata.command_alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.command_rotation;
  }

  plotTAresults(save_u, save_rotation, save_alpha, save_alpha_deg, save_Balphau,
                save_tau);
}

void testbiling() {
  // set the parameters in the thrust allocation
  const int m = 6;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::FULLYACTUATED;

  std::vector<int> index_thrusters{2, 2, 2, 2, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      500,             // Q_surge
      500,             // Q_sway
      1000,            // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      2,           // lx
      0,           // ly
      2.8E-6,      // K
      100,         // max_delta_rotation
      1500,        // max rotation
      5,           // min_rotation
      0.1277,      // max_delta_alpha
      1.5 * M_PI,  // max_alpha
      0.5 * M_PI,  // min_alpha
      20,          // max_thrust
      0.05         // min_thrust
  });
  v_azimuththrusterdata.push_back({
      0.63,    // lx
      -0.83,   // ly
      2.8e-6,  // K
      100,     // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      M_PI,    // max_alpha
      0,       // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      0.63,    // lx
      0.83,    // ly
      2.8e-6,  // K
      100,     // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      0,       // max_alpha
      -M_PI,   // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -0.64,   // lx
      -0.83,   // ly
      2.8E-6,  // K
      100,     // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      M_PI,    // max_alpha
      0,       // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -0.64,   // lx
      0.83,    // ly
      2.8E-6,  // K
      100,     // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      0,       // max_alpha
      -M_PI,   // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.72,        // lx
      0,            // ly
      2.8E-6,       // K
      100,          // max_delta_rotation
      1500,         // max rotation
      5,            // min_rotation
      0.1277,       // max_delta_alpha
      0.5 * M_PI,   // max_alpha
      -0.5 * M_PI,  // min_alpha
      20,           // max_thrust
      0.05          // min_thrust
  });
  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::DYNAMICPOSITION);

  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(m, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 0.0 * sin(angle) + 0.0 * std::rand() / RAND_MAX;
  }
  save_tau.block(0, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 20) +
                                 0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(0, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 20) +
                                   0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(1) = 0.00 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // update feedback
    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;
    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // save variables
    save_u.col(i) = _controllerRTdata.command_u;
    save_alpha.col(i) = _controllerRTdata.command_alpha;
    save_alpha_deg.col(i) = _controllerRTdata.command_alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.command_rotation;
  }

  plotTAresults(save_u, save_rotation, save_alpha, save_alpha_deg, save_Balphau,
                save_tau);
}

void testoutboard() {
  // set the parameters in the thrust allocation
  const int m = 1;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::UNDERACTUATED;

  std::vector<int> index_thrusters{2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      50,              // Q_surge
      50,              // Q_sway
      100,             // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -2,           // lx
      0,            // ly
      2.8E-4,       // K
      100,          // max_delta_rotation
      1500,         // max rotation
      5,            // min_rotation
      0.1277,       // max_delta_alpha
      0.5 * M_PI,   // max_alpha
      -0.5 * M_PI,  // min_alpha
      20,           // max_thrust
      0.05          // min_thrust
  });

  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                    // state_toggle
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // command_u
      Eigen::Matrix<int, m, 1>::Zero(),     // command_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),     // command_alpha_deg
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_u
      Eigen::Matrix<int, m, 1>::Zero(),     // feedback_rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()      // feedback_alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::MANEUVERING);

  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(m, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 100 * sin(angle) + 0.6 * std::rand() / RAND_MAX;
  }
  save_tau.block(0, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 80) +
                                 0.8 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(0, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 90) +
                                   0.9 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(1) = 0.00 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // update feedback
    _controllerRTdata.feedback_rotation = _controllerRTdata.command_rotation;
    _controllerRTdata.feedback_alpha_deg = _controllerRTdata.command_alpha_deg;
    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // save variables
    save_u.col(i) = _controllerRTdata.command_u;
    save_alpha.col(i) = _controllerRTdata.command_alpha;
    save_alpha_deg.col(i) = _controllerRTdata.command_alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.command_rotation;
  }

  plotTAresults(save_u, save_rotation, save_alpha, save_alpha_deg, save_Balphau,
                save_tau);
}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // test_multiplethrusterallocation();
  // testrudder();
  // test_twinfixed();
  testbiling();
  // testoutboard();

  LOG(INFO) << "Shutting down.";
  return 0;
}