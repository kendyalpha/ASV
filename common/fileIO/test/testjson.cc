/*
***********************************************************************
* testjson.cc:
* Utility test for the JSON
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <boost/test/included/unit_test.hpp>
#include "../include/jsonparse.h"

BOOST_AUTO_TEST_CASE(vessel) {
  const int m = 6;
  const int n = 3;
  const std::string jsonpath = "../data/test.json";
  ASV::common::jsonparse<m, n> _jsonparse(jsonpath);

  /*****************************  vessel  *****************************/
  ASV::common::vessel vessel_correct{
      (Eigen::Matrix<double, 3, 3>() << 540, 0, 0, 0, 540, 200, 0, 200, 1000)
          .finished(),  // Mass
      (Eigen::Matrix<double, 3, 3>() << 83, 0, 0, 0, 166, 244, 0, 244, 298)
          .finished(),  // AddedMass
      (Eigen::Matrix<double, 3, 3>() << 17, 0, 0, 0, 20, 0, 0, 0, 100)
          .finished(),  // LinearDamping
      (Eigen::Matrix<double, 3, 3>() << 20, 0, 0, 0, 0, 0, 0, 0, 0)
          .finished(),                              // QuadraticDamping
      (Eigen::Vector3d() << 1.6, 0, 0).finished(),  // cog
      (Eigen::Vector2d() << -5, 6).finished(),      // x_thrust
      (Eigen::Vector2d() << -1, 1).finished(),      // y_thrust
      (Eigen::Vector2d() << -4, 5).finished(),      // mz_thrust
      (Eigen::Vector2d() << -4, 3).finished(),      // surge_v
      (Eigen::Vector2d() << -0.5, 0.5).finished(),  // sway_v
      (Eigen::Vector2d() << -1, 2).finished(),      // yaw_v
      (Eigen::Vector2d() << -1.5, 1).finished(),    // roll_v
      3,                                            // L
      2                                             // B
  };

  // read a JSON file
  auto vessel = _jsonparse.getvessel();

  // TEST
  for (int i = 0; i != 3; ++i) {
    BOOST_CHECK_CLOSE(vessel_correct.cog(i), vessel.cog(i), 1e-7);
    for (int j = 0; j != 3; ++j) {
      BOOST_CHECK_CLOSE(vessel_correct.Mass(i, j), vessel.Mass(i, j), 1e-7);
      BOOST_CHECK_CLOSE(vessel_correct.AddedMass(i, j), vessel.AddedMass(i, j),
                        1e-7);
      BOOST_CHECK_CLOSE(vessel_correct.LinearDamping(i, j),
                        vessel.LinearDamping(i, j), 1e-7);
      BOOST_CHECK_CLOSE(vessel_correct.QuadraticDamping(i, j),
                        vessel.QuadraticDamping(i, j), 1e-7);
    }
  }

  for (int i = 0; i != 2; ++i) {
    BOOST_CHECK_CLOSE(vessel_correct.x_thrust(i), vessel.x_thrust(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.y_thrust(i), vessel.y_thrust(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.mz_thrust(i), vessel.mz_thrust(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.surge_v(i), vessel.surge_v(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.sway_v(i), vessel.sway_v(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.yaw_v(i), vessel.yaw_v(i), 1e-7);
    BOOST_CHECK_CLOSE(vessel_correct.roll_v(i), vessel.roll_v(i), 1e-7);
  }

  BOOST_CHECK_CLOSE(vessel_correct.L, vessel.L, 1e-7);
  BOOST_CHECK_CLOSE(vessel_correct.B, vessel.B, 1e-7);

  /*****************************  control  *****************************/
  ASV::control::controllerdata controllerdata_correct{
      0.1,                                    // sample_time
      3,                                      // los_radius
      2.1,                                    // los_capture_radius
      ASV::control::CONTROLMODE::MANUAL,      // controlmode
      ASV::control::ACTUATION::FULLYACTUATED  // index_actuation
  };

  ASV::control::thrustallocationdata thrustallocationdata_correct{
      40,                 // Q_surge
      400,                // Q_sway
      4000,               // Q_yaw
      1,                  // num_tunnel
      3,                  // num_azimuth
      1,                  // num_mainrudder
      1,                  // num_twinfixed
      {1, 2, 2, 2, 3, 4}  // index_thrusters
  };
  std::vector<ASV::control::tunnelthrusterdata> tunnelthrusterdata_correct = {{
      3.6 - vessel_correct.cog(0),  // lx
      0 - vessel_correct.cog(1),    // ly
      3e-4,                         // K_positive
      1e-4,                         // K_negative
      10,                           // max_delta_rotation
      1500,                         // max_rotation
      0,                            // max_thrust_positive
      0                             // max_thrust_negative
  }};
  std::vector<ASV::control::azimuththrusterdata> azimuththrusterdata_correct = {
      {
          2.23 - vessel_correct.cog(0),   // lx
          -0.83 - vessel_correct.cog(1),  // ly
          2.8E-6,                         // K
          10,                             // max_delta_rotation
          1500,                           // max_rotation
          5,                              // min_rotation
          0.01221730,                     // max_delta_alpha
          3.3161256,                      // max_alpha
          -0.1745329,                     // min_alpha
          0,                              // max_thrust
          0                               // min_thrust
      },
      {
          2.23 - vessel_correct.cog(0),  // lx
          0.83 - vessel_correct.cog(1),  // ly
          2.8E-6,                        // K
          10,                            // max_delta_rotation
          1500,                          // max_rotation
          5,                             // min_rotation
          0.01221730,                    // max_delta_alpha
          0.1745329,                     // max_alpha
          -3.3161256,                    // min_alpha
          0,                             // max_thrust
          0                              // min_thrust
      },
      {
          0.96 - vessel_correct.cog(0),   // lx
          -0.83 - vessel_correct.cog(1),  // ly
          2.8E-5,                         // K
          10,                             // max_delta_rotation
          1200,                           // max_rotation
          5,                              // min_rotation
          0.0349066,                      // max_delta_alpha
          3.3161256,                      // max_alpha
          -0.1745329,                     // min_alpha
          0,                              // max_thrust
          0                               // min_thrust
      }};

  std::vector<ASV::control::ruddermaindata> ruddermaindata_correct = {{
      -0.12 - vessel_correct.cog(0),  // lx
      0 - vessel_correct.cog(1),      // ly
      2.8e-6,                         // K
      11,                             // Cy
      20,                             // max_delta_rotation (no less than 1)
      200,                            // max rotation
      1,                              // min_rotation
      50,                             // max_thrust (effective thrust)
      2e-4,                           // min_thrust
      M_PI,                           // max_alpha
      -M_PI,                          // min_alpha
      1,                              // max_delta_varphi (no less than 1)
      30,                             // max_varphi
      -30                             // min_varphi
  }};
  std::vector<ASV::control::twinfixedthrusterdata> twinfixeddata_correct = {{
      -1.9 - vessel_correct.cog(0),  // lx
      -0.7 - vessel_correct.cog(1),  // ly
      0.0083,                        // K_positive
      0.0036,                        // K_negative
      50,                            // max_delta_rotation
      7,                             // max_delta_rotation_p2n
      220,                           // max rotation
      20,                            // max_thrust_positive
      0.002                          // max_thrust_negative
  }};
  std::vector<ASV::control::pidcontrollerdata> pidcontrollerdata_correct = {
      {
          10,                          // position_P;
          2,                           // position_I;
          1,                           // velocity_P;
          1,                           // velocity_I;
          1,                           // position_allowed_error;
          1,                           // velocity_allowed_error;
          vessel_correct.x_thrust(0),  // min_output;
          vessel_correct.x_thrust(1)   // max_output;
      },
      {
          10,                          // position_P;
          2,                           // position_I;
          1,                           // velocity_P;
          1,                           // velocity_I;
          0.1,                         // position_allowed_error;
          1,                           // velocity_allowed_error;
          vessel_correct.y_thrust(0),  // min_output;
          vessel_correct.y_thrust(1)   // max_output;
      },
      {
          10,                           // position_P;
          2,                            // position_I;
          1,                            // velocity_P;
          1,                            // velocity_I;
          0.1,                          // position_allowed_error;
          1,                            // velocity_allowed_error;
          vessel_correct.mz_thrust(0),  // min_output;
          vessel_correct.mz_thrust(1)   // max_output;
      }};

  // read a JSON file
  auto controldata = _jsonparse.getcontrollerdata();
  auto TAdata = _jsonparse.getthrustallocationdata();
  auto tunneldata = _jsonparse.gettunneldata();
  auto azimuthdata = _jsonparse.getazimuthdata();
  auto rudderdata = _jsonparse.getmainrudderdata();
  auto twinfixeddata = _jsonparse.gettwinfixeddata();
  auto piddata = _jsonparse.getpiddata();

  // TEST
  {
    BOOST_CHECK_CLOSE(controldata.sample_time,
                      controllerdata_correct.sample_time, 1e-7);
    BOOST_CHECK_CLOSE(controldata.los_radius, controllerdata_correct.los_radius,
                      1e-7);
    BOOST_CHECK_CLOSE(controldata.los_capture_radius,
                      controllerdata_correct.los_capture_radius, 1e-7);
  }

  for (int i = 0; i != 3; ++i) {
    BOOST_CHECK_CLOSE(piddata[i].position_P,
                      pidcontrollerdata_correct[i].position_P, 1e-7);
    BOOST_CHECK_CLOSE(piddata[i].position_I,
                      pidcontrollerdata_correct[i].position_I, 1e-7);
    BOOST_CHECK_CLOSE(piddata[i].velocity_P,
                      pidcontrollerdata_correct[i].velocity_P, 1e-7);
    BOOST_CHECK_CLOSE(piddata[i].velocity_I,
                      pidcontrollerdata_correct[i].velocity_I, 1e-7);
    BOOST_CHECK_CLOSE(piddata[i].position_allowed_error,
                      pidcontrollerdata_correct[i].position_allowed_error,
                      1e-7);
    BOOST_CHECK_CLOSE(piddata[i].min_output,
                      pidcontrollerdata_correct[i].min_output, 1e-7);
    BOOST_CHECK_CLOSE(piddata[i].max_output,
                      pidcontrollerdata_correct[i].max_output, 1e-7);
  }

  {
    BOOST_CHECK_CLOSE(TAdata.Q_surge, thrustallocationdata_correct.Q_surge,
                      1e-7);
    BOOST_CHECK_CLOSE(TAdata.Q_sway, thrustallocationdata_correct.Q_sway, 1e-7);
    BOOST_CHECK_CLOSE(TAdata.Q_yaw, thrustallocationdata_correct.Q_yaw, 1e-7);
    BOOST_TEST(TAdata.num_tunnel == thrustallocationdata_correct.num_tunnel);
    BOOST_TEST(TAdata.num_azimuth == thrustallocationdata_correct.num_azimuth);
    BOOST_TEST(TAdata.num_mainrudder ==
               thrustallocationdata_correct.num_mainrudder);
    BOOST_TEST(TAdata.num_twinfixed ==
               thrustallocationdata_correct.num_twinfixed);
    for (int i = 0; i != m; ++i)
      BOOST_TEST(TAdata.index_thrusters[i] ==
                 thrustallocationdata_correct.index_thrusters[i]);
  }

  for (int i = 0; i != thrustallocationdata_correct.num_tunnel; ++i) {
    BOOST_CHECK_CLOSE(tunneldata[i].lx, tunnelthrusterdata_correct[i].lx, 1e-7);
    BOOST_CHECK_CLOSE(tunneldata[i].ly, tunnelthrusterdata_correct[i].ly, 1e-7);
    BOOST_CHECK_CLOSE(tunneldata[i].K_positive,
                      tunnelthrusterdata_correct[i].K_positive, 1e-7);
    BOOST_CHECK_CLOSE(tunneldata[i].K_negative,
                      tunnelthrusterdata_correct[i].K_negative, 1e-7);
    BOOST_TEST(tunneldata[i].max_delta_rotation ==
               tunnelthrusterdata_correct[i].max_delta_rotation);
    BOOST_TEST(tunneldata[i].max_rotation ==
               tunnelthrusterdata_correct[i].max_rotation);
  }
  for (int i = 0; i != thrustallocationdata_correct.num_azimuth; ++i) {
    BOOST_CHECK_CLOSE(azimuthdata[i].lx, azimuththrusterdata_correct[i].lx,
                      1e-7);
    BOOST_CHECK_CLOSE(azimuthdata[i].ly, azimuththrusterdata_correct[i].ly,
                      1e-7);
    BOOST_CHECK_CLOSE(azimuthdata[i].K, azimuththrusterdata_correct[i].K, 1e-7);
    BOOST_TEST(azimuthdata[i].max_delta_rotation ==
               azimuththrusterdata_correct[i].max_delta_rotation);
    BOOST_TEST(azimuthdata[i].max_rotation ==
               azimuththrusterdata_correct[i].max_rotation);
    BOOST_TEST(azimuthdata[i].min_rotation ==
               azimuththrusterdata_correct[i].min_rotation);
    BOOST_CHECK_CLOSE(azimuthdata[i].max_delta_alpha,
                      azimuththrusterdata_correct[i].max_delta_alpha, 1e-4);
    BOOST_CHECK_CLOSE(azimuthdata[i].max_alpha,
                      azimuththrusterdata_correct[i].max_alpha, 1e-4);
    BOOST_CHECK_CLOSE(azimuthdata[i].min_alpha,
                      azimuththrusterdata_correct[i].min_alpha, 1e-4);
  }
  for (int i = 0; i != thrustallocationdata_correct.num_mainrudder; ++i) {
    BOOST_CHECK_CLOSE(rudderdata[i].lx, ruddermaindata_correct[i].lx, 1e-7);
    BOOST_CHECK_CLOSE(rudderdata[i].ly, ruddermaindata_correct[i].ly, 1e-7);
    BOOST_CHECK_CLOSE(rudderdata[i].K, ruddermaindata_correct[i].K, 1e-7);
    BOOST_CHECK_CLOSE(rudderdata[i].Cy, ruddermaindata_correct[i].Cy, 1e-7);
    BOOST_TEST(rudderdata[i].max_delta_rotation ==
               ruddermaindata_correct[i].max_delta_rotation);
    BOOST_TEST(rudderdata[i].max_rotation ==
               ruddermaindata_correct[i].max_rotation);
    BOOST_TEST(rudderdata[i].min_rotation ==
               ruddermaindata_correct[i].min_rotation);
    BOOST_CHECK_CLOSE(rudderdata[i].max_delta_varphi,
                      ruddermaindata_correct[i].max_delta_varphi, 1e-4);
    BOOST_CHECK_CLOSE(rudderdata[i].max_varphi,
                      ruddermaindata_correct[i].max_varphi, 1e-4);
    BOOST_CHECK_CLOSE(rudderdata[i].min_varphi,
                      ruddermaindata_correct[i].min_varphi, 1e-4);
  }
  for (int i = 0; i != thrustallocationdata_correct.num_twinfixed; ++i) {
    BOOST_CHECK_CLOSE(twinfixeddata[i].lx, twinfixeddata_correct[i].lx, 1e-7);
    BOOST_CHECK_CLOSE(twinfixeddata[i].ly, twinfixeddata_correct[i].ly, 1e-7);
    BOOST_CHECK_CLOSE(twinfixeddata[i].K_positive,
                      twinfixeddata_correct[i].K_positive, 1e-7);
    BOOST_CHECK_CLOSE(twinfixeddata[i].K_negative,
                      twinfixeddata_correct[i].K_negative, 1e-7);
    BOOST_TEST(twinfixeddata[i].max_delta_rotation ==
               twinfixeddata_correct[i].max_delta_rotation);
    BOOST_TEST(twinfixeddata[i].max_delta_rotation_p2n ==
               twinfixeddata_correct[i].max_delta_rotation_p2n);
    BOOST_TEST(twinfixeddata[i].max_rotation ==
               twinfixeddata_correct[i].max_rotation);
  }

  /*****************************  estimator  *****************************/
  // ASV::control::estimatordata controllerdata_correct{
  //     0.1,                                    // sample_time
  //     3,                                      // los_radius
  //     2.1,                                    // los_capture_radius
  //     ASV::control::CONTROLMODE::MANUAL,      // controlmode
  //     ASV::control::ACTUATION::FULLYACTUATED  // index_actuation
  // };

  // std::cout << _jsonparse << std::endl;
}
