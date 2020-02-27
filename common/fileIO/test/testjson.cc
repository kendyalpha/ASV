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
  ASV::localization::estimatordata estimatordata_correct{
      0.2,  // sample_time
      vessel_correct.cog -
          (Eigen::Vector3d() << 0.1, 0, 0).finished(),  // antenna2cog
      (Eigen::Matrix<double, 6, 6>() << 5e-1, 0, 0, 0, 0, 0, 0, 5e-2, 0, 0, 0,
       0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0,
       0, 0, 0, 1e-7)
          .finished(),  // Q
      (Eigen::Matrix<double, 6, 6>() << 2.8e-2, 0, 0, 0, 0, 0, 0, 3.5e-3, 0, 0,
       0, 0, 0, 0, 1e-4, 0, 0, 0, 0, 0, 0, 9e-4, 0, 0, 0, 0, 0, 0, 4e-4, 0, 0,
       0, 0, 0, 0, 4.8e-8)
          .finished()  // R
  };

  auto estimatordata = _jsonparse.getestimatordata();

  {
    BOOST_CHECK_CLOSE(estimatordata.sample_time,
                      estimatordata_correct.sample_time, 1e-7);

    for (int i = 0; i != 3; ++i)
      BOOST_CHECK_CLOSE(estimatordata.antenna2cog(i),
                        estimatordata_correct.antenna2cog(i), 1e-7);
    for (int i = 0; i != 6; ++i) {
      for (int j = 0; j != 6; ++j) {
        BOOST_CHECK_CLOSE(estimatordata.Q(i, j), estimatordata_correct.Q(i, j),
                          1e-9);
        BOOST_CHECK_CLOSE(estimatordata.R(i, j), estimatordata_correct.R(i, j),
                          1e-9);
      }
    }
  }

  /*****************************  planner  *****************************/
  ASV::planning::LatticeData latticedata_correct{
      0.1,                        // SAMPLE_TIME
      vessel_correct.surge_v(1),  // MAX_SPEED
      0.05,                       // TARGET_COURSE_ARC_STEP
      7.0,                        // MAX_ROAD_WIDTH
      1,                          // ROAD_WIDTH_STEP
      5.0,                        // MAXT
      4.0,                        // MINT
      0.2,                        // DT
      1.2,                        // MAX_SPEED_DEVIATION
      0.3                         // TRAGET_SPEED_STEP
  };

  ASV::planning::CollisionData collisiondata_correct{
      vessel_correct.surge_v(1),  // MAX_SPEED
      1.5 * vessel_correct.x_thrust(1) /
          vessel_correct.Mass(0, 0),  // MAX_ACCEL
      1.5 * vessel_correct.x_thrust(0) /
          vessel_correct.Mass(0, 0),  // MIN_ACCEL
      1.5 * vessel_correct.mz_thrust(1) /
          vessel_correct.Mass(2, 2),  // MAX_ANG_ACCEL
      1.5 * vessel_correct.mz_thrust(0) /
          vessel_correct.Mass(2, 2),  // MIN_ANG_ACCEL
      1.0,                            // MAX_CURVATURE
      vessel_correct.L,               // HULL_LENGTH
      vessel_correct.B,               // HULL_WIDTH
      2.5                             // ROBOT_RADIUS
  };

  auto planner_lattice_data = _jsonparse.getlatticedata();
  auto collision_data = _jsonparse.getcollisiondata();

  {
    BOOST_CHECK_CLOSE(planner_lattice_data.SAMPLE_TIME,
                      latticedata_correct.SAMPLE_TIME, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.MAX_SPEED,
                      latticedata_correct.MAX_SPEED, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.TARGET_COURSE_ARC_STEP,
                      latticedata_correct.TARGET_COURSE_ARC_STEP, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.MAX_ROAD_WIDTH,
                      latticedata_correct.MAX_ROAD_WIDTH, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.MAXT, latticedata_correct.MAXT,
                      1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.MINT, latticedata_correct.MINT,
                      1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.DT, latticedata_correct.DT, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.MAX_SPEED_DEVIATION,
                      latticedata_correct.MAX_SPEED_DEVIATION, 1e-7);
    BOOST_CHECK_CLOSE(planner_lattice_data.TRAGET_SPEED_STEP,
                      latticedata_correct.TRAGET_SPEED_STEP, 1e-7);
  }
  {
    BOOST_CHECK_CLOSE(collision_data.MAX_SPEED, collisiondata_correct.MAX_SPEED,
                      1e-7);
    BOOST_CHECK_CLOSE(collision_data.MAX_ACCEL, collisiondata_correct.MAX_ACCEL,
                      1e-7);
    BOOST_CHECK_CLOSE(collision_data.MIN_ACCEL, collisiondata_correct.MIN_ACCEL,
                      1e-7);
    BOOST_CHECK_CLOSE(collision_data.MAX_ANG_ACCEL,
                      collisiondata_correct.MAX_ANG_ACCEL, 1e-7);
    BOOST_CHECK_CLOSE(collision_data.MIN_ANG_ACCEL,
                      collisiondata_correct.MIN_ANG_ACCEL, 1e-7);
    BOOST_CHECK_CLOSE(collision_data.MAX_CURVATURE,
                      collisiondata_correct.MAX_CURVATURE, 1e-7);
    BOOST_CHECK_CLOSE(collision_data.HULL_LENGTH,
                      collisiondata_correct.HULL_LENGTH, 1e-7);
    BOOST_CHECK_CLOSE(collision_data.HULL_WIDTH,
                      collisiondata_correct.HULL_WIDTH, 1e-7);
    BOOST_CHECK_CLOSE(collision_data.ROBOT_RADIUS,
                      collisiondata_correct.ROBOT_RADIUS, 1e-7);
  }

  /*****************************  perception  *****************************/

  ASV::perception::ClusteringData Clustering_correct{
      1,  // p_radius
      3   // p_minumum_neighbors
  };

  ASV::perception::SpokeProcessdata SpokeProcess_correct{
      0.1,      // sampletime
      -1.0,     // radar_x
      1.222223  // radar_y
  };

  ASV::perception::AlarmZone Alarm_Zone_correct{
      10,        // start_range_m
      20,        // end_range_m
      0,         // center_bearing_rad
      M_PI / 2,  // width_bearing_rad
      0xc8       // sensitivity_threhold
  };

  ASV::perception::TrackingTargetData TrackingTarget_correct{
      1,    // min_squared_radius
      9,    // max_squared_radius
      1,    // speed_threhold
      20,   // max_speed
      5,    // max_acceleration
      600,  // max_roti
      1,    // safe_distance
      0.8,  // K_radius
      1,    // K_delta_speed
      1     // K_delta_yaw;
  };

  {
    auto Clustering_data = _jsonparse.getClusteringdata();

    BOOST_CHECK_CLOSE(Clustering_data.p_radius, Clustering_correct.p_radius,
                      1e-7);
    BOOST_TEST(Clustering_data.p_minumum_neighbors ==
               Clustering_correct.p_minumum_neighbors);
  }

  {
    auto spoke_data = _jsonparse.getSpokeProcessdata();

    BOOST_CHECK_CLOSE(spoke_data.sample_time, SpokeProcess_correct.sample_time,
                      1e-7);
    BOOST_CHECK_CLOSE(spoke_data.radar_x, SpokeProcess_correct.radar_x, 1e-7);
    BOOST_CHECK_CLOSE(spoke_data.radar_y, SpokeProcess_correct.radar_y, 1e-7);
  }

  {
    auto Alarm_data = _jsonparse.getalarmzonedata();

    BOOST_CHECK_CLOSE(Alarm_data.start_range_m,
                      Alarm_Zone_correct.start_range_m, 1e-7);
    BOOST_CHECK_CLOSE(Alarm_data.end_range_m, Alarm_Zone_correct.end_range_m,
                      1e-7);
    BOOST_CHECK_CLOSE(Alarm_data.center_bearing_rad,
                      Alarm_Zone_correct.center_bearing_rad, 1e-7);
    BOOST_TEST(Alarm_data.sensitivity_threhold ==
               Alarm_Zone_correct.sensitivity_threhold);
  }
  {
    auto trackingtarget_data = _jsonparse.getTargetTrackingdata();
    BOOST_CHECK_CLOSE(trackingtarget_data.min_squared_radius,
                      TrackingTarget_correct.min_squared_radius, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.max_squared_radius,
                      TrackingTarget_correct.max_squared_radius, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.speed_threhold,
                      TrackingTarget_correct.speed_threhold, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.max_speed,
                      TrackingTarget_correct.max_speed, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.max_acceleration,
                      TrackingTarget_correct.max_acceleration, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.max_roti,
                      TrackingTarget_correct.max_roti, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.safe_distance,
                      TrackingTarget_correct.safe_distance, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.K_radius,
                      TrackingTarget_correct.K_radius, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.K_delta_speed,
                      TrackingTarget_correct.K_delta_speed, 1e-7);
    BOOST_CHECK_CLOSE(trackingtarget_data.K_delta_yaw,
                      TrackingTarget_correct.K_delta_yaw, 1e-7);
  }

  unsigned long gps_baudrate_correct = 115200;
  std::string gps_port_correct = "/dev/ttyUSB0";
  unsigned long gui_baudrate_correct = 19200;
  std::string gui_port_correct = "/dev/ttyUSB1";
  unsigned long rc_baudrate_correct = 115200;
  std::string rc_port_correct = "/dev/ttyUSB2";
  unsigned long wind_baudrate_correct = 9600;
  std::string wind_port_correct = "/dev/ttyUSB3";
  unsigned long stm32_baudrate_correct = 115200;
  std::string stm32_port_correct = "/dev/ttyUSB4";

  {
    BOOST_TEST(_jsonparse.getgpsbaudrate() == gps_baudrate_correct);
    BOOST_TEST(_jsonparse.getguibaudrate() == gui_baudrate_correct);
    BOOST_TEST(_jsonparse.getrcbaudrate() == rc_baudrate_correct);
    BOOST_TEST(_jsonparse.getwindbaudrate() == wind_baudrate_correct);
    BOOST_TEST(_jsonparse.getstm32baudrate() == stm32_baudrate_correct);

    BOOST_TEST(_jsonparse.getgpsport() == gps_port_correct);
    BOOST_TEST(_jsonparse.getguiport() == gui_port_correct);
    BOOST_TEST(_jsonparse.getremotecontrolport() == rc_port_correct);
    BOOST_TEST(_jsonparse.getwindport() == wind_port_correct);
    BOOST_TEST(_jsonparse.getstm32port() == stm32_port_correct);
  }

  std::cout << "db_config: " << _jsonparse.getdbconfigpath() << std::endl;
  std::cout << "sqlite_path: " << _jsonparse.getsqlitepath() << std::endl;
}
