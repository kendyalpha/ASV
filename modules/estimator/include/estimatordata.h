/*
***********************************************************************
* estimatordata.h:
* header file to define the constant and real-time data in the
* motion estimators, dependent on each autonomous system.
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _ESTIMATORDATA_H_
#define _ESTIMATORDATA_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

#include "common/property/include/priority.h"

namespace ASV::localization {

enum class USEKALMAN {
  KALMANOFF = 0,  // turn off kalman filtering
  KALMANON        // turn on kalman filtering
};

enum class WINDCOMPENSATION {
  WINDOFF = 0,  // turn off the wind compenstation
  WINDON        // turn on the wind compenstation
};

/********************* constant ***********************************/
struct estimatordata {
  double sample_time;  // sample time of estimator((unit: second))

  // location of CoG relative to primary anntena
  Eigen::Vector3d antenna2cog;

  // Kalman filter
  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 6, 6> R;
};

// real-time data in the state estimators
struct estimatorRTdata {
  /********************* state toggle  *********************/
  common::STATETOGGLE state_toggle;
  /********************* transformation matrix  *********************/
  Eigen::Matrix3d CTB2G;  // body  --> global
  Eigen::Matrix3d CTG2B;  // global  --> body

  /********************* measured data  *********************/
  /** Measurement: sensor data of CoG
   * +------+------+---------------+--------+--------+-------------+
   * |  0   |  1   |       2       |   3    |   4    |     5       |
   * | x(m) | y(m) | heading (rad) | u(m/s) | v(m/s) | roti(rad/s) |
   * +------+------+---------------+--------+--------+-------------+
   */
  Eigen::Matrix<double, 6, 1> Measurement;
  // x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad) of CoG
  Eigen::Matrix<double, 6, 1> Measurement_6dof;

  /**************************** Marine state  **************************/
  /** Motion in Marine coordinate
   * +----+----+------------+----------------+------------+--------------+
   * | 0  | 1  |      2     |        3       |   4        |     5        |
   * |x(m)|y(m)|heading(rad)| curvature(1/m) | speed(m/s) | dspeed(m/s2) |
   * +----+----+------------+----------------+------------+--------------+
   */
  Eigen::Matrix<double, 6, 1> Marine_state;

  /**************************** radar_state **************************/
  /** Motion in Marine coordinate, for radar detection
   * +----+----+------------+------------+-----------+
   * | 0  | 1  |      2     |     3      |    4      |
   * |x(m)|y(m)|heading(rad)| V_x (m/s)  | V_y (m/s) |
   * +----+----+------------+------------+-----------+
   */
  Eigen::Matrix<double, 5, 1> radar_state;
  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Eigen::Matrix<double, 6, 1> State;
  /********************* error *********************************************/
  Eigen::Matrix<double, 3, 1> p_error;  // error in surge, sway and heading
  Eigen::Matrix<double, 3, 1>
      v_error;  // velocity error in surge, sway and heading

  // estimated force in body, including thrust, wind force, etc
  Eigen::Matrix<double, 3, 1> BalphaU;
};

// real-time motion data from sensors
struct motionrawdata {
  /********************* GPS  *********************/
  double gps_x;
  double gps_y;
  double gps_z;
  double gps_roll;
  double gps_pitch;
  double gps_heading;
  double gps_Ve;
  double gps_Vn;
  double gps_roti;
};

struct sealoadRTdata {
  // wind load for wind compensation
  Eigen::Vector3d windload;
  WINDCOMPENSATION windstatus;
};

}  // namespace ASV::localization

#endif /*_ESTIMATORDATA_H_*/