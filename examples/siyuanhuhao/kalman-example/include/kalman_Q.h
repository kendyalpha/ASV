/*
***********************************************************************
* Kalman_Q.h: thread-based DP controller and network
* function to run the whole loop on server (including TCP/IP server,
* senser, estimator, controller, planner, database, etc).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _KALMAN_Q_H_
#define _KALMAN_Q_H_

#include <pthread.h>
#include <chrono>
#include <thread>

#include "common/communication/include/tcpserver.h"
#include "common/fileIO/include/database.h"
#include "common/fileIO/include/jsonparse.h"
#include "common/logging/include/easylogging++.h"
#include "common/property/include/priority.h"
#include "common/timer/include/timecounter.h"
#include "controller/include/controller.h"
#include "estimator/include/estimator.h"
#include "messages/sensors/gpsimu/include/gps.h"

namespace ASV {

constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = USEKALMAN::KALMANON;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse("./../../properties/property.json"),
        _estimator(_estimatorRTdata, _jsonparse.getvessel(),
                   _jsonparse.getestimatordata()),
        _sqlite(_jsonparse.getsqlitedata()),
        db("../data//Mon Oct 21 09:57:25 2019.db") {
    intializethreadloop();
  }
  ~threadloop() {}

  void mainloop() {
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);

    estimator_thread.join();
    sql_thread.join();
  }

 private:
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  std::vector<control::twinfixedthrusterdata> _twinfixeddata;

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Marine_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  // real time GPS/IMU data
  std::vector<double> gps_x;
  std::vector<double> gps_y;
  std::vector<double> gps_z;
  std::vector<double> gps_roll;
  std::vector<double> gps_pitch;
  std::vector<double> gps_heading;
  std::vector<double> gps_Ve;
  std::vector<double> gps_Vn;
  std::vector<double> gps_roti;

  // stm32 data
  std::vector<double> stm32_u1;
  std::vector<double> stm32_u2;

  estimator<indicator_kalman, 1, 1, 1, 1, 1, 1> _estimator;

  common::database<num_thruster, dim_controlspace> _sqlite;

  sqlite::database db;

  void intializethreadloop() { _sqlite.initializetables(); }

  // loop to give real time state estimation
  void estimatorloop() {
    _twinfixeddata = _jsonparse.gettwinfixeddata();
    readstm32data();
    readgpsdata();

    int si = 50;

    _estimator.setvalue(gps_x[si],        // gps_x
                        gps_y[si],        // gps_y
                        gps_z[si],        // gps_z
                        gps_roll[si],     // gps_roll
                        gps_pitch[si],    // gps_pitch
                        gps_heading[si],  // gps_heading
                        gps_Ve[si],       // gps_Ve
                        gps_Vn[si],       // gps_Vn
                        gps_roti[si]      // gps_roti
    );
    while (1) {
      si++;

      _estimatorRTdata =
          _estimator
              .updateestimatedforce(calculateThrust(stm32_u1[si], stm32_u2[si]),
                                    Eigen::Vector3d::Zero())
              .estimatestate(gps_x[si],        // gps_x
                             gps_y[si],        // gps_y
                             gps_z[si],        // gps_z
                             gps_roll[si],     // gps_roll
                             gps_pitch[si],    // gps_pitch
                             gps_heading[si],  // gps_heading
                             gps_Ve[si],       // gps_Ve
                             gps_Vn[si],       // gps_Vn
                             gps_roti[si],     // gps_roti
                             0                 //_dheading
                             )
              .getEstimatorRTData();
    }

  }  // estimatorloop()

  // loop to save real time data using sqlite3 and modern_sqlite3_cpp_wrapper
  void sqlloop() {
    while (1) {
      _sqlite.update_estimator_table(_estimatorRTdata);
    }
  }  // sqlloop()

  // calculate Balpha as function of alpha
  Eigen::Vector3d calculateThrust(double sign_u1, double sign_u2) {
    Eigen::Vector3d _thrust = Eigen::Vector3d::Zero();

    double angle1 = sign_u1 > 0 ? 0 : M_PI;
    double angle2 = sign_u2 > 0 ? 0 : M_PI;
    double u1 = std::abs(sign_u1);
    double u2 = std::abs(sign_u2);

    _thrust(0) = std::cos(angle1) * u1 + std::cos(angle2) * u2;
    _thrust(1) = 0;
    _thrust(2) = -_twinfixeddata[0].ly * std::cos(angle1) * u1 -
                 _twinfixeddata[1].ly * std::cos(angle2) * u2;

    return _thrust;
  }  // calculateBalpha

  // loop to give messages to stm32
  void readstm32data() {
    db << "SELECT numbers from stm32 where name = ?;"
       << "feedback_u1" >>
        stm32_u1;
    db << "SELECT numbers from stm32 where name = ?;"
       << "feedback_u2" >>
        stm32_u2;
  }  // readstm32data()

  // read gps data  from database
  void readgpsdata() {
    db << "SELECT numbers from GPS where name = ?;"
       << "UTM_x" >>
        gps_x;
    db << "SELECT numbers from GPS where name = ?;"
       << "UTM_y" >>
        gps_y;
    db << "SELECT numbers from GPS where name = ?;"
       << "altitude" >>
        gps_z;
    db << "SELECT numbers from GPS where name = ?;"
       << "roll" >>
        gps_roll;
    db << "SELECT numbers from GPS where name = ?;"
       << "pitch" >>
        gps_pitch;
    db << "SELECT numbers from GPS where name = ?;"
       << "heading" >>
        gps_heading;
    db << "SELECT numbers from GPS where name = ?;"
       << "Ve" >>
        gps_Ve;
    db << "SELECT numbers from GPS where name = ?;"
       << "Vn" >>
        gps_Vn;
    db << "SELECT numbers from GPS where name = ?;"
       << "roti" >>
        gps_roti;

  }  // readgpsdata()

};  // namespace ASV

}  // end namespace ASV

#endif /* _KALMAN_Q_H_ */