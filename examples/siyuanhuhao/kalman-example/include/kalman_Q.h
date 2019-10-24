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
        db("./../../data/Mon Oct 21 09:57:25 2019.db") {
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

  void intializethreadloop() {
    _sqlite.initializetables();
    _twinfixeddata = _jsonparse.gettwinfixeddata();
    readstm32data();
    readgpsdata();
  }

  // loop to give real time state estimation
  void estimatorloop() {
    common::timecounter timer_estimator;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _estimator.getsampletime());

    std::size_t si = 4247;

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
      outerloop_elapsed_time = timer_estimator.timeelapsed();

      if (si < gps_x.size()) {
        Eigen::Vector3d thrust = calculateThrust(stm32_u1[si], stm32_u2[si]);
        // std::cout << thrust << std::endl;
        _estimatorRTdata =
            _estimator.updateestimatedforce(thrust,
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
      } else {
        std::cout << "estimiation done!\n";
      }

      si++;

      innerloop_elapsed_time = timer_estimator.timeelapsed();

      std::cout << innerloop_elapsed_time << std::endl;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));
    }
  }  // estimatorloop()

  // loop to save real time data using sqlite3 and modern_sqlite3_cpp_wrapper
  void sqlloop() {
    while (1) {
      _sqlite.update_estimator_table(_estimatorRTdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    db << "select feedback_u1 from stm32;" >>
        [&](double _x) { stm32_u1.push_back(_x); };
    db << "select feedback_u2 from stm32;" >>
        [&](double _x) { stm32_u2.push_back(_x); };

  }  // readstm32data()

  // read gps data  from database
  void readgpsdata() {
    db << "select UTM_x from GPS;" >> [&](double _x) { gps_x.push_back(_x); };
    db << "select UTM_y from GPS;" >> [&](double _x) { gps_y.push_back(_x); };
    db << "select altitude from GPS;" >>
        [&](double _x) { gps_z.push_back(_x); };
    db << "select roll from GPS;" >> [&](double _x) { gps_roll.push_back(_x); };
    db << "select pitch from GPS;" >>
        [&](double _x) { gps_pitch.push_back(_x); };
    db << "select heading from GPS;" >>
        [&](double _x) { gps_heading.push_back(_x); };
    db << "select Ve from GPS;" >> [&](double _x) { gps_Ve.push_back(_x); };
    db << "select Vn from GPS;" >> [&](double _x) { gps_Vn.push_back(_x); };
    db << "select roti from GPS;" >> [&](double _x) { gps_roti.push_back(_x); };

  }  // readgpsdata()

};  // namespace ASV

}  // end namespace ASV

#endif /* _KALMAN_Q_H_ */