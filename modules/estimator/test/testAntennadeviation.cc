/*
***********************************************************************
* testAntennadeviation.cc:
* Utility test for the state estimation and wind compenstation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "common/fileIO/recorder/include/datarecorder.h"
#include "common/timer/include/timecounter.h"
#include "modules/estimator/include/estimator.h"
#include "modules/estimator/include/windcompensation.h"
#include "modules/messages/sensors/gpsimu/include/gps.h"

using std::setprecision;
using namespace ASV;

int main() {
  // real time GPS/IMU data
  messages::gpsRTdata gps_data{
      0,    // UTC
      0,    // latitude
      0,    // longitude
      0,    // heading
      0,    // pitch
      0,    // roll
      0,    // altitude
      0,    // Ve
      0,    // Vn
      0,    // roti
      0,    // status
      0,    // UTM_x
      0,    // UTM_y
      "0n"  // UTM_zone
  };

  common::vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // LinearDamping
      Eigen::Matrix3d::Zero(),  // QuadraticDamping
      Eigen::Vector3d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      0,                        // L
      0                         // B
  };

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      common::STATETOGGLE::IDLE,            // state_toggle
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Cartesian_state
      Eigen::Matrix<double, 5, 1>::Zero(),  // radar_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  sealoadRTdata _sealoadRTdata{
      Eigen::Vector3d::Zero(),   // windload
      WINDCOMPENSATION::WINDOFF  // windstatus
  };

  // estimatordata
  estimatordata estimatordata_input{
      0.1,                                        // sample_time
      (Eigen::Vector3d() << 2, 0, 0).finished(),  // cog2anntena_position
      Eigen::Matrix<double, 6, 6>::Identity(),    // Q
      Eigen::Matrix<double, 6, 6>::Identity()     // R
  };

  const std::string db_folder = "../../data/";
  const std::string config_path =
      "../../../../common/fileIO/recorder/config/dbconfig.json";

  common::gps_db gps_db(db_folder, config_path);
  common::estimator_db estimator_db(db_folder, config_path);

  gps_db.create_table();
  estimator_db.create_table();

  common::timecounter _timer;
  messages::GPS _gpsimu(115200);  // zone 51 N
  estimator<USEKALMAN::KALMANOFF> _estimator(_estimatorRTdata, _vessel,
                                             estimatordata_input);
  _estimator.setvalue(gps_data.UTM_x, gps_data.UTM_y, gps_data.altitude,
                      gps_data.roll, gps_data.pitch, gps_data.heading,
                      gps_data.Ve, gps_data.Vn, gps_data.roti);
  windcompensation _windcompensation(_sealoadRTdata);

  int count = 0;
  long int totaltime = 0;

  while (1) {
    gps_data = _gpsimu.parseGPS().getgpsRTdata();

    ++count;
    if (count == 4) {
      count = 0;

      _sealoadRTdata =
          _windcompensation.computewindload(0, 0).getsealoadRTdata();

      _estimator.updateestimatedforce(Eigen::Vector3d::Zero(),
                                      _sealoadRTdata.windload);
      _estimatorRTdata =
          _estimator
              .estimatestate(gps_data.UTM_x, gps_data.UTM_y, gps_data.altitude,
                             gps_data.roll, gps_data.pitch, gps_data.heading,
                             gps_data.Ve, gps_data.Vn, gps_data.roti, 0)
              .getEstimatorRTData();
      _estimator.estimateerror(Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());

      long int et = _timer.timeelapsed();

      gps_db.update_table(common::gps_db_data{
          0,                   // local_time
          gps_data.UTC,        // UTC
          gps_data.latitude,   // latitude
          gps_data.longitude,  // longitude
          gps_data.heading,    // heading
          gps_data.pitch,      // pitch
          gps_data.roll,       // roll
          gps_data.altitude,   // altitude
          gps_data.Ve,         // Ve
          gps_data.Vn,         // Vn
          gps_data.roti,       // roti
          gps_data.status,     // status
          gps_data.UTM_x,      // UTM_x
          gps_data.UTM_y,      // UTM_y
          gps_data.UTM_zone    // UTM_zone
      });
      estimator_db.update_measurement_table(common::est_measurement_db_data{
          -1,                               // local_time
          _estimatorRTdata.Measurement(0),  // meas_x
          _estimatorRTdata.Measurement(1),  // meas_y
          _estimatorRTdata.Measurement(2),  // meas_theta
          _estimatorRTdata.Measurement(3),  // meas_u
          _estimatorRTdata.Measurement(4),  // meas_v
          _estimatorRTdata.Measurement(5)   // meas_r
      });
      estimator_db.update_state_table(common::est_state_db_data{
          -1,                                // local_time
          _estimatorRTdata.State(0),         // state_x
          _estimatorRTdata.State(1),         // state_y
          _estimatorRTdata.State(2),         // state_theta
          _estimatorRTdata.State(3),         // state_u
          _estimatorRTdata.State(4),         // state_v
          _estimatorRTdata.State(5),         // state_r
          _estimatorRTdata.Marine_state(3),  // curvature
          _estimatorRTdata.Marine_state(4),  // speed
          _estimatorRTdata.Marine_state(5)   // dspeed
      });
      estimator_db.update_error_table(common::est_error_db_data{
          -1,                           // local_time
          _estimatorRTdata.p_error(0),  // perror_x
          _estimatorRTdata.p_error(1),  // perror_y
          _estimatorRTdata.p_error(2),  // perror_mz
          _estimatorRTdata.v_error(0),  // verror_x
          _estimatorRTdata.v_error(1),  // verror_y
          _estimatorRTdata.v_error(2)   // verror_mz
      });

      std::cout << "UTC:      " << gps_data.UTC << std::endl;
      std::cout << "latitude:   " << std::fixed << setprecision(7)
                << gps_data.latitude << std::endl;
      std::cout << "longitude: " << std::fixed << setprecision(7)
                << gps_data.longitude << std::endl;
      std::cout << "heading:   " << std::fixed << setprecision(2)
                << gps_data.heading << std::endl;
      std::cout << "pitch:     " << std::fixed << setprecision(2)
                << gps_data.pitch << std::endl;
      std::cout << "roll:      " << std::fixed << setprecision(2)
                << gps_data.roll << std::endl;
      std::cout << "UTM_x:     " << std::fixed << setprecision(7)
                << gps_data.UTM_x << std::endl;
      std::cout << "UTM_y:     " << std::fixed << setprecision(7)
                << gps_data.UTM_y << std::endl;
      std::cout << "altitude:  " << std::fixed << setprecision(2)
                << gps_data.altitude << std::endl;
      std::cout << "Ve:   " << std::fixed << setprecision(3) << gps_data.Ve
                << std::endl;
      std::cout << "Vn:   " << std::fixed << setprecision(3) << gps_data.Vn
                << std::endl;
      std::cout << "rot:   " << std::fixed << setprecision(2) << gps_data.roti
                << std::endl;
      switch (gps_data.status) {
        case 0:
          std::cout << "GPS no fix" << std::endl;
          break;
        case 1:
          std::cout << "GPS fix" << std::endl;
          break;
        case 2:
          std::cout << "Diff GPX fix" << std::endl;
          break;
        default:
          std::cout << "Satus:     状态未知" << std::endl;
          break;
      }
      std::cout << std::endl;
    }
  }
}