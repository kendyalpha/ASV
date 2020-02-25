/*
***********************************************************************
* testestimator.cc:
* Utility test for the state estimation and wind compenstation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "modules/estimator/include/estimator.h"
#include "modules/estimator/include/windcompensation.h"

using namespace ASV;

int main() {
  common::vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // LinearDamping
      Eigen::Matrix3d::Zero(),  // LinearDamping
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
  localization::estimatorRTdata _estimatorRTdata{
      common::STATETOGGLE::IDLE,            // state_toggle
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Marine_state
      Eigen::Matrix<double, 5, 1>::Zero(),  // radar_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  localization::sealoadRTdata _sealoadRTdata{
      Eigen::Vector3d::Zero(),                 // windload
      localization::WINDCOMPENSATION::WINDOFF  // windstatus
  };

  // estimatordata
  localization::estimatordata estimatordata_input{
      0.1,                                      // sample_time
      Eigen::Vector3d::Zero(),                  // cog2anntena_position
      Eigen::Matrix<double, 6, 6>::Identity(),  // Q
      Eigen::Matrix<double, 6, 6>::Identity()   // R
  };

  double gps_x = 1;
  double gps_y = 0;
  double altitude = 0;
  double roll = 0;
  double pitch = 0;
  double heading = 0;
  double Ve = 0;
  double Vn = 0;
  double roti = 0;

  localization::estimator<localization::USEKALMAN::KALMANON> _estimator(
      _estimatorRTdata, _vessel, estimatordata_input);

  localization::windcompensation _windcompensation(_sealoadRTdata);

  _sealoadRTdata = _windcompensation.computewindload(0, 0).getsealoadRTdata();

  _estimatorRTdata =
      _estimator
          .setvalue(gps_x, gps_y, altitude, roll, pitch, heading, Ve, Vn, roti)
          .getEstimatorRTData();

  _estimator.updateestimatedforce(Eigen::Vector3d::Zero(),
                                  _sealoadRTdata.windload);
  _estimator.estimatestate(gps_x, gps_y, altitude, roll, pitch, heading, Ve, Vn,
                           roti, 0);
  _estimator.estimateerror(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}