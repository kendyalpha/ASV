#include "estimator.h"

int main() {
  vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // Damping
      Eigen::Vector2d::Zero(),  // cog
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
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero(),              // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero()   // motiondata_6dof
  };

  sealoadRTdata _sealoadRTdata{
      Eigen::Vector3d::Zero()  // windload
  };

  // estimatordata
  estimatordata estimatordata_input{
      0.1,                        // sample_time
      Eigen::Vector2d::Zero(),    // cog2anntena_position
      WINDCOMPENSATION::WINDOFF,  // windstatus
  };

  double gps_x = 1;
  double gps_y = 0;
  double altitude = 0;
  double roll = 0;
  double pitch = 0;
  double heading = 0;
  double Ve = 0;
  double Vn = 0;

  estimator<USEKALMAN::KALMANON> _estimator(_estimatorRTdata, _vessel,
                                            estimatordata_input);
  _estimatorRTdata =
      _estimator.setvalue(gps_x, gps_y, altitude, roll, pitch, heading, Ve, Vn)
          .getEstimatorRTData();

  _estimator.updateestimatedforce(Eigen::Vector3d::Zero(),
                                  _sealoadRTdata.windload);
  _estimator.estimatestate(gps_x, gps_y, altitude, roll, pitch, heading, Ve, Vn,
                           0);
  _estimator.estimateerror(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}