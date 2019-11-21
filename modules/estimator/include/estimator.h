/*
***********************************************************************
* estimator.h: state estimation of USV (3 DoF)
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include "common/math/miscellaneous/include/math_utils.h"
#include "kalmanfilter.h"
#include "lowpass.h"
#include "outlierremove.h"

namespace ASV {

template <USEKALMAN indicator_kalman, int nlp_x = 1, int nlp_y = 1,
          int nlp_z = 1, int nlp_heading = 1, int nlp_roll = 1,
          int nlp_pitch = 1, int nlp_u = 3, int nlp_v = 3, int nlp_roti = 1>
class estimator {
 public:
  explicit estimator(const estimatorRTdata& _estimatorRTdata,
                     const common::vessel& _vessel,
                     const estimatordata& _estimatordata)
      : EstimatorRTData(_estimatorRTdata),
        Motionrawdata({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        roll_outlierremove(_vessel.roll_v(1), _vessel.roll_v(0),
                           _estimatordata.sample_time),
        KalmanFilter(_vessel, _estimatordata),
        sample_time(_estimatordata.sample_time),
        antenna2cog(_estimatordata.antenna2cog) {}
  estimator() = delete;
  ~estimator() {}

  // setvalue after the initialization
  estimator& setvalue(double gps_x, double gps_y, double gps_z, double gps_roll,
                      double gps_pitch, double gps_heading, double gps_Ve,
                      double gps_Vn, double gps_roti) {
    setmotionrawdata(gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_heading,
                     gps_Ve, gps_Vn, gps_roti);
    // convert to standard unit
    convert2standardunit(Motionrawdata);

    // Coordinate transformation matrix
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 Motionrawdata.gps_heading);

    // primary antenna revision
    compensateantennadeviation(EstimatorRTData, Motionrawdata);

    // low pass filter
    initializeLowpass(EstimatorRTData);

    // outlier removal (no used in this version)
    roll_outlierremove.setlastvalue(Motionrawdata.gps_roll);

    // specify state
    EstimatorRTData.State = EstimatorRTData.Measurement;
    // Kalman filtering
    if constexpr (indicator_kalman == USEKALMAN::KALMANON)
      KalmanFilter.setState(EstimatorRTData.State);
    // compute Cartesian state
    computeCartesianState(EstimatorRTData);

    // state toggle
    EstimatorRTData.state_toggle = common::STATETOGGLE::READY;
    return *this;
  }
  // update the estimated force acting on the vessel
  estimator& updateestimatedforce(const Eigen::Vector3d& _thrust,
                                  const Eigen::Vector3d& _wind) {
    EstimatorRTData.BalphaU = _thrust + _wind;
    return *this;
  }
  // read sensor data and perform state estimation
  estimator& estimatestate(double gps_x, double gps_y, double gps_z,
                           double gps_roll, double gps_pitch,
                           double gps_heading, double gps_Ve, double gps_Vn,
                           double gps_roti, double _dheading) {
    setmotionrawdata(gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_heading,
                     gps_Ve, gps_Vn, gps_roti);
    // convert to standard unit
    convert2standardunit(Motionrawdata);
    // calculate the coordinate transform matrix
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 Motionrawdata.gps_heading, _dheading);

    // primary antenna revision
    compensateantennadeviation(EstimatorRTData, Motionrawdata);

    if constexpr (indicator_kalman == USEKALMAN::KALMANON)
      EstimatorRTData.State = KalmanFilter.linearkalman(EstimatorRTData)
                                  .getState();  // kalman filtering
    else
      performlowpass(EstimatorRTData);  // use low-pass filtering only

    // compute Cartesian state
    computeCartesianState(EstimatorRTData);

    return *this;

  }  // estimatestate

  // read sensor data and perform state estimation (simulation)
  estimator& estimatestate(const Eigen::Matrix<double, 6, 1>& _simulator_state,
                           double _dheading) {
    EstimatorRTData.Measurement = _simulator_state;
    // calculate the coordinate transform matrix
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 EstimatorRTData.Measurement(2), _dheading);

    if constexpr (indicator_kalman == USEKALMAN::KALMANON)
      EstimatorRTData.State = KalmanFilter.linearkalman(EstimatorRTData)
                                  .getState();  // kalman filtering
    else
      performlowpass(EstimatorRTData);  // use low-pass filtering only

    // compute Cartesian state
    computeCartesianState(EstimatorRTData);
    return *this;
  }

  // realtime calculation of position and velocity errors
  estimator& estimateerror(const Eigen::Vector3d& _setpoints,
                           const Eigen::Vector3d& _vsetpoints) {
    Eigen::Vector3d _perror = Eigen::Vector3d::Zero();
    for (int i = 0; i != 2; ++i)
      _perror(i) = _setpoints(i) - EstimatorRTData.State(i);
    _perror(2) = common::math::Normalizeheadingangle(_setpoints(2) -
                                                     EstimatorRTData.State(2));

    EstimatorRTData.p_error = EstimatorRTData.CTG2B * _perror;
    EstimatorRTData.v_error = _vsetpoints - EstimatorRTData.State.tail(3);
    return *this;
  }

  void setmotionrawdata(double gps_x, double gps_y, double gps_z,
                        double gps_roll, double gps_pitch, double gps_heading,
                        double gps_Ve, double gps_Vn,
                        double gps_roti) noexcept {
    Motionrawdata.gps_x = gps_x;
    Motionrawdata.gps_y = gps_y;
    Motionrawdata.gps_z = gps_z;
    Motionrawdata.gps_roll = gps_roll;
    Motionrawdata.gps_pitch = gps_pitch;
    Motionrawdata.gps_heading = gps_heading;
    Motionrawdata.gps_Ve = gps_Ve;
    Motionrawdata.gps_Vn = gps_Vn;
    Motionrawdata.gps_roti = gps_roti;
  }
  auto getEstimatorRTData() const noexcept { return EstimatorRTData; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  // real time data
  estimatorRTdata EstimatorRTData;
  motionrawdata Motionrawdata;

  // variable for low passing
  lowpass<nlp_x> x_lowpass;
  lowpass<nlp_y> y_lowpass;
  lowpass<nlp_z> z_lowpass;
  lowpass<nlp_heading> heading_lowpass;
  lowpass<nlp_roll> roll_lowpass;
  lowpass<nlp_pitch> pitch_lowpass;
  lowpass<nlp_u> u_lowpass;
  lowpass<nlp_v> v_lowpass;
  lowpass<nlp_roti> roti_lowpass;
  // variable for outlier removal
  outlierremove roll_outlierremove;
  // Kalman filtering
  USV_kalmanfilter KalmanFilter;

  const double sample_time;
  const Eigen::Vector3d antenna2cog;  // Xcog - Xantenna

  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G, double _rtheading,
                                    double desired_heading) {
    _CTG2B.setIdentity();
    _CTB2G.setIdentity();
    double cvalue = 0.0;
    double svalue = 0.0;

    if (std::abs(common::math::Normalizeheadingangle(
            _rtheading - desired_heading)) < M_PI / 36) {
      // use the fixed setpoint orientation to prevent measurement noise
      cvalue = std::cos(desired_heading);
      svalue = std::sin(desired_heading);
    } else {
      // if larger than 5 deg, we use the realtime orientation
      cvalue = std::cos(_rtheading);
      svalue = std::sin(_rtheading);
    }

    _CTG2B(0, 0) = cvalue;  //_CTG2B: global to body-fixed
    _CTG2B(1, 1) = cvalue;
    _CTG2B(0, 1) = svalue;
    _CTG2B(1, 0) = -svalue;
    _CTB2G(0, 0) = cvalue;  //_CTG2B: body-fixed to global
    _CTB2G(1, 1) = cvalue;
    _CTB2G(0, 1) = -svalue;
    _CTB2G(1, 0) = svalue;
  }

  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G,
                                    double _rtheading) {
    _CTG2B.setIdentity();
    _CTB2G.setIdentity();
    double cvalue = std::cos(_rtheading);
    double svalue = std::sin(_rtheading);

    _CTG2B(0, 0) = cvalue;
    _CTG2B(1, 1) = cvalue;
    _CTG2B(0, 1) = svalue;
    _CTG2B(1, 0) = -svalue;
    _CTB2G(0, 0) = cvalue;
    _CTB2G(1, 1) = cvalue;
    _CTB2G(0, 1) = -svalue;
    _CTB2G(1, 0) = svalue;
  }

  // GPS primary antenna is different from CoG, such deviation should be
  // considered.
  void compensateantennadeviation(estimatorRTdata& _RTdata,
                                  const motionrawdata& _motionrawdata) {
    // TODO: consider roll/pitch revision for Z deviation
    Eigen::Vector2d antenna2cog_global =
        _RTdata.CTB2G.block<2, 2>(0, 0) * antenna2cog.head(2);
    Eigen::Vector2d velocity_body =
        _RTdata.CTG2B.block<2, 2>(0, 0) *
        (Eigen::Vector2d() << _motionrawdata.gps_Vn, _motionrawdata.gps_Ve)
            .finished();
    _RTdata.Measurement(0) = antenna2cog_global(0) + _motionrawdata.gps_x;
    _RTdata.Measurement(1) = antenna2cog_global(1) + _motionrawdata.gps_y;
    _RTdata.Measurement(2) = _motionrawdata.gps_heading;
    _RTdata.Measurement(3) =
        velocity_body(0) - _motionrawdata.gps_roti * antenna2cog(1);
    _RTdata.Measurement(4) =
        velocity_body(1) + _motionrawdata.gps_roti * antenna2cog(0);
    _RTdata.Measurement(5) = _motionrawdata.gps_roti;

    _RTdata.Measurement_6dof(0) = _RTdata.Measurement(0);
    _RTdata.Measurement_6dof(1) = _RTdata.Measurement(1);
    _RTdata.Measurement_6dof(2) = antenna2cog(2) + _motionrawdata.gps_z;
    _RTdata.Measurement_6dof(3) = _motionrawdata.gps_roll;
    _RTdata.Measurement_6dof(4) = _motionrawdata.gps_pitch;
    _RTdata.Measurement_6dof(5) = _RTdata.Measurement(2);
  }  // compensateantennadeviation

  void convert2standardunit(motionrawdata& _motionrawdata) noexcept {
    // project x is the east, y is the north
    double temp = _motionrawdata.gps_x;
    _motionrawdata.gps_x = _motionrawdata.gps_y;
    _motionrawdata.gps_y = temp;
    _motionrawdata.gps_z *= (-1);
    _motionrawdata.gps_roll = common::math::Degree2Rad(_motionrawdata.gps_roll);
    _motionrawdata.gps_pitch =
        common::math::Degree2Rad(_motionrawdata.gps_pitch);
    // convert degree to rad and -pi ~ pi
    _motionrawdata.gps_heading = common::math::Normalizeheadingangle(
        common::math::Degree2Rad(_motionrawdata.gps_heading));
    _motionrawdata.gps_roti =
        common::math::Degree2Rad(_motionrawdata.gps_roti) /
        60.0;  // degree/min -> rad/s

  }  // convert2standardunit

  void initializeLowpass(const estimatorRTdata& _RTdata) {
    // low pass for position of CoG
    x_lowpass.setaveragevector(_RTdata.Measurement_6dof(0));
    y_lowpass.setaveragevector(_RTdata.Measurement_6dof(1));
    z_lowpass.setaveragevector(_RTdata.Measurement_6dof(2));
    roll_lowpass.setaveragevector(_RTdata.Measurement_6dof(3));
    pitch_lowpass.setaveragevector(_RTdata.Measurement_6dof(4));
    heading_lowpass.setaveragevector(_RTdata.Measurement_6dof(5));

    u_lowpass.setaveragevector(_RTdata.Measurement(3));
    v_lowpass.setaveragevector(_RTdata.Measurement(4));
    roti_lowpass.setaveragevector(_RTdata.Measurement(5));
  }  // initializeLowpass

  void performlowpass(estimatorRTdata& _RTdata) {
    // low pass for position of CoG
    _RTdata.State(0) = x_lowpass.movingaverage(_RTdata.Measurement(0));
    _RTdata.State(1) = y_lowpass.movingaverage(_RTdata.Measurement(1));
    _RTdata.State(2) = heading_lowpass.movingaverage(_RTdata.Measurement(2));
    _RTdata.State(3) = u_lowpass.movingaverage(_RTdata.Measurement(3));
    _RTdata.State(4) = v_lowpass.movingaverage(_RTdata.Measurement(4));
    _RTdata.State(5) = roti_lowpass.movingaverage(_RTdata.Measurement(5));

    // z_lowpass.movingaverage(_RTdata.Measurement_6dof(2));
    // roll_lowpass.movingaverage(_RTdata.Measurement_6dof(3));
    // pitch_lowpass.movingaverage(_RTdata.Measurement_6dof(4));
  }  // performlowpass

  // estimate the state for Frenet optimal trajectory generator
  void computeCartesianState(estimatorRTdata& _RTdata) {
    static double previous_cart_x = 0.0;
    static double previous_cart_y = 0.0;
    static double previous_theta = 0.0;
    static double previous_speed = 0.0;

    double _ds = std::sqrt(std::pow(_RTdata.State(0) - previous_cart_x, 2) +
                           std::pow(_RTdata.State(1) - previous_cart_y, 2));

    double _dtheta =
        common::math::Normalizeheadingangle(_RTdata.State(2) - previous_theta);
    double _curvature = 0.0;

    if (_ds > (0.05 * sample_time))
      _curvature = _dtheta / _ds;  // consider the low-speed situation

    // todo:save
    double _speed = std::sqrt(std::pow(_RTdata.State(3), 2) +
                              std::pow(_RTdata.State(4), 2));
    double _dspeed = (_speed - previous_speed) / sample_time;
    //
    _RTdata.Marine_state(0) = _RTdata.State(0);
    _RTdata.Marine_state(1) = _RTdata.State(1);
    _RTdata.Marine_state(2) = _RTdata.State(2);
    _RTdata.Marine_state(3) = _curvature;
    _RTdata.Marine_state(4) = _speed;
    _RTdata.Marine_state(5) = 0 * _dspeed;

    // update previous state
    previous_cart_x = _RTdata.State(0);
    previous_cart_y = _RTdata.State(1);
    previous_theta = _RTdata.State(2);
    previous_speed = _speed;
  }  // computeCartesianState

};  // end class estimator
}  // end namespace ASV

#endif /* _ESTIMATOR_H_ */
