/*
***********************************************************************
* estimator.h: state estimation of USV
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include "kalmanfilter.h"
#include "lowpass.h"
#include "outlierremove.h"

template <USEKALMAN indicator_kalman>
class estimator {
 public:
  explicit estimator(const estimatorRTdata& _estimatorRTdata,
                     const vessel& _vessel, const estimatordata& _estimatordata)
      : EstimatorRTData(_estimatorRTdata),
        roll_outlierremove(_vessel.roll_v(1), _vessel.roll_v(0),
                           _estimatordata.sample_time),
        KalmanFilter(_vessel, _estimatordata.sample_time),
        former_heading(0),
        sample_time(_estimatordata.sample_time),
        cog2anntena_position(_estimatordata.cog2anntena_position) {}
  estimator() = delete;
  ~estimator() {}

  // setvalue after the initialization
  estimator& setvalue(double gps_x, double gps_y, double gps_z, double gps_roll,
                      double gps_pitch, double gps_heading, double gps_Ve,
                      double gps_Vn) {
    // change directions
    changedirections(gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_heading,
                     gps_Ve, gps_Vn);

    // heading rate
    former_heading = restrictheadingangle(gps_heading * M_PI / 180);
    heading_lowpass.setaveragevector(former_heading);
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 former_heading);

    // low pass for position of GPS back anntena
    x_lowpass.setaveragevector(gps_x);
    y_lowpass.setaveragevector(gps_y);
    z_lowpass.setaveragevector(gps_z);
    //
    Eigen::Vector2d velocity_body =
        EstimatorRTData.CTG2B.block<2, 2>(0, 0) *
        (Eigen::Vector2d() << gps_Vn, gps_Ve).finished();
    surgev_lowpass.setaveragevector(velocity_body(0));
    swayv_lowpass.setaveragevector(velocity_body(1));
    yawv_lowpass.setaveragevector(0);
    roll_lowpass.setaveragevector(gps_roll * M_PI / 180);
    pitch_lowpass.setaveragevector(gps_pitch * M_PI / 180);
    // outlier removal
    roll_outlierremove.setlastvalue(gps_roll * M_PI / 180);

    //
    EstimatorRTData.Measurement.head(2) =
        EstimatorRTData.CTB2G.block<2, 2>(0, 0) * cog2anntena_position +
        (Eigen::Vector2d() << gps_x, gps_y).finished();
    EstimatorRTData.Measurement(2) = former_heading;
    EstimatorRTData.Measurement.segment(3, 2) = velocity_body;
    EstimatorRTData.Measurement(5) = 0;

    EstimatorRTData.State = EstimatorRTData.Measurement;
    // Kalman filtering
    KalmanFilter.setState(EstimatorRTData.State);

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
                           double _dheading) {
    // change directions
    changedirections(gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_heading,
                     gps_Ve, gps_Vn);
    //
    EstimatorRTData.Measurement(2) = preprocessheading(gps_heading);
    // calculate the coordinate transform matrix
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 EstimatorRTData.Measurement(2), _dheading);

    computevesselposition(EstimatorRTData, gps_x, gps_y);

    computevelocity(EstimatorRTData, gps_Ve, gps_Vn);
    //
    compute6dof(EstimatorRTData, gps_z, gps_roll, gps_pitch);

    // kalman filtering
    if constexpr (indicator_kalman == USEKALMAN::KALMANON)
      EstimatorRTData.State = KalmanFilter.linearkalman(EstimatorRTData)
                                  .getState();  // kalman filtering
    else
      EstimatorRTData.State =
          EstimatorRTData.Measurement;  // use low-pass filtering only
    return *this;

  }  // estimatestate

  // read sensor data and perform state estimation (simulation)
  estimator& estimatestate(double _dheading) {
    // calculate the coordinate transform matrix
    EstimatorRTData.Measurement = EstimatorRTData.State;
    calculateCoordinateTransform(EstimatorRTData.CTG2B, EstimatorRTData.CTB2G,
                                 EstimatorRTData.Measurement(2), _dheading);
    EstimatorRTData.State = KalmanFilter.linearkalman(EstimatorRTData)
                                .getState();  // kalman filtering

    return *this;
  }

  // realtime calculation of position and velocity errors
  estimator& estimateerror(const Eigen::Vector3d& _setpoints,
                           const Eigen::Vector3d& _vsetpoints) {
    Eigen::Vector3d _perror = Eigen::Vector3d::Zero();
    for (int i = 0; i != 2; ++i)
      _perror(i) = _setpoints(i) - EstimatorRTData.State(i);
    _perror(2) = restrictheadingangle(_setpoints(2) - EstimatorRTData.State(2));
    EstimatorRTData.p_error = EstimatorRTData.CTG2B * _perror;
    EstimatorRTData.v_error = _vsetpoints - EstimatorRTData.State.tail(3);
    return *this;
  }

  auto getEstimatorRTData() const noexcept { return EstimatorRTData; }

  double getsampletime() const noexcept { return sample_time; }

 private:
  estimatorRTdata EstimatorRTData;

  // variable for low passing
  // lowpass<5> x_lowpass;
  // lowpass<5> y_lowpass;
  // lowpass<10> heading_lowpass;
  // lowpass<10> roll_lowpass;
  // lowpass<5> surgev_lowpass;
  // lowpass<5> swayv_lowpass;
  // lowpass<5> yawv_lowpass;

  lowpass<1> x_lowpass;
  lowpass<1> y_lowpass;
  lowpass<1> z_lowpass;
  lowpass<1> heading_lowpass;
  lowpass<1> roll_lowpass;
  lowpass<1> pitch_lowpass;
  lowpass<1> surgev_lowpass;
  lowpass<2> swayv_lowpass;
  lowpass<2> yawv_lowpass;
  // variable for outlier removal
  outlierremove roll_outlierremove;

  USV_kalmanfilter KalmanFilter;

  double former_heading;  // heading rate estimation
  double sample_time;
  Eigen::Vector2d cog2anntena_position;

  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G, double _rtheading,
                                    double desired_heading) {
    _CTG2B.setIdentity();
    _CTB2G.setIdentity();
    double cvalue = 0.0;
    double svalue = 0.0;

    if (abs(restrictheadingangle(_rtheading - desired_heading)) < M_PI / 36) {
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

  // low pass for heading
  double preprocessheading(double gps_heading) {
    double _gps_heading = restrictheadingangle(gps_heading * M_PI / 180);
    return heading_lowpass.movingaverage(_gps_heading);
  }

  // estimate u, v, r
  void computevelocity(estimatorRTdata& _RTdata, double gps_Ve, double gps_Vn) {
    Eigen::Vector2d velocity_body =
        _RTdata.CTG2B.block<2, 2>(0, 0) *
        (Eigen::Vector2d() << gps_Vn, gps_Ve).finished();

    _RTdata.Measurement(3) = surgev_lowpass.movingaverage(velocity_body(0));
    _RTdata.Measurement(4) = swayv_lowpass.movingaverage(velocity_body(1));
    _RTdata.Measurement(5) = yawv_lowpass.movingaverage(calheadingrate(
        _RTdata.Measurement(2)));  // we have to estimate the heading rate
  }
  // outlier removal, unit converstion, low pass filtering
  void compute6dof(estimatorRTdata& _RTdata, double gps_z, double gps_roll,
                   double gps_pitch) {
    // change direction, convert rad to deg, outlier removal or ....
    double _gps_roll = roll_outlierremove.removeoutlier(gps_roll * M_PI / 180);
    double _gps_pitch = gps_pitch * M_PI / 180;

    // update raw measured data from GPS/IMU sensors
    _RTdata.motiondata_6dof(0) = _RTdata.Measurement(0);
    _RTdata.motiondata_6dof(1) = _RTdata.Measurement(1);
    _RTdata.motiondata_6dof(2) = z_lowpass.movingaverage(gps_z);
    _RTdata.motiondata_6dof(3) = roll_lowpass.movingaverage(_gps_roll);
    _RTdata.motiondata_6dof(4) = pitch_lowpass.movingaverage(_gps_pitch);
    _RTdata.motiondata_6dof(5) = _RTdata.Measurement(2);
  }

  // calculate the heading rate
  double calheadingrate(double _newvalue) {
    double delta_yaw = restrictheadingangle(_newvalue - former_heading);
    former_heading = _newvalue;
    return delta_yaw / sample_time;
  }
  // restrict heading angle or delta heading to (-PI ~ PI)
  // compute the delta heading to find the shortest way to rotate
  double restrictheadingangle(double _heading) noexcept {
    double heading = 0;
    if (_heading > M_PI)
      heading = _heading - 2 * M_PI;
    else if (_heading < -M_PI)
      heading = _heading + 2 * M_PI;
    else
      heading = _heading;
    return heading;
  }

  // convert the location of back anntena of GPS into CoG
  void computevesselposition(estimatorRTdata& _RTdata, double gps_x,
                             double gps_y) {
    _RTdata.Measurement(0) = x_lowpass.movingaverage(gps_x);
    _RTdata.Measurement(1) = y_lowpass.movingaverage(gps_y);

    // _RTdata.Measurement.head(2) =
    //     _RTdata.CTB2G.block<2, 2>(0, 0) * cog2anntena_position +
    //     _RTdata.Measurement.head(2);
    _RTdata.Measurement.head(2) +=
        _RTdata.CTB2G.block<2, 2>(0, 0) * cog2anntena_position;
  }

  void changedirections(double& gps_x, double& gps_y, double& gps_z,
                        double& gps_roll, double& gps_pitch,
                        double& gps_heading, double& gps_Ve, double& gps_Vn) {
    // project x is the east, y is the north
    double temp = gps_x;
    gps_x = gps_y;
    gps_y = temp;
    gps_z = -gps_z;
    gps_roll = gps_roll;
    gps_pitch = gps_pitch;
    gps_heading = gps_heading;
    gps_Ve = gps_Ve;
    gps_Vn = gps_Vn;
  }
};

#endif /* _ESTIMATOR_H_ */
