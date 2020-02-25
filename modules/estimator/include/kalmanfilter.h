/*
***********************************************************************
* kalmanfilter.h: observer using kalman filter
* function to simulate the kalman filter based on Eigen
* This header file can be read by C++ compilers
*
*  Kalman Filter Class Definition.
*
*  Matrix Dimension must be:
*  x[k] = A * x[k-1] + B * u[k-1] + w[k-1]
*  z[k] = H * x[k] + v[k]
*  x: n x 1, state vector
*  z: m x 1, observer vector
*  u: l x 1, input vector
*  A: n x n
*  B: n x l
*  H: m x n
*  Q: n x n
*  R: m x m
*  I: n x n
*  P: n x n
*  K: n x m
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include <common/math/eigen/Eigen/Dense>
#include <fstream>
#include <iostream>
#include "common/property/include/vesseldata.h"
#include "estimatordata.h"

namespace ASV::localization {

// Kalman filtering for linear system
template <int l = 1, int m = 1, int n = 1>
class kalmanfilter {
 protected:
  using vectorld = Eigen::Matrix<double, l, 1>;
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnnd = Eigen::Matrix<double, n, n>;
  using matrixnld = Eigen::Matrix<double, n, l>;
  using matrixmnd = Eigen::Matrix<double, m, n>;
  using matrixnmd = Eigen::Matrix<double, n, m>;
  using matrixmmd = Eigen::Matrix<double, m, m>;

 public:
  // disable the default constructor
  kalmanfilter() = delete;
  explicit kalmanfilter(const matrixnnd &_A, const matrixnld &_B,
                        const matrixmnd &_H, const matrixnnd &_Q,
                        const matrixmmd &_R) noexcept
      : A(_A),
        B(_B),
        H(_H),
        Q(_Q),
        R(_R),
        P(matrixnnd::Identity()),
        K(matrixnmd::Zero()),
        X(vectornd::Zero()) {}

  virtual ~kalmanfilter() = default;
  /* Set Initial Value */
  void setInitial(const vectornd &_X0, const matrixnnd &_P0) {
    X = _X0;
    P = _P0;
  }

  // perform kalman filter for one step
  void linearkalman(const matrixnnd &_A, const matrixnld &_B,
                    const vectorld &former_U, const vectormd &_Z) {
    updatesystem(_A, _B);
    predict(former_U);
    correct(_Z);
  }  // linearkalman

  void linearkalman(const matrixnnd &_A, const vectorld &former_U,
                    const vectormd &_Z) {
    updatesystem(_A);
    predict(former_U);
    correct(_Z);
  }  // linearkalman

  void linearkalman(const vectorld &former_U, const vectormd &_Z) {
    predict(former_U);
    correct(_Z);
  }  // linearkalman

  // calculate the max eigenvalue of P
  double getMaxEigenP() const {
    Eigen::SelfAdjointEigenSolver<matrixnnd> eigensolver(P);
    if (eigensolver.info() != Eigen::Success)
      return 100;
    else
      return eigensolver.eigenvalues().maxCoeff();
  }  // getMaxEigenP

  vectornd getState() const noexcept { return X; }
  void setState(const vectornd &_state) { X = _state; }
  // After intialization of sensors, we can specify value to state
  void setQ(const matrixnnd &_Q) { Q = _Q; }
  void setR(const matrixmmd &_R) { R = _R; }

 protected:
  /* Fixed Matrix */
  matrixnnd A;  // System dynamics matrix
  matrixnld B;  // Control matrix
  matrixmnd H;  // Mesaurement Adaptation matrix
  matrixnnd Q;  // Process Noise Covariance matrix
  matrixmmd R;  // Measurement Noise Covariance matrix

  /* Variable Matrix */
  matrixnnd P;  // State Covariance
  matrixnmd K;  // Kalman Gain matrix
  vectornd X;   //(Current) State vector

 private:
  /* Do prediction based of physical system (No external input) */
  void predict(void) {
    X = A * X;
    P = A * P * A.transpose() + Q;
  }  // predict

  /* Do prediction based of physical system (with external input)
   * U: Control vector
   */
  void predict(const vectorld &_U) {
    X = A * X + B * _U;
    P = A * P * A.transpose() + Q;
  }  // predict

  /* Correct the prediction, using mesaurement
   *  Z: mesaure vector */
  void correct(const vectormd &_Z) {
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    // K = (P * H.transpose()) * (H * P * H.transpose() + R).llt().solve(Im);
    X = X + K * (_Z - H * X);
    P = (matrixnnd::Identity() - K * H) * P;
  }  // correct

  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const matrixnnd &_A, const matrixnld &_B) {
    A = _A;
    B = _B;
  }

  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const matrixnnd &_A) { A = _A; }
};  //  // end class kalmanfilter

// Kalman filtering for surface vessel
class USV_kalmanfilter : public kalmanfilter<3, 6, 6> {
 public:
  explicit USV_kalmanfilter(const common::vessel &_vessel,
                            const estimatordata &_estimatordata) noexcept
      : kalmanfilter(matrixnnd::Zero(), matrixnld::Zero(),
                     matrixmnd::Identity(), _estimatordata.Q, _estimatordata.R),
        sample_time(_estimatordata.sample_time) {
    initializekalman(_vessel);
  }

  ~USV_kalmanfilter() noexcept {}

  // perform kalman filter for one step
  USV_kalmanfilter &linearkalman(const estimatorRTdata &_RTdata) {
    updateKalmanA(_RTdata.CTB2G);
    kalmanfilter::linearkalman(_RTdata.BalphaU, _RTdata.Measurement);
    return *this;
  }

 private:
  const double sample_time;

  // initialize parameters in Kalman filter
  void initializekalman(const common::vessel &_vessel) {
    // copy the constant data
    Eigen::Matrix3d Mass(_vessel.Mass + _vessel.AddedMass);
    Eigen::Matrix3d Damping(_vessel.LinearDamping);

    // calcualte the A and B in continous equation
    matrixnld Bk = matrixnld::Zero();
    matrixnnd Ak = matrixnnd::Zero();
    Eigen::Matrix3d Inv_Mass = Mass.inverse();
    Ak.topRightCorner(3, 3) = Eigen::Matrix3d::Identity();
    Ak.bottomRightCorner(3, 3) = -Inv_Mass * Damping;
    Bk.bottomRows(3) = Inv_Mass;

    // calculate discrete time A, B, and H
    kalmanfilter::A = matrixnnd::Identity() + sample_time * Ak;
    kalmanfilter::B = sample_time * Bk;
  }

  // real time update the Kalman filter matrix using orientation
  void updateKalmanA(const Eigen::Matrix3d &_CTB2G) {
    kalmanfilter::A.topRightCorner(3, 3) = sample_time * _CTB2G;
  }  // updateKalmanA

};  // end class USV_kalmanfilter

}  // namespace ASV::localization

#endif /* _KALMANFILTER_H_ */