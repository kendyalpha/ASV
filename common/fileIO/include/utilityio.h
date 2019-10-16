/*
***********************************************************************
* utilityio.h:
* The utility library for file io
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef UTILITYIO_H
#define UTILITYIO_H

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace ASV::common {

template <typename T>
std::string to_string_with_precision(const T _value, const int p = 6) {
  std::ostringstream out;
  out.precision(p);
  out << std::fixed << _value;
  return out.str();
}

template <typename T, int m, int n>
Eigen::Matrix<T, m, n> convertstdvector2EigenMat(const std::vector<T> &_vec) {
  Eigen::Matrix<T, m, n> mat = Eigen::Matrix<T, m, n>::Zero();
  for (int i = 0; i != n; ++i)
    for (int j = 0; j != m; ++j) mat(j, i) = _vec[i * m + j];
  return mat;
}
// convert std vector to Eigen3 Matrix (by column)
Eigen::MatrixXd convertstdvector2EigenMat(const std::vector<double> &_vec,
                                          int nrow, int ncol) {
  Eigen::MatrixXd mat(nrow, ncol);
  for (int i = 0; i != ncol; ++i)
    for (int j = 0; j != nrow; ++j) mat(j, i) = _vec[i * nrow + j];
  return mat;
}
// convert Eigen3 Matrix to std vector (by column)
std::vector<double> convertEigenMat2stdvector(const Eigen::MatrixXd &_mat) {
  std::vector<double> vec(_mat.data(), _mat.data() + _mat.rows() * _mat.cols());
  return vec;
}
// convert Eigen3 vector to std vector
std::vector<double> convertEigenVec2stdvector(const Eigen::VectorXd &_vec) {
  std::vector<double> vec(_vec.data(), _vec.data() + _vec.size());
  return vec;
}  // convertEigenVec2stdvector

// write Eigen matrix into csv file (double)
void write2csvfile(const std::string &name, const Eigen::MatrixXd &_matrix) {
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                         Eigen::DontAlignCols, ",", "\n");
  std::ofstream file(name.c_str());
  if (file.is_open()) {
    // header in csv
    int num_cols = _matrix.cols();
    for (int i = 0; i != (num_cols - 1); ++i) file << "column " << i + 1 << ",";
    file << "column " << num_cols << std::endl;
    // data
    file << _matrix.format(CSVFormat);
  }
}
// write Eigen vector into csv file (double)
void write2csvfile(const std::string &name, const Eigen::VectorXd &_vector) {
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                         Eigen::DontAlignCols, ",", "\n");
  std::ofstream file(name.c_str());
  if (file.is_open()) {
    // header in csv
    file << "column 1" << std::endl;
    // data
    file << _vector.format(CSVFormat);
  }
}  // write2csvfile
// write Eigen matrix into csv file (int)
void write2csvfile(const std::string &name, const Eigen::MatrixXi &_matrix) {
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                         Eigen::DontAlignCols, ",", "\n");
  std::ofstream file(name.c_str());
  if (file.is_open()) {
    // header in csv
    int num_cols = _matrix.cols();
    for (int i = 0; i != (num_cols - 1); ++i) file << "column " << i + 1 << ",";
    file << "column " << num_cols << std::endl;
    // data
    file << _matrix.format(CSVFormat);
  }
}  // write2csvfile

}  // namespace ASV::common

#endif /*UTILITYIO_H*/