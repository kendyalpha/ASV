/*
***********************************************************************
* testutilityIO.cpp:
* Utility test for the utility library for file io
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/utilityio.h"

int main() {
  Eigen::MatrixXd mat(3, 4);
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  Eigen::VectorXd Vec(12);
  Vec << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  std::cout << mat << std::endl;
  // unit test for converting Eigen3 Matrix to std vector
  std::vector<double> mat2vec = ASV::common::convertEigenMat2stdvector(mat);
  for (std::vector<double>::iterator it = mat2vec.begin(); it != mat2vec.end();
       ++it) {
    std::cout << *it << '\n';
  }

  // unit test for converting Eigen3 vector to std vector
  std::vector<double> vec2vec = ASV::common::convertEigenVec2stdvector(Vec);
  for (std::vector<double>::iterator it = vec2vec.begin(); it != vec2vec.end();
       ++it) {
    std::cout << *it << '\n';
  }

  // unit test for converting std vector to Eigen3 Matrix
  Eigen::MatrixXd vec2mat =
      ASV::common::convertstdvector2EigenMat(mat2vec, 3, 4);
  std::cout << vec2mat << std::endl;

  // unit test for writing Eigen matrix to csv file
  ASV::common::write2csvfile("../data/csvfile.csv", vec2mat);
}