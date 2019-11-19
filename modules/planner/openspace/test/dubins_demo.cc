/*
***********************************************************************
* dubins_demo.cc:
* Utility test for dubins path generator
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <stdio.h>
#include <fstream>
#include "dubins.h"

using namespace ASV::planning;

int printConfiguration(double q[3], double x, void* user_data) {
  printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
  return 0;
}

int csvConfiguration(double q[3], double x, void* user_data) {
  std::ofstream outfile;
  outfile.open("../../data/dubins.csv", std::ios_base::app);
  outfile << q[0] << "," << q[1] << "," << q[2] << "," << x << "\n";
  outfile.close();

  return 0;
}

int main() {
  double q0[] = {0, 0, 0};
  double q1[] = {4, -4, 0};
  DubinsPath path;
  dubins _dp;

  _dp.dubins_shortest_path(&path, q0, q1, 1.0);

  std::ofstream outfile;
  outfile.open("../../data/dubins.csv");
  outfile << "X,Y,theta,t\n";
  outfile.close();
  _dp.dubins_path_sample_many(&path, 0.1, csvConfiguration, NULL);

  return 0;
}
