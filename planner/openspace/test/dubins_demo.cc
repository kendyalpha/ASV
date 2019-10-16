#include <stdio.h>
#include "dubins.h"

using namespace ASV::planning;

int printConfiguration(double q[3], double x, void* user_data) {
  printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
  return 0;
}

int main() {
  double q0[] = {0, 0, 0};
  double q1[] = {4, 4, 3.142};
  DubinsPath path;
  dubins _dp;
  _dp.dubins_shortest_path(&path, q0, q1, 1.0);

  printf("#x,y,theta,t\n");
  _dp.dubins_path_sample_many(&path, 0.1, printConfiguration, NULL);

  return 0;
}
