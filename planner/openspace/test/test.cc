#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include "common/timer/include/timecounter.h"

std::tuple<double, char, std::string> get_student(int id) {
  if (id == 0)
    return {3.8, 'A', "Lisa Simpson"};
  else if (id == 1)
    return {2.9, 'C', "Milhouse Van Houten"};
  else
    return {1.7, 'D', "Ralph Wiggum"};
}

void get_student(int id, double &d, char &c, std::string &s) {
  if (id == 0) {
    d = 3.8;
    c = 'A';
    s = "Lisa Simpson";
  } else if (id == 1) {
    d = 2.9;
    c = 'C';
    s = "Milhouse Van Houten";
  } else {
    d = 1.7;
    c = 'D';
    s = "Ralph Wiggum";
  }
}

int main() {
  ASV::common::timecounter _timer;

  // C++17 structured binding:

  for (int i = 0; i != 100000000; ++i) {
    double gpa1;
    char grade1;
    std::string name1;

    auto [gpa2, grade2, name2] = get_student(2);

    // double gpa2 = 0;
    // char grade2 = '0';
    // std::string name2 = "";
    // get_student(2, gpa2, grade2, name2);

    // std::cout << "ID: 2, "
    //           << "GPA: " << gpa2 << ", "
    //           << "grade: " << grade2 << ", "
    //           << "name: " << name2 << '\n';
  }

  long int et = _timer.timeelapsed();
  std::cout << et << std::endl;
}