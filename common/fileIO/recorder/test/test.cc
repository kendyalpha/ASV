#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
int main() {
  std::unordered_map<std::string, double> test;
  test["C"] = 1;
  test["B"] = 2;
  test["A"] = 3;

  for (auto const& [name, type] : test) {
    std::cout << name << " " << type;
  }
}