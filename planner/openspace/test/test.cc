#include <iostream>
#include <string>
int main() {
  std::string t = "test";
  std::cout << t.size() << std::endl;
  char test = t.at(1);
  if (test == 'e') std::cout << "ttt" << std::endl;
}