/*
***********************************************************************
* testjson.cc:
* Utility test for the JSON
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "jsonparse.h"

int main() {
  // read a JSON file
  ASV::jsonparse<6, 3> _jsonparse("../data/test.json");
  _jsonparse.readjson();
  std::cout << _jsonparse << std::endl;
}