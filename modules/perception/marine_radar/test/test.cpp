#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

void findelement(const uint8_t *arr, const int array_size) {
  auto results = std::find_if(arr, arr + array_size,
                              [](uint8_t element) { return element == 0x00; });

  int element_index = std::distance(arr, results);
  if (element_index < array_size)
    std::cout << "element:" << element_index << std::endl;
  else
    std::cout << "element is not present in the given array!\n";
}

int main() {
  uint8_t arr[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                     0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0xfa, 0xcc};
  findelement(arr, 16);

  std::size_t num = 1;
  double constant = 1.0;
  std::vector<double> testv(num, constant);
  for (auto i = testv.begin(); i != testv.end(); i++) {
    std::cout << *i << std::endl;
  }
}