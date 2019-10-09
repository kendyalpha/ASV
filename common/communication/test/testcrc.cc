#include <string>
#include "../include/crc.h"

using namespace std;
using namespace ASV;
int main(int argc, char* argv[]) {
  CRC16 crc16(CRC16::eCCITT_FALSE);
  string str = "IJG02F,0.01";

  char data1[] = {'1', '2', '3', '4', '5', '6', '7', '8', '0'};
  const char* data2 = str.data();
  unsigned short c1, c2;
  c1 = crc16.crcCompute(data1, 9);
  c2 = crc16.crcCompute(str.c_str(), 11);

  printf("%04x\n", c1);
  printf("%04x\n", c2);

  return 0;
}
