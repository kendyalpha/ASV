/*
*******************************************************************************
* testUAV.cc:
* unit test for socket client using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include "crc.h"
#include "serial/serial.h"
#include "tcpclient.h"
#include "timecounter.h"

template <typename T>
std::string to_string_with_precision(const T _value, const int p = 6) {
  std::ostringstream out;
  out.precision(p);
  out << std::fixed << _value;
  return out.str();
}

union lidarmsg {
  double double_msg[3];
  char char_msg[24];
};

union crcmsg {
  unsigned short crc_short;
  unsigned char crc_char[2];
} _crcmsg;

void stm32_crc16(const char *inputCharArray, int arrayLength,
                 unsigned short *Result) {
  int k, j;
  unsigned char i = 0;
  unsigned short poly = 0x8408;
  unsigned short crcR = 0xffff;  //¢Ù
  unsigned short temp = 0;
  for (k = 0; k < arrayLength;
       k++)  //¢Þ´¦ÀíÊÕµ½ÐÅÏ¢µÄÏÂÒ»¸ö×Ö½Ú£¬Ö±µ½´¦ÀíÍêËùÓÐ×Ö½Ú
  {
    temp = inputCharArray[k];
    crcR = (crcR & 0xff00) + ((temp ^ crcR) & 0x00ff);  //¢Ú
    for (j = 0; j < 8; j++)  //¢Ý´¦Àí½ÓÊÕµ½µÄÐÅÏ¢µÄµÚÒ»¸ö×Ö½Ú£¨8Î»Êý¾Ý£
    {
      i = crcR & 0x0001;  //¢ÛµÃµ½ÒÆ³öÎ»
      crcR = (crcR >> 1);

      if (i == 1)  //¢Û¼ì²éÒÆ³öÎ»
      {
        crcR = (crcR ^ poly);  //¢ÜcrcRµÄÖµÓë¶àÏîÊ½Òì»ò
      }
    }
  }
  *Result = crcR;
}

void testuav() {
  // initialize socket TCP communication
  tcpclient _tcpclient("127.0.0.1", "9341");
  const int tcp_recv_size = 24;
  const int tcp_send_size = 10;
  lidarmsg tcp_recvmsg;
  char tcp_send_buffer[tcp_send_size] = "socket";

  // initialize serial port
  serial::Serial my_serial("/dev/ttyS5", 9600,
                           serial::Timeout::simpleTimeout(2000));
  CRC16 crc16(CRC16::eCCITT_FALSE);
  std::string serial_send_data;

  // timer
  timecounter timer_UAV;
  long int outerloop_elapsed_time = 0;
  long int innerloop_elapsed_time = 0;
  long int sample_time = 100;  // ms

  while (1) {
    outerloop_elapsed_time = timer_UAV.timeelapsed();
    // recive data from control program
    _tcpclient.senddata(tcp_recvmsg.char_msg, tcp_send_buffer, tcp_recv_size,
                        tcp_send_size);
    // serial data
    serial_send_data.clear();
    serial_send_data = "$UAV";
    for (int i = 0; i != 2; ++i) {
      serial_send_data += ",";
      serial_send_data +=
          to_string_with_precision<double>(tcp_recvmsg.double_msg[i], 7);
    }
    serial_send_data += ",";
    serial_send_data +=
        to_string_with_precision<double>(tcp_recvmsg.double_msg[2], 3);

    unsigned short crc16Result = 0;

    stm32_crc16(serial_send_data.c_str(), serial_send_data.length(),
                &crc16Result);
    serial_send_data += "," + std::to_string(crc16Result) + "\n";
    my_serial.write(serial_send_data);

    innerloop_elapsed_time = timer_UAV.timeelapsed();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

    if (outerloop_elapsed_time > 1.1 * sample_time)
      std::cout << "communication too much time!" << std::endl;
  }
}

void testserial() {
  // initialize serial port
  serial::Serial my_serial("/dev/ttyUSB0", 115200,
                           serial::Timeout::simpleTimeout(2000));
  CRC16 crc16(CRC16::eCCITT_FALSE);
  std::string serial_send_data;

  // timer
  timecounter timer_UAV;
  long int outerloop_elapsed_time = 0;
  long int innerloop_elapsed_time = 0;
  long int sample_time = 100;  // ms
  unsigned short crc16Result = 0;

  while (1) {
    outerloop_elapsed_time = timer_UAV.timeelapsed();

    //
    serial_send_data.clear();
    serial_send_data = "$UAV";

    for (int i = 0; i != 3; ++i) {
      serial_send_data += ",";
      serial_send_data += to_string_with_precision<double>(1.2223, 3);
    }
    // unsigned short _crc =
    // _crcmsg.crc_short =
    //     crc16.crcCompute(serial_send_data.c_str(),
    //     serial_send_data.length());
    stm32_crc16(serial_send_data.c_str(), serial_send_data.length(),
                &crc16Result);
    serial_send_data += "," + std::to_string(crc16Result) + "\n";
    std::size_t bytes_send = my_serial.write(serial_send_data);
    std::cout << "data_sent: " << serial_send_data << std::endl;
    std::cout << "bytes_sent: " << bytes_send << std::endl;
    printf("%x ", _crcmsg.crc_char[0]);
    printf("%x\n", _crcmsg.crc_char[1]);
    // std::cout << std::hex << _crcmsg.crc_char[0] << std::endl;
    // std::cout << std::hex << _crcmsg.crc_char[1] << std::endl;
    innerloop_elapsed_time = timer_UAV.timeelapsed();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

    if (outerloop_elapsed_time > 1.1 * sample_time)
      std::cout << "communication too much time!" << std::endl;
  }
}

int main() {
  // testuav();
  testserial();
}