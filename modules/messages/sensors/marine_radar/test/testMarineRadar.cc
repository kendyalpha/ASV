/*
****************************************************************************
* TestMarineRadar.cc:
* example for marine radar and write spoke data to sqlite3
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <sqlite_modern_cpp.h>
#include "../include/MarineRadar.h"
#include "common/communication/include/tcpserver.h"

using namespace ASV::messages;
using namespace sqlite;

// void socketserver() {
//   union spokeheader {
//     uint32_t headerdata32[11];
//     char headerdata4[44];
//   };
//   const int recv_size = 10;
//   const int send_size = 44 + SAMPLES_PER_SPOKE / 2;
//   char recv_buffer[recv_size];
//   char send_buffer[send_size];

//   ASV::tcpserver _tcpserver("9340");
//   spokeheader _spokeheader;

//   while (1) {
//     _spokeheader.headerdata32[0] = m_pSpoke.header.spokeLength_bytes;
//     _spokeheader.headerdata32[1] = m_pSpoke.header.sequenceNumber;
//     _spokeheader.headerdata32[2] = m_pSpoke.header.nOfSamples;
//     _spokeheader.headerdata32[3] = m_pSpoke.header.bitsPerSample;
//     _spokeheader.headerdata32[4] = m_pSpoke.header.rangeCellSize_mm;
//     _spokeheader.headerdata32[5] = m_pSpoke.header.spokeAzimuth;
//     _spokeheader.headerdata32[6] = m_pSpoke.header.bearingZeroError;
//     _spokeheader.headerdata32[7] = m_pSpoke.header.spokeCompass;
//     _spokeheader.headerdata32[8] = m_pSpoke.header.trueNorth;
//     _spokeheader.headerdata32[9] = m_pSpoke.header.compassInvalid;
//     _spokeheader.headerdata32[10] = m_pSpoke.header.rangeCellsDiv2;

//     for (int i = 0; i != 44; ++i) send_buffer[i] =
//     _spokeheader.headerdata4[i]; for (int i = 0; i != (SAMPLES_PER_SPOKE /
//     2); ++i)
//       send_buffer[i + 44] = static_cast<char>(m_pSpoke.data[i]);

//     _tcpserver.selectserver(recv_buffer, send_buffer, recv_size, send_size);
//   }
// }

void readsqlitedata() {
  database db("radar.db");
  std::vector<uint8_t> numbers_test;
  db << "SELECT numbers from person where id = ?;" << 1 >> numbers_test;
}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // radar
  MarineRadar _MarineRadar;
  _MarineRadar.StartMarineRadar();

  // sqlite3
  database db("radar.db");
  db << "CREATE TABLE person (id integer primary key autoincrement not "
        "null, angle INT, numbers BLOB);";

  while (1) {
    auto spokedata = _MarineRadar.getSpoke();

    db << "INSERT INTO person (angle, numbers) VALUES (?, ?)"
       << spokedata.header.spokeAzimuth
       << std::vector<uint8_t>(&spokedata.data[0],
                               &spokedata.data[SAMPLES_PER_SPOKE / 2]);

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}