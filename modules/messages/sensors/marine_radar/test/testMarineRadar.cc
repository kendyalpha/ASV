/*
****************************************************************************
* testMarineRadar.cc:
* example for marine radar and write spoke data to sqlite3
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include "common/communication/include/tcpserver.h"
#include "common/fileIO/recorder/include/datarecorder.h"
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"

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

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // radar
  MarineRadar _MarineRadar;
  _MarineRadar.StartMarineRadar();

  // sqlite3
  const std::string config_path =
      "../../../../common/fileIO/recorder/config/dbconfig.json";
  ASV::common::marineradar_db marineradar_db("../../data/radar.db",
                                             config_path);
  marineradar_db.create_table();

  while (1) {
    auto MarineRadar_RTdata = _MarineRadar.getMarineRadarRTdata();

    marineradar_db.update_table(ASV::common::marineradar_db_data{
        0,                                       // local_time
        MarineRadar_RTdata.spoke_azimuth_deg,    // azimuth_deg
        MarineRadar_RTdata.spoke_samplerange_m,  // sample_range
        std::vector<uint8_t>(
            &MarineRadar_RTdata.spokedata[0],
            &MarineRadar_RTdata.spokedata[SAMPLES_PER_SPOKE / 2])  // spokedata
    });
    std::cout << "spoke_azimuth_deg: " << MarineRadar_RTdata.spoke_azimuth_deg
              << std::endl;
    for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i) {
      printf("%u\n", MarineRadar_RTdata.spokedata[i]);
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
