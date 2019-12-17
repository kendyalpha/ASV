//-----------------------------------------------------------------------------
// Copyright (C) 2007-2016 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "GUIDemo.h"
//-----------------------------------------------------------------------------
// GUIDemo Class Implementation
//-----------------------------------------------------------------------------
GUIDemo::GUIDemo(QWidget* pParent, Qt::WindowFlags flags)
    : QMainWindow(pParent, flags),
      m_PixelCellSize_mm(0),
      m_pTargetLocations(nullptr) {
  InitProtocolData();

  // setup UI
  ui.setupUi(this);
  m_pMultiRadar = new tMultiRadar(ui, this);
  m_pTabBScan = new tTabBScan(ui, m_pTargetLocations, cMaxTargets, this,
                              m_OverlayManager);
  m_pTabPPI =
      new tTabPPI(ui, m_pTargetLocations, cMaxTargets, this, m_OverlayManager);

  Connect(true, m_pMultiRadar, SIGNAL(ConnectChanged(bool)), this,
          SLOT(MultiRadar_ConnectChanged(bool)));
}

//-----------------------------------------------------------------------------
GUIDemo::~GUIDemo() {
  delete m_pTabPPI;
  delete m_pTabBScan;
  delete m_pMultiRadar;

  // cleanup target-tracking
  delete[] m_pTargetLocations;
}

//-----------------------------------------------------------------------------
void GUIDemo::MultiRadar_ConnectChanged(bool connect) {
  if (connect) {
    // client services connected ok - initialise all dependent user interfaces
    m_pTabBScan->OnConnect();
    m_pTabPPI->OnConnect();

//    std::thread _PPIthread(&GUIDemo::UpdatePPI, this);
    std::thread _spokethread(&GUIDemo::UpdateSpoke, this);
//    _PPIthread.detach();
    _spokethread.detach();

  } else {
    // first disconnect all dependent user interfaces
    m_pTabBScan->OnDisconnect();
    m_pTabPPI->OnDisconnect();

    // zero data for next time we connect
    InitProtocolData();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdatePPI() {
  while (1) {
      m_PixelCellSize_mm =
            Navico::Protocol::NRP::Spoke::GetPixelCellSize_mm(mypSpoke.header);
    m_pTabBScan->OnUpdateSpoke(&mypSpoke);
    m_pTabPPI->OnUpdateSpoke(&mypSpoke);
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  }
}

void GUIDemo::UpdateSpoke() {
  union spokeheader {
    uint32_t headerdata32[11];
    char headerdata4[44];
  };
  const int recv_size = 44 + SAMPLES_PER_SPOKE / 2;
  const int send_size = 10;
  char send_buffer[send_size] = "socket";
  char recv_buffer[recv_size];

  spokeheader _spokeheader;
  tcpclient _tcpclient("127.0.0.1", "9340");

  while (1) {
    _tcpclient.senddata(recv_buffer, send_buffer, recv_size, send_size);

    for (int i = 0; i != 44; ++i) _spokeheader.headerdata4[i] = recv_buffer[i];
    for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i) {
      mypSpoke.data[i] = static_cast<uint8_t>(recv_buffer[i + 44]);
    }

    mypSpoke.header.spokeLength_bytes = _spokeheader.headerdata32[0];
    mypSpoke.header.sequenceNumber = _spokeheader.headerdata32[1];
    mypSpoke.header.nOfSamples = _spokeheader.headerdata32[2];
    mypSpoke.header.bitsPerSample = _spokeheader.headerdata32[3];
    mypSpoke.header.rangeCellSize_mm = _spokeheader.headerdata32[4];
    mypSpoke.header.spokeAzimuth = _spokeheader.headerdata32[5];
    mypSpoke.header.bearingZeroError = _spokeheader.headerdata32[6];
    mypSpoke.header.spokeCompass = _spokeheader.headerdata32[7];
    mypSpoke.header.trueNorth = _spokeheader.headerdata32[8];
    mypSpoke.header.compassInvalid = _spokeheader.headerdata32[9];
    mypSpoke.header.rangeCellsDiv2 = _spokeheader.headerdata32[10];

    m_PixelCellSize_mm =
          Navico::Protocol::NRP::Spoke::GetPixelCellSize_mm(mypSpoke.header);
  m_pTabBScan->OnUpdateSpoke(&mypSpoke);
  m_pTabPPI->OnUpdateSpoke(&mypSpoke);
//std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }


}
