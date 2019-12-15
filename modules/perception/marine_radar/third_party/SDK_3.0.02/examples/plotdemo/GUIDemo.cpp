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

    std::thread _spokethread(&GUIDemo::UpdateSpoke, this);
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
void GUIDemo::UpdateSpoke() {
  while (1) {
    mypSpoke.header.spokeAzimuth++;
    m_PixelCellSize_mm =
        Navico::Protocol::NRP::Spoke::GetPixelCellSize_mm(mypSpoke.header);
    m_pTabBScan->OnUpdateSpoke(&mypSpoke);
    m_pTabPPI->OnUpdateSpoke(&mypSpoke);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
