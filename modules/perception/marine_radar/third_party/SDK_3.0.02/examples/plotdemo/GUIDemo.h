//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file GUIDemo.h
//!
//! Window for testing/demonstrating the NRP library protocol
//-----------------------------------------------------------------------------
#ifndef GUIDEMO_H
#define GUIDEMO_H

#include <ClientErrors.h>
#include <Feature.h>
#include <FeatureManager.h>
#include <ImageClient.h>
#include <ImageClientObserver.h>
#include <MultiRadarClient.h>
#include <NavRadarProtocol.h>
#include <PPIController.h>
#include <TargetTrackingClient.h>

#include <QMainWindow>
#include <QMessageBox>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include "MultiRadar.h"
#include "QControlUtils.h"
#include "TabPPI.h"

#include "common/communication/include/tcpclient.h"
#include "ui_GUIDemo.h"
//-----------------------------------------------------------------------------
// GUIDemo Class
//-----------------------------------------------------------------------------

enum { eGuardZone1 = 0, eGuardZone2 };

class GUIDemo : public QMainWindow {
  Q_OBJECT

 public:
  GUIDemo(QWidget* parent = nullptr, Qt::WindowFlags flags = nullptr);
  ~GUIDemo();

 private:
  //-----------------------------------------------------------------------------
  void InitProtocolData() {
    // setup trget-tracking protocol data structures
    InitStruct(m_pTargetLocations, cMaxTargets);
  }
  void UpdateSpoke();

 private:
  unsigned m_PixelCellSize_mm;
  static const unsigned cMaxTargets = 10;

 private:
  void updateboatstate() {
      union spokeheader {
        uint32_t headerdata32[11];
        char headerdata4[44];
      };
    const int recv_size =  45 + SAMPLES_PER_SPOKE / 2;
    const int send_size = 10;
    spokeheader _recvmsg;
    char send_buffer[send_size] = "socket";
char recv_buffer[recv_size];
    tcpclient _tcpclient("127.0.0.1", "9340");

    while (1) {
      _tcpclient.senddata(recv_buffer, send_buffer, recv_size, send_size);
      memcpy(_recvmsg.headerdata4, recv_buffer, 44);
      memcpy(_recvmsg.headerdata4, recv_buffer, 44);

    }
  }

 private slots:
  // slots for callbacks
  void MultiRadar_ConnectChanged(bool connect);

 private:
  // working variables
  tTargetLocation* m_pTargetLocations;

 private:
  struct Navico::Protocol::NRP::Spoke::t9174Spoke mypSpoke {
    {
        12,  // spokeLength_bytes
        12,  // sequenceNumber
        12,  // nOfSamples
        4,   // bitsPerSample
        16,  // rangeCellSize_mm
        13,  // spokeAzimuth
        1,   // bearingZeroError
        14,  // spokeCompass
        1,   // trueNorth
        1,   // compassInvalid
        16   // rangeCellsDiv2
    },
    {
      0xFA, 0x11, 0x28, 0x33, 0x00, 0x34, 0x23, 0x5a, 0x54, 0x00, 0xff, 0xff,
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    }
  };

  tMultiRadar* m_pMultiRadar;
  tTabBScan* m_pTabBScan;
  tTabPPI* m_pTabPPI;

  Ui::GUIDemoClass ui;

  tOverlayManager m_OverlayManager;
  bool m_AlarmTypes[Navico::Protocol::NRP::cMaxGuardZones];

  template <typename Type>
  inline void InitStruct(Type*& ptr, unsigned count = 1) {
    if (ptr == nullptr) {
      if (count > 1)
        ptr = new Type[count];
      else
        ptr = new Type;
    }

    if (ptr != nullptr) {
      if (count > 1)
        memset(ptr, 0, count * sizeof(Type));
      else
        memset(ptr, 0, sizeof(Type));
    }
  }
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
