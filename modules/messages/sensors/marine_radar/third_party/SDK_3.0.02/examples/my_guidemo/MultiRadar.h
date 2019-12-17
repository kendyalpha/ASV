//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file MultiRadar.h
//!
//! User interface for handling radars - listing, selecting and unlocking.
//---------------------------------------------------------------------------
#ifndef GUIDEMO_MULTIDEVICE_H
#define GUIDEMO_MULTIDEVICE_H

#include <MultiRadarClient.h>
#include <QMainWindow>
#include <QMessageBox>
#include <QMutex>
#include <QString>
#include <QTimer>
#include <cassert>
#include <iomanip>
#include <iostream>
#include "QControlUtils.h"

#include "ui_GUIDemo.h"

//-----------------------------------------------------------------------------
// tMultiRadar
//-----------------------------------------------------------------------------
class tMultiRadar : public QObject,
                    public Navico::Protocol::iRadarListObserver,
                    public Navico::Protocol::iUnlockStateObserver,
                    public Navico::Protocol::iUnlockKeySupplier {
  Q_OBJECT

 public:
  tMultiRadar(Ui::GUIDemoClass& ui, QObject* pParent = nullptr);
  ~tMultiRadar();

  // Return the text of the current selection
  std::string GetRadarSelection() {
    return multi_radar_devices[current_radar_index][0];
  }

  // Return the serial-number of the selected radar
  std::string GetRadarSerialNumber() { return current_radar_serial_number; }
  // Return the instance/range selected
  unsigned GetRadarInstance() { return current_radar_index; }

  void InitiateUnlock();
  void SetConnectState(bool connected);

 signals:
  void ConnectChanged(bool connect);

  //-------------------------------------------------------------------------
 public:
  // iRadarListObserver, iUnlockKeySupplier, iUnlockStateObserver - multi-device
  // callbacks
  virtual void UpdateRadarList(const char* pSerialNumber,
                               iRadarListObserver::eAction action);
  virtual int GetUnlockKey(const char* pSerialNumber, const uint8_t* pLockID,
                           unsigned lockIDSize, uint8_t* pUnlockKey,
                           unsigned maxUnlockKeySize);
  virtual void UpdateUnlockState(const char* pSerialNumber, int lockState);

 private slots:
  // slots for UI actions
  void MultiRadarConnect_clicked(bool checked);
  void MultiRadarUnlock_clicked(bool checked);

 private:
  const unsigned cMaxRadars = 8;
  std::map<unsigned, std::vector<std::string>> multi_radar_devices;
  std::string current_radar_serial_number;
  unsigned current_radar_index = 0;

  unsigned hexstring_to_uint8(const std::string& hexText, uint8_t* pData) {
    std::size_t len = hexText.length();
    for (std::size_t i = 0; i < len; i += 2) {
      std::string byteString = hexText.substr(i, 2);
      pData[i / 2] =
          static_cast<uint8_t>(strtol(byteString.c_str(), nullptr, 16));
    }
    return static_cast<unsigned>(len / 2);
  }

  Ui::GUIDemoClass& ui;
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
