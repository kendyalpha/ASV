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
class tMultiRadar : public QObject {
  Q_OBJECT

 public:
  tMultiRadar(Ui::GUIDemoClass& ui, QObject* pParent = nullptr);
  ~tMultiRadar();

 signals:
  void ConnectChanged(bool connect);

 private slots:
  // slots for UI actions
  void MultiRadarConnect_clicked(bool checked);

 private:
  Ui::GUIDemoClass& ui;
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
