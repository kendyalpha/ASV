//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file QControlUtils.h
//!
//! Various Qt helper functions and classes.
//-----------------------------------------------------------------------------

#ifndef NAVICO_QCONTROLUTILS_H
#define NAVICO_QCONTROLUTILS_H

#include <NavRadarProtocol.h>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <cassert>
//-----------------------------------------------------------------------------
// tQDoubleIntConnector Class
//-----------------------------------------------------------------------------

/** Simple intermediate class for connecting a double spin-box to an integer
   slider. If the ranges of the spin-box and slider are different this class
   will automatically rescale values transferred between the two **/
class tQDoubleIntConnector : public QObject {
  Q_OBJECT

 public:
  tQDoubleIntConnector(QDoubleSpinBox* pSpin, QSlider* pSlider,
                       const QString& name);

 private:
  QDoubleSpinBox* m_pSpin;
  QSlider* m_pSlider;
};

//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------

QString ToItemName(unsigned value, QComboBox* pCombo);
QString Slot(const QString& slot);

/** Function that will assert if a Qt dis/connect fails **/
bool Connect(bool connect, const QObject* sender, const char* signal = nullptr,
             const QObject* receiver = nullptr, const char* method = nullptr,
             Qt::ConnectionType type = Qt::AutoConnection);

/** Function that will scan all sub-controls of 'parent' and automatically
   connect them to an appropriately named slot assumed to exist on 'manager' **/
void ConnectControls(bool connect, QObject& manager, QWidget& parent);

//-----------------------------------------------------------------------------

#endif
