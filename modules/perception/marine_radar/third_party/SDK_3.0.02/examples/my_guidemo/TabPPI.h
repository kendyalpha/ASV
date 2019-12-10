//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file TabPPI.h
//!
//! User interface for PPI display, BScan display
//!  that displays spokes using a polar coordinate system.
//---------------------------------------------------------------------------

#ifndef GUIDEMO_TABPPI_H
#define GUIDEMO_TABPPI_H

#define _USE_MATH_DEFINES

#include <ImageClient.h>
#include <PPIController.h>
#include <QDebug>
#include <QMenu>
#include <QObject>
#include <QPainter>
#include <QTimer>
#include <cassert>
#include <cmath>
#include "CustomFrames.h"
#include "ImageClient.h"
#include "QControlUtils.h"
#include "ui_GUIDemo.h"

static Navico::tRadarColourLookUpTableNavico gNavicoLUT;

//-----------------------------------------------------------------------------
// tQPPIFram Helper Class
//-----------------------------------------------------------------------------
class tQPPIFrame : public tQCustomFrame {
  Q_OBJECT

 public:
  tQPPIFrame(tTargetLocation* pTargets, unsigned maxTargets, QWidget* pParent,
             QImage* pImage, tOverlayManager& overlayManager)
      : tQCustomFrame(pTargets, maxTargets, pParent, pImage, overlayManager),
        m_TrailsTime(-1) {}

 signals:
  void ChangeTrailsTime(int secs);

 protected:
  void addActions(QMenu& menu);
  void performAction(QAction& action);
  void convertXYtoSD(int x, int y, double& s, double& d);
  void convertSDtoXY(double s, double d, int& x, int& y);

  void DrawOverlay(QPainter& painter, const tOverlay* pOverlay);

 private:
  int m_TrailsTime;
};

//-----------------------------------------------------------------------------
// tTabPPI Class
//-----------------------------------------------------------------------------
class tTabPPI : public QObject {
  Q_OBJECT

 public:
  tTabPPI(Ui::GUIDemoClass& ui, tTargetLocation* pTargets, unsigned maxTargets,
          QObject* pParent, tOverlayManager& overlayManager);
  ~tTabPPI();

  void OnConnect();
  void OnDisconnect();

  void OnUpdateSpoke(const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke);

 signals:
  void AcquireTarget(double sample, double degrees);

 private slots:
  void Timer_timeout();
  void Frame_ChangeTrailsTime(int secs);

 private:
  tQPPIFrame* m_pFrame;
  Navico::Image::tPPIController* m_pController;
  QImage* m_pImage;
  QTimer m_Timer;
  Ui::GUIDemoClass& ui;
};

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//! tQBScanFrame Helper Class
//-----------------------------------------------------------------------------
class tQBScanFrame : public tQCustomFrame {
  Q_OBJECT

 public:
  tQBScanFrame(tTargetLocation* pTargets, unsigned maxTargets, QWidget* pParent,
               QImage* pImage, tOverlayManager& overlayManager)
      : tQCustomFrame(pTargets, maxTargets, pParent, pImage, overlayManager) {}

 protected:
  void convertXYtoSD(int x, int y, double& s, double& d);
  void convertSDtoXY(double s, double d, int& x, int& y);
  void DrawOverlay(QPainter& painter, const tOverlay* pOverlay);
};

//-----------------------------------------------------------------------------
//! tTabBScan Class
//-----------------------------------------------------------------------------
class tTabBScan : public QObject {
  Q_OBJECT

 public:
  tTabBScan(Ui::GUIDemoClass& ui, tTargetLocation* pTargets,
            unsigned maxTargets, QObject* pParent,
            tOverlayManager& overlayManager);
  ~tTabBScan();

  void OnConnect();
  void OnDisconnect();

  void OnUpdateSpoke(const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke);

 signals:
  void AcquireTarget(double sample, double degrees);

 private slots:
  void Timer_timeout();

 private:
  tQBScanFrame* m_pFrame;
  QTimer m_Timer;
  QImage* m_pImage;
  unsigned m_NumSamples;

  Ui::GUIDemoClass& ui;
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
