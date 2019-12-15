//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file CustomFrames.h
//!
//! Custom frames for BScan & PPI widgets. Providing a common interface for
//! handling the display and acquiring of tracked targets.
//-----------------------------------------------------------------------------

#ifndef GUIDEMO_CUSTOMFRAMES_H
#define GUIDEMO_CUSTOMFRAMES_H

#include <QFrame>
#include <QMutex>

#include <NavRadarProtocol.h>
#include <RadarColourLookUpTable.h>
#include <QContextMenuEvent>
#include <QImage>
#include <QMenu>
#include <QMutexLocker>
#include <QPainter>
#include <QTransform>


//-----------------------------------------------------------------------------
// Target Locations
//-----------------------------------------------------------------------------
struct tTargetLocation {
  bool isValid;
  double sample;
  double degrees;
};

//-----------------------------------------------------------------------------
// tOverlay structure
//-----------------------------------------------------------------------------
struct tOverlay {
  bool enabled;
  uint32_t startRange_m;
  uint32_t endRange_m;
  uint16_t startBearing_deg;
  uint16_t endBearing_deg;
  QColor lineColour;
  QColor fillColour;
};

//-----------------------------------------------------------------------------
// tOverlayManager Class
//-----------------------------------------------------------------------------
class tOverlayManager {
 public:
  tOverlayManager(){ Clear();}
  void Clear(){memset(&m_Overlays, 0, sizeof(m_Overlays));}
  const tOverlay* GetGuardZone(int index){
      if (index < Navico::Protocol::NRP::cMaxGuardZones) {
        return &m_Overlays[index];
      } else {
        return nullptr;
      }

  }
  void SetGuardZone(int index, bool enabled, uint32_t startRange_m,
                    uint32_t endRange_m, uint16_t startBearing_deg,
                    uint16_t width_deg){
      if (index < Navico::Protocol::NRP::cMaxGuardZones) {
        uint16_t halfWidth_deg = width_deg / 2;
        tOverlay& overlay = m_Overlays[index];
        overlay.enabled = enabled;
        overlay.startRange_m = startRange_m;
        overlay.endRange_m = endRange_m;
        overlay.startBearing_deg = (startBearing_deg >= halfWidth_deg)
                                       ? (startBearing_deg - halfWidth_deg)
                                       : (startBearing_deg + 360u - halfWidth_deg);
        overlay.endBearing_deg = (startBearing_deg + halfWidth_deg < 360u)
                                     ? (startBearing_deg + halfWidth_deg)
                                     : (startBearing_deg + halfWidth_deg - 360u);
        overlay.lineColour = Qt::yellow;
        overlay.fillColour = Qt::yellow;
        overlay.fillColour.setAlpha(100);
      }

  }

 private:
  tOverlay m_Overlays[Navico::Protocol::NRP::cMaxGuardZones +
                      Navico::Protocol::NRP::cMaxBlankSectors];
};

//-----------------------------------------------------------------------------
// tQCustomFrame Class
//-----------------------------------------------------------------------------
class tQCustomFrame : public QFrame {
  Q_OBJECT

 public:
  tQCustomFrame(tTargetLocation* pTargets, unsigned maxTargets,
                QWidget* pWidget, QImage* pImage,
                tOverlayManager& overlayManager);
  virtual ~tQCustomFrame() {}

  void setImage(QImage* pImage = nullptr) { m_pImage = pImage; }
  QImage* getImage() { return m_pImage; }
  void SetFullRange_m(uint32_t fullRange_m) { m_FullRange_m = fullRange_m; }
  void SetBearing_deg(float bearing_deg) { m_Bearing_deg = bearing_deg; }

  static QMutex* getImageMutex() { return &m_ImageMutex; }

 signals:
  void AcquireTarget(double sample, double degrees);

 protected:
  virtual void paintEvent(QPaintEvent* /*pEvent*/);
  virtual void contextMenuEvent(QContextMenuEvent* pEvent);
  virtual void addActions(QMenu& /*menu*/) {}
  virtual void performAction(QAction& /*action*/) {}
  virtual void convertXYtoSD(int x, int y, double& s, double& d) = 0;
  virtual void convertSDtoXY(double s, double d, int& x, int& y) = 0;
  virtual void DrawOverlay(QPainter& painter, const tOverlay* pOverlay) = 0;

  uint32_t m_FullRange_m;
  float m_Bearing_deg;
  //    Navico::tRadarColourLookUpTableNavico  gNavicoLUT;

 private:
  QImage* m_pImage;
  unsigned m_MaxTargets;
  tTargetLocation* m_pTargets;
  unsigned m_TargetRadius;
  static QMutex m_ImageMutex;

  tOverlayManager& m_OverlayManager;
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
