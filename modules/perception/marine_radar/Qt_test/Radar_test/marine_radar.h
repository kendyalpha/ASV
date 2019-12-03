#ifndef MARINE_RADAR_H
#define MARINE_RADAR_H

#include <QMainWindow>


//#include <ImageClient.h>
//#include <ImageClientObserver.h>
//#include <TargetTrackingClient.h>
//#include <MultiRadarClient.h>
//#include <PPIController.h>
//#include <NavRadarProtocol.h>
//#include <ClientErrors.h>
//#include <TargetTrackingClient.h>
//#include <Feature.h>
//#include <FeatureManager.h>

//#include "MultiRadar.h"
//#include "ScannerInfo.h"
//#include "TabImage.h"
//#include "TabInstallation.h"
//#include "TabAdvanced.h"
//#include "TabGuardZone.h"
//#include "TabTargets.h"
//#include "TabBScan.h"
//#include "TabPPI.h"
//#include "TabFeatures.h"
//#include "TabSectorBlanking.h"
//#include "OverlayManager.h"



QT_BEGIN_NAMESPACE
namespace Ui { class Marine_radar; }
QT_END_NAMESPACE

class Marine_radar : public QMainWindow
{
    Q_OBJECT

public:
    Marine_radar(QWidget *parent = nullptr);
    ~Marine_radar();

private:
    Ui::Marine_radar *ui;
};
#endif // MARINE_RADAR_H
