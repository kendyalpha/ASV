//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "ScannerInfo.h"
#include "QControlUtils.h"

#include <cassert>

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

inline double UnitsConvertMetersX10ToNauticalMiles_double(int metersX10) {
  return static_cast<double>(metersX10 / 18520.0);
}

inline int UnitsConvertNauticalMilesToMeters_double(double nauticalMiles) {
  return static_cast<int>(nauticalMiles * 1852.0);
}

QString ToString(Navico::Protocol::NRP::eRadarErrorType type) {
  switch (type) {
    case Navico::Protocol::NRP::eErrorPersistenceCorrupt:
      return "Persistence Corrupt";
    case Navico::Protocol::NRP::eErrorZeroBearingFault:
      return "Zero Bearing Fault";
    case Navico::Protocol::NRP::eErrorBearingPulseFault:
      return "Bearing Pulse Fault";
    case Navico::Protocol::NRP::eErrorMotorNotRunning:
      return "Motor Not Running";
    case Navico::Protocol::NRP::eErrorCommsNotActive:
      return "CommsNotActive";
    case Navico::Protocol::NRP::eErrorMagnetronHeaterVoltage:
      return "Magnetron Heater Voltage";
    case Navico::Protocol::NRP::eErrorModulationVoltage:
      return "Modulation Voltage";
    case Navico::Protocol::NRP::eErrorTriggerFault:
      return "Trigger Fault";
    case Navico::Protocol::NRP::eErrorVideoFault:
      return "Video Fault";
    case Navico::Protocol::NRP::eErrorFanFault:
      return "Fan Fault";
    case Navico::Protocol::NRP::eErrorScannerConfigFault:
      return "Scanner Config Fault";
    case Navico::Protocol::NRP::eErrorPowerSupplyTransient:
      return "Power Supply Transient";
    case Navico::Protocol::NRP::eErrorScannerDetectFail:
      return "Scanner Detect Fail";
    case Navico::Protocol::NRP::eErrorPASoftOverheat:
      return "PA Soft-Overheat";
    case Navico::Protocol::NRP::eErrorPAHardOverheat:
      return "PA Hard-Overheat";
    case Navico::Protocol::NRP::eErrorGWDatapathError:
      return "GW Datapath Error";
    case Navico::Protocol::NRP::eErrorPSUOverheat:
      return "PSU Overheat";
    case Navico::Protocol::NRP::eErrorPSUVoltage:
      return "PSU Voltage";
    case Navico::Protocol::NRP::eErrorPSUPower:
      return "PSU Power";
    case Navico::Protocol::NRP::eErrorPSUHWFault:
      return "PSU HW-Fault";
    case Navico::Protocol::NRP::eErrorPSUCommsFault:
      return "PSU Comms-Fault";
    case Navico::Protocol::NRP::eErrorMechanicalFault:
      return "Mechanical Fault";
    case Navico::Protocol::NRP::eErrorLEDFault:
      return "LED Fault";
    case Navico::Protocol::NRP::eErrorScannerFail:
      return "Scanner Fail";
    case Navico::Protocol::NRP::eErrorRIFault:
      return "Radar-Interface Fault";
    case Navico::Protocol::NRP::eErrorLowBattery:
      return "Low Battery";
    case Navico::Protocol::NRP::eErrorMotorStall:
      return "Motor Stall";
    case Navico::Protocol::NRP::eErrorSafetyMode:
      return "Safety Mode";
    default:
      return QString("Radar Error #%1 (0x%2)").arg(type).arg(type, 0, 16);
  }
}

//-----------------------------------------------------------------------------
// tScannerInfo Implementation
//-----------------------------------------------------------------------------
tScannerInfo::tScannerInfo(Ui::GUIDemoClass& myUI, QWidget* pParent)
    : QObject(pParent), m_pImageClient(nullptr), ui(myUI) {
  uint32_t major, minor, build;
  Navico::Protocol::NRP::tImageClient::GetVersion(major, minor, build);
  ui.labelSDKVersion->setText(tr("%1.%2.%3").arg(major).arg(minor).arg(build));

  ui.comboScannerRange->addItem("1/64 Nm", 29);
  ui.comboScannerRange->addItem("1/48 Nm", 39);
  ui.comboScannerRange->addItem("1/32 Nm", 58);
  ui.comboScannerRange->addItem("1/24 Nm", 77);
  ui.comboScannerRange->addItem("1/16 Nm", 116);
  ui.comboScannerRange->addItem("1/8 Nm", 232);
  ui.comboScannerRange->addItem("1/4 Nm", 463);
  ui.comboScannerRange->addItem("1/2 Nm", 926);
  ui.comboScannerRange->addItem("3/4 Nm", 1389);
  ui.comboScannerRange->addItem("1 Nm", 1852);
  ui.comboScannerRange->addItem("1.5 Nm", 2778);
  ui.comboScannerRange->addItem("2 Nm", 3704);
  ui.comboScannerRange->addItem("3 Nm", 5556);
  ui.comboScannerRange->addItem("4 Nm", 7408);
  ui.comboScannerRange->addItem("6 Nm", 11112);
  ui.comboScannerRange->addItem("8 Nm", 14816);
  ui.comboScannerRange->addItem("12 Nm", 22224);
  ui.comboScannerRange->addItem("16 Nm", 29632);
  ui.comboScannerRange->addItem("24 Nm", 44448);
  ui.comboScannerRange->addItem("36 Nm", 66672);
  ui.comboScannerRange->addItem("48 Nm", 88896);
  ui.comboScannerRange->addItem("64 Nm", 118528);
  ui.comboScannerRange->addItem("72 Nm", 133344);
  ui.comboScannerRange->addItem("96 Nm", 177792);

  ConnectControls(true, *this, *ui.groupScanner);
}

//-----------------------------------------------------------------------------
tScannerInfo::~tScannerInfo() {
  ConnectControls(false, *this, *ui.groupScanner);
}

//-----------------------------------------------------------------------------
void tScannerInfo::OnConnect(
    Navico::Protocol::NRP::tImageClient* pImageClient) {
  assert(m_pImageClient == NULL);

  m_pImageClient = pImageClient;
  ui.groupScanner->setEnabled(m_pImageClient != nullptr);
  m_pImageClient->SetAutoSendClientWatchdog(
      ui.checkScannerKickRadar->isChecked());
}

//----------------------------------------------------------
void tScannerInfo::OnDisconnect() {
  ui.groupScanner->setEnabled(false);
  m_pImageClient = nullptr;
}

//-----------------------------------------------------------------------------
void tScannerInfo::OnModeChanged(Navico::Protocol::NRP::tMode* pMode) {
  switch (pMode->state) {
    case Navico::Protocol::NRP::eOff:
      ui.editScannerState->setText(tr("Off"));
      ui.checkScannerPower->setChecked(false);
      ui.checkScannerTransmit->setChecked(false);
      break;
    case Navico::Protocol::NRP::eStandby:
      ui.editScannerState->setText(tr("Standby"));
      ui.checkScannerPower->setChecked(true);
      ui.checkScannerTransmit->setChecked(false);
      break;
    case Navico::Protocol::NRP::eTransmit:
      ui.editScannerState->setText(tr("Transmit"));
      ui.checkScannerPower->setChecked(true);
      ui.checkScannerTransmit->setChecked(true);
      break;
    case Navico::Protocol::NRP::eWarming:
      ui.editScannerState->setText(tr("Warming up"));
      ui.checkScannerPower->setChecked(true);
      ui.checkScannerTransmit->setChecked(false);
      break;
    case Navico::Protocol::NRP::eNoScanner:
      ui.editScannerState->setText(tr("No Scanner"));
      ui.checkScannerPower->setChecked(false);
      ui.checkScannerTransmit->setChecked(false);
      break;
    case Navico::Protocol::NRP::eDetectingScanner:
      ui.editScannerState->setText(tr("Detecting Scanner"));
      ui.checkScannerPower->setChecked(true);
      ui.checkScannerTransmit->setChecked(false);
      break;
    default:
      ui.editScannerState->setText(tr("Undefined"));
      break;
  }

  if (pMode->warmupTime_sec == 0) {
    ui.editScannerTimeout->setText(tr("%1").arg(pMode->ttCount_sec));
  } else {
    ui.editScannerTimeout->setText(tr("%1").arg(pMode->warmupTime_sec));
  }
}

//-----------------------------------------------------------------------------
void tScannerInfo::OnPropertiesChanged(
    Navico::Protocol::NRP::tProperties* pProperties) {
  switch (pProperties->scannerType) {
    case Navico::Protocol::NRP::eTypeNoScanner:
      ui.editScannerType->setText(tr("No Scanner"));
      break;
    case Navico::Protocol::NRP::eNKE_1065:
      ui.editScannerType->setText(tr("JRC 2KW Radome"));
      break;
    case Navico::Protocol::NRP::eNKE_249:
      ui.editScannerType->setText(tr("JRC 4KW Radome"));
      break;
    case Navico::Protocol::NRP::eNKE_250_4:
      ui.editScannerType->setText(tr("JRC 6KW 4ft OA"));
      break;
    case Navico::Protocol::NRP::eNKE_250_4_NAX:
      ui.editScannerType->setText(tr("JRC 6KW 4ft N OA"));
      break;
    case Navico::Protocol::NRP::eNKE_2102_6:
      ui.editScannerType->setText(tr("JRC 10KW 6ft OA"));
      break;
    case Navico::Protocol::NRP::eNKE_2252_7:
      ui.editScannerType->setText(tr("JRC 25KW 7ft OA"));
      break;
    case Navico::Protocol::NRP::eNKE_2252_9:
      ui.editScannerType->setText(tr("JRC 25KW 9ft OA"));
      break;
    case Navico::Protocol::NRP::e4kWSimulator:
      ui.editScannerType->setText(tr("4kW Simulator"));
      break;
    case Navico::Protocol::NRP::eGWTestScanner:
      ui.editScannerType->setText(tr("GWTest"));
      break;
    case Navico::Protocol::NRP::eFMCW400_BR24:
      ui.editScannerType->setText(tr("Navico BR-24"));
      break;
    case Navico::Protocol::NRP::eFMCW400_Simulator:
      ui.editScannerType->setText(tr("Navico BR-24 Sim"));
      break;
    case Navico::Protocol::NRP::eFMCW400_HD3G:
      ui.editScannerType->setText(tr("Navico BRHD-3G"));
      break;
    case Navico::Protocol::NRP::eFMCW400_HD4G:
      ui.editScannerType->setText(tr("Navico BRHD-4G"));
      break;
    case Navico::Protocol::NRP::ePCOMP_HALO:
      ui.editScannerType->setText(tr("Navico BRHALO"));
      break;
    case Navico::Protocol::NRP::ePROP_MAGGIE:
      ui.editScannerType->setText(tr("Navico-Pro Maggie"));
      break;
    default:
      ui.editScannerType->setText(tr("Type %1").arg(pProperties->scannerId));
  }

  QString s1 = QString::fromUtf16((const ushort*)(pProperties->buildDate));
  QString s2 = QString::fromUtf16((const ushort*)(pProperties->buildTime));

  // the build number is appended to the build time, extract the parts
  QStringList sl = s2.split(" ");
  QString build = "";
  QString time = "";
  if (sl.size() != 2) {
    build = "Error";
    time = s2;
  } else {
    build = sl[1];
    time = sl[0];
  }

  ui.labelNavRadarSWBuildDate->setText(tr("%1 %2").arg(s1).arg(time));
  ui.labelNavRadarSWVersion->setText(tr("%1.%2.%3")
                                         .arg(pProperties->radarSwVersionMajor)
                                         .arg(pProperties->radarSwVersionMinor)
                                         .arg(build));
  ui.labelDriverVersion->setText(tr("%1.%2")
                                     .arg(pProperties->driverVersionMajor)
                                     .arg(pProperties->driverVersionMinor));

  ui.labelScannerSWVersion->setText(
      tr("%1.%2-%3")
          .arg(pProperties->scannerSwVersionMajor)
          .arg(pProperties->scannerSwVersionMinor)
          .arg(pProperties->scannerSwVersionBuild));
  ui.labelMaxRange->setText(tr("%1 Nm").arg(
      UnitsConvertMetersX10ToNauticalMiles_double(pProperties->maxRange_dm)));
  ui.labelOperationTime->setText(
      tr("%1 h").arg(pProperties->operationTime_hour));
  ui.labelCycleCount->setText(tr("%1").arg(pProperties->powerCycles));

  ui.labelRegVersion->setText(tr("%1.%2")
                                  .arg(pProperties->gwRegVersionMajor)
                                  .arg(pProperties->gwRegVersionMinor));
  ui.labelGWVersion->setText(tr("%1.%2")
                                 .arg(pProperties->gwVersionMajor)
                                 .arg(pProperties->gwVersionMinor));
  ui.labelGWCompiled->setText(tr("0x%1").arg(
      QString::number(pProperties->gwCompileSource, 16).toUpper()));
}

//-----------------------------------------------------------------------------
void tScannerInfo::OnSetupChanged(Navico::Protocol::NRP::tSetup* pSetup) {
  ui.editScannerRange->setText(
      tr("%1 m ").arg(pSetup->range_dm / 10.0, 4, 'f', 1));
}

//-----------------------------------------------------------------------------
void tScannerInfo::OnRadarErrorChanged(
    Navico::Protocol::NRP::tRadarError* pError) {
  ui.editScannerError->setText(
      ToString(Navico::Protocol::NRP::eRadarErrorType(pError->type)));
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerPower_clicked(bool checked) {
  if (m_pImageClient) {
    m_pImageClient->SetPower(checked);
    ui.checkScannerPower->setChecked(false);
  }
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerTransmit_clicked(bool checked) {
  if (m_pImageClient) {
    m_pImageClient->SetTransmit(checked);
    ui.checkScannerTransmit->setChecked(false);
  }
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerKickRadar_clicked(bool checked) {
  if (m_pImageClient) m_pImageClient->SetAutoSendClientWatchdog(checked);
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerQuery_clicked(bool /*checked*/) {
  if (m_pImageClient) m_pImageClient->QueryAll();
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerError_clicked(bool /*checked*/) {
  ui.editScannerError->setText(QString());
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerRange_valueChanged(int value) {
  if (m_pImageClient) m_pImageClient->SetRange(value);
}

//-----------------------------------------------------------------------------
void tScannerInfo::ScannerRange_currentIndexChanged(int /*index*/) {
  ui.spinScannerRange->setValue(
      ui.comboScannerRange->itemData(ui.comboScannerRange->currentIndex())
          .toUInt());
}
