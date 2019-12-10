//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "MultiRadar.h"
#include <QDebug>
#include <iomanip>
#include <iostream>
#include "QControlUtils.h"
#include "ui_Registration.h"

unsigned hexstring_to_uint8(const std::string& hexText, uint8_t* pData) {
  std::size_t len = hexText.length();
  for (std::size_t i = 0; i < len; i += 2) {
    std::string byteString = hexText.substr(i, 2);
    pData[i / 2] =
        static_cast<uint8_t>(strtol(byteString.c_str(), nullptr, 16));
  }
  return static_cast<unsigned>(len / 2);
}

//-----------------------------------------------------------------------------
//  Helpers
//-----------------------------------------------------------------------------
QString ToHexString(const void* pData, int dataSize) {
  QString str;
  for (int i = 0; i < dataSize; ++i) {
    str += QString("%0").arg(static_cast<const uint8_t*>(pData)[i], 2, 16,
                             QChar('0'));
  }
  return str;
}

//-----------------------------------------------------------------------------
//  tMultiRadar Implementation
//-----------------------------------------------------------------------------
tMultiRadar::tMultiRadar(Ui::GUIDemoClass& myUI, QObject* pParent)
    : QObject(pParent), ui(myUI) {
  Connect(true, this, SIGNAL(UpdateRadarList_signal()), this,
          SLOT(MultiRadar_UpdateRadarList()), Qt::QueuedConnection);
  Connect(true, this, SIGNAL(UpdateUnlockState_signal(QString, int)), this,
          SLOT(MultiRadar_UpdateUnlockState(QString, int)),
          Qt::QueuedConnection);
  Connect(true, this, SIGNAL(GetUnlockKey_signal(const char*, QString)), this,
          SLOT(MultiRadar_GetUnlockKey(const char*, QString)),
          Qt::QueuedConnection);

  ConnectControls(true, *this, *ui.groupMultiRadar);

  Navico::Protocol::tMultiRadarClient* pClient =
      Navico::Protocol::tMultiRadarClient::GetInstance();
  pClient->AddRadarListObserver(this);
  pClient->AddUnlockStateObserver(this);
  pClient->SetUnlockKeySupplier(this);
  pClient->Connect();
  pClient->QueryRadars();
}

//-----------------------------------------------------------------------------
tMultiRadar::~tMultiRadar() {
  ConnectControls(false, *this, *ui.groupMultiRadar);

  Navico::Protocol::tMultiRadarClient* pClient =
      Navico::Protocol::tMultiRadarClient::GetInstance();
  pClient->Disconnect();
  pClient->SetUnlockKeySupplier(nullptr);
  pClient->RemoveUnlockStateObserver(this);
  pClient->RemoveRadarListObserver(this);
}

//-----------------------------------------------------------------------------
QString tMultiRadar::GetRadarSelection() {
  return ui._comboMultiRadarSelect->currentText();
}

//-----------------------------------------------------------------------------
QString tMultiRadar::GetRadarSerialNumber() {
  if (ui._comboMultiRadarSelect->count() > 0)
    return ui._comboMultiRadarSelect
        ->itemData(ui._comboMultiRadarSelect->currentIndex())
        .toStringList()[0];
  return QString();
}

//-----------------------------------------------------------------------------
unsigned tMultiRadar::GetRadarInstance() {
  if (ui._comboMultiRadarSelect->count() > 0)
    return ui._comboMultiRadarSelect
        ->itemData(ui._comboMultiRadarSelect->currentIndex())
        .toStringList()[1]
        .toUInt();
  return 0;
}

//-----------------------------------------------------------------------------
void tMultiRadar::InitiateUnlock() { MultiRadarUnlock_clicked(true); }

//-----------------------------------------------------------------------------
void tMultiRadar::SetConnectState(bool connected) {
  ui.pushMultiRadarConnect->setChecked(connected);
}

//-----------------------------------------------------------------------------
// Observer Callbacks and Handling
//-----------------------------------------------------------------------------
void tMultiRadar::UpdateRadarList(const char* /*pSerialNumber*/,
                                  iRadarListObserver::eAction /*action*/) {
  emit UpdateRadarList_signal();
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadar_UpdateRadarList() {
  int currIndex = 0;
  QString currSerialNumber = ui._comboMultiRadarSelect->currentText();
  ui._comboMultiRadarSelect->clear();

  const unsigned cMaxRadars = 8;
  char radars[cMaxRadars][MAX_SERIALNUMBER_SIZE];

  unsigned numRadars =
      Navico::Protocol::tMultiRadarClient::GetInstance()->GetRadars(radars,
                                                                    cMaxRadars);
  if (numRadars > 0) {
    if (numRadars > cMaxRadars) numRadars = cMaxRadars;
    for (unsigned i = 0; i < numRadars; ++i) {
      int numImageServices = Navico::Protocol::tMultiRadarClient::GetInstance()
                                 ->GetImageStreamCount(radars[i]);
      if (numImageServices > 0) {
        QString serialNumber(radars[i]);
        if (serialNumber == currSerialNumber) {
          currIndex = i;
        }
        for (int i = 0; i < numImageServices; ++i) {
          QStringList userData;
          userData << serialNumber << QString::number(i);

          QString itemText = serialNumber;
          if (numImageServices > 1) {
            itemText += QString(" ") + QChar('A' + i);
          }
          ui._comboMultiRadarSelect->addItem(itemText, userData);
        }
      }
    }
    ui._comboMultiRadarSelect->setCurrentIndex(currIndex);
  } else {
    ui._comboMultiRadarSelect->clearEditText();
  }
}

//-----------------------------------------------------------------------------
int tMultiRadar::GetUnlockKey(const char* pSerialNumber, const uint8_t* pLockID,
                              unsigned lockIDSize, uint8_t* /*pUnlockKey*/,
                              unsigned /*maxUnlockKeySize*/) {
  // Create a local serial number and copy content.
  size_t length = strlen(pSerialNumber) + 1;
  char* pLocalSerialNumber = new char[length];
  memcpy(pLocalSerialNumber, pSerialNumber, length);

  emit GetUnlockKey_signal(pLocalSerialNumber,
                           ToHexString(pLockID, lockIDSize));

  // can't return an unlock key immediately because we need to prompt the user
  return -1;
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadar_GetUnlockKey(const char* pSerialNumber,
                                          QString lockID) {
  QWidget* pParent = dynamic_cast<QWidget*>(parent());
  assert(pParent);

  QDialog* pDialog = new QDialog(pParent);
  assert(pDialog);

  Ui::RegistrationDialog uiReg;
  uiReg.setupUi(pDialog);
  uiReg.textLockID->setPlainText(lockID);
  pDialog->setWindowTitle(QString("Radar - ") + pSerialNumber);
  int result = pDialog->exec();

  uiReg.textUnlockKey->setPlainText(
      "FCA53C6FE25450CA93E4AF63B78769E659E56E2705AFAD443F1D6EDBEFB249FF2C7AFD75"
      "89122F80DE38FA32638C36F195816F5EE5C1257EFFED4A02537252FE");
  QString unlockKey = uiReg.textUnlockKey->toPlainText();

  delete pDialog;

  if (result) {
    uint8_t data[MAX_UNLOCKKEY_SIZE];

    unsigned len = hexstring_to_uint8(
        "FCA53C6FE25450CA93E4AF63B78769E659E56E2705AFAD443F1D6EDBEFB249FF2C7AFD"
        "75"
        "89122F80DE38FA32638C36F195816F5EE5C1257EFFED4A02537252FE",
        data);
    //    int len = FromHexString(unlockKey, data, sizeof(data));

    if (len > 0) {
      Navico::Protocol::tMultiRadarClient::GetInstance()->SetUnlockKey(
          pSerialNumber, data, len);
    } else {
      QMessageBox::critical(pParent, "Error", "Invalid unlock key entered");
    }
  }

  // Free the local serial number
  delete[] pSerialNumber;
}

//-----------------------------------------------------------------------------
void tMultiRadar::UpdateUnlockState(const char* pSerialNumber, int lockState) {
  emit UpdateUnlockState_signal(pSerialNumber, lockState);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadar_UpdateUnlockState(QString serialNumber,
                                               int lockState) {
  QString title("Unlock");
  QString radar("Radar \"" + serialNumber + "\"");

  QWidget* pParent = dynamic_cast<QWidget*>(parent());
  if (lockState >= 0)
    QMessageBox::information(
        pParent, title,
        (lockState == 0)
            ? radar + " still locked"
            : radar + QString(" unlocked (level %0)").arg(lockState));
  else
    QMessageBox::critical(pParent, title, "Unlock of " + radar + " Failed");
}

//-----------------------------------------------------------------------------
// UI Event Handling
//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarConnect_clicked(bool checked) {
  emit ConnectChanged(checked);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarQuery_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->QueryRadars();
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarReset_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->ResetDeviceIDs();
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarClear_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->ClearRadars();
}
//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarGetLockID_clicked(bool) {
  QString title("Lock ID");
  QString radar("Radar: \"" + GetRadarSerialNumber() + "\"\n\n");

  char lockID[MAX_LOCKID_SIZE * 2 + 1];
  int length = Navico::Protocol::tMultiRadarClient::GetInstance()->GetLockID(
      lockID, GetRadarSerialNumber().toLatin1().data());
  lockID[length] = '\0';
  QWidget* pParent = dynamic_cast<QWidget*>(parent());
  QMessageBox::information(pParent, title, radar + lockID);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarUnlock_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->UnlockRadar(
      GetRadarSerialNumber().toLatin1().data(), 0);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarUnlockAll_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->UnlockRadar(nullptr, 0);
}

//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarFlush_clicked(bool) {
  Navico::Protocol::tMultiRadarClient::GetInstance()->ClearUnlockKeys();
}
