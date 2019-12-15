//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "QControlUtils.h"

//-----------------------------------------------------------------------------
tQDoubleIntConnector::tQDoubleIntConnector(QDoubleSpinBox* pSpin,
                                           QSlider* pSlider,
                                           const QString& name)
    : QObject(pSlider), m_pSpin(pSpin), m_pSlider(pSlider) {
  setObjectName(name);
}
//-----------------------------------------------------------------------------
QString ToItemName(unsigned value, QComboBox* pCombo) {
  return (pCombo == nullptr || value >= unsigned(pCombo->count()))
             ? QString::number(value)
             : pCombo->itemText(static_cast<int>(value));
}

//-----------------------------------------------------------------------------
QString Slot(const QString& slot) { return QString::number(QSLOT_CODE) + slot; }

//-----------------------------------------------------------------------------
bool Connect(bool connect, const QObject* sender, const char* signal,
             const QObject* receiver, const char* method,
             Qt::ConnectionType type) {
  bool result;
  if (connect == true)
    result = QObject::connect(sender, signal, receiver, method, type);
  else
    result = QObject::disconnect(sender, signal, receiver, method);

  assert(result);
  return result;
}

//-----------------------------------------------------------------------------
void LinkControls(bool connect, QObject& manager, QWidget& parent) {
  // TODO: make this more efficient

  QList<QSpinBox*> spins(parent.findChildren<QSpinBox*>(QRegExp("^spin")));
  foreach (QSpinBox* pSpin, spins) {
    QString name(pSpin->objectName().mid(4));
    Connect(connect, pSpin, SIGNAL(valueChanged(int)), &manager,
            Slot(name + "_valueChanged(int)").toLatin1().data());

    QSlider* pSlider = parent.findChild<QSlider*>("slider" + name);
    if (pSlider != nullptr) {
      Connect(connect, pSlider, SIGNAL(sliderMoved(int)), pSpin,
              SLOT(setValue(int)));
      Connect(connect, pSpin, SIGNAL(valueChanged(int)), pSlider,
              SLOT(setValue(int)));
    }
  }

  QList<QDoubleSpinBox*> dspins(
      parent.findChildren<QDoubleSpinBox*>(QRegExp("^spin")));
  foreach (QDoubleSpinBox* pSpin, dspins) {
    QString name(pSpin->objectName().mid(4));
    Connect(connect, pSpin, SIGNAL(valueChanged(double)), &manager,
            Slot(name + "_valueChanged(double)").toLatin1().data());

    QSlider* pSlider = parent.findChild<QSlider*>("slider" + name);
    if (pSlider != nullptr) {
      tQDoubleIntConnector* pConnector =
          (connect == true) ? new tQDoubleIntConnector(pSpin, pSlider, name)
                            : pSlider->findChild<tQDoubleIntConnector*>(name);

      assert(pConnector != nullptr);
      if (pConnector != nullptr) {
        Connect(connect, pSlider, SIGNAL(sliderMoved(int)), pConnector,
                SLOT(SliderValueChanged(int)));
        Connect(connect, pSpin, SIGNAL(valueChanged(double)), pConnector,
                SLOT(SpinnerValueChanged(double)));

        if (connect == false) delete pConnector;
      }
    }
  }

  QList<QComboBox*> combos(parent.findChildren<QComboBox*>(QRegExp("^combo")));
  foreach (QComboBox* pCombo, combos) {
    QString name(pCombo->objectName().mid(5));
    Connect(connect, pCombo, SIGNAL(currentIndexChanged(int)), &manager,
            Slot(name + "_currentIndexChanged(int)").toLatin1().data());
  }

  QList<QCheckBox*> checks(parent.findChildren<QCheckBox*>(QRegExp("^check")));
  foreach (QCheckBox* pCheck, checks) {
    QString name(pCheck->objectName().mid(5));
    Connect(connect, pCheck, SIGNAL(clicked(bool)), &manager,
            Slot(name + "_clicked(bool)").toLatin1().data());
  }

  QList<QPushButton*> pushes(
      parent.findChildren<QPushButton*>(QRegExp("^push")));
  foreach (QPushButton* pPush, pushes) {
    QString name(pPush->objectName().mid(4));
    Connect(connect, pPush, SIGNAL(clicked(bool)), &manager,
            Slot(name + "_clicked(bool)").toLatin1().data());
  }
}

//-----------------------------------------------------------------------------
void ConnectControls(bool connect, QObject& manager, QWidget& parent) {
  LinkControls(connect, manager, parent);
}

//-----------------------------------------------------------------------------
void DisconnectControls(QObject& manager, QWidget& parent) {
  LinkControls(false, manager, parent);
}
