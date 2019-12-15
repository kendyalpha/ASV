//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "MultiRadar.h"

//-----------------------------------------------------------------------------
//  tMultiRadar Implementation
//-----------------------------------------------------------------------------
tMultiRadar::tMultiRadar(Ui::GUIDemoClass& myUI, QObject* pParent)
    : QObject(pParent), ui(myUI) {
  ConnectControls(true, *this, *ui.groupMultiRadar);

}

//-----------------------------------------------------------------------------
tMultiRadar::~tMultiRadar() {
  ConnectControls(false, *this, *ui.groupMultiRadar);
}

//-----------------------------------------------------------------------------
// UI Event Handling
//-----------------------------------------------------------------------------
void tMultiRadar::MultiRadarConnect_clicked(bool checked) {
  emit ConnectChanged(checked);
}


