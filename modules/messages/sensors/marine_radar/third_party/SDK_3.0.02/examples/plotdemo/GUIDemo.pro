QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += warn_on
#CONFIG += console
#------------------------------------------------------------
# Source Files
#------------------------------------------------------------

FORMS += \
    GUIDemo.ui \

HEADERS += \
    CustomFrames.h \
    GUIDemo.h \
    MultiRadar.h \
    QControlUtils.h \
    TabBase.h \
    TabPPI.h \

SOURCES += \
    CustomFrames.cpp \
    GUIDemo.cpp \
    MultiRadar.cpp \
    QControlUtils.cpp \
    TabBase.cpp \
    TabPPI.cpp \
    main.cpp \

#------------------------------------------------------------
# Include directories
#------------------------------------------------------------

INCLUDES = \
    ../../include \
    ../../../../../../../../

#------------------------------------------------------------
# UI Generation
#------------------------------------------------------------

CONFIG(debug,   debug|release): TARGET_DIR = debug
CONFIG(release, debug|release): TARGET_DIR = release

UI_DIR = GeneratedFiles
MOC_DIR = $${UI_DIR}/$${TARGET_DIR}
INCLUDEPATH += $${UI_DIR}

#------------------------------------------------------------
# Libraries
#------------------------------------------------------------

LIBS += -lNRPClient \
  	-lNRPPPI \
   
#------------------------------------------------------------
# Final config
#------------------------------------------------------------

#set the qmake variables
DEPENDPATH += $$INCLUDES
INCLUDEPATH += $$INCLUDES


