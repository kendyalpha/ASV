/********************************************************************************
** Form generated from reading UI file 'GUIDemo.ui'
**
** Created by: Qt User Interface Compiler version 5.9.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUIDEMO_H
#define UI_GUIDEMO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GUIDemoClass
{
public:
    QWidget *centralWidget;
    QGroupBox *groupMultiRadar;
    QPushButton *pushMultiRadarConnect;
    QTabWidget *tabs_2;
    QWidget *tabPPI;
    QVBoxLayout *verticalLayout_tabPPI;
    QTabWidget *tabs_3;
    QWidget *tabBScan;
    QVBoxLayout *verticalLayout_tabBscan;

    void setupUi(QMainWindow *GUIDemoClass)
    {
        if (GUIDemoClass->objectName().isEmpty())
            GUIDemoClass->setObjectName(QStringLiteral("GUIDemoClass"));
        GUIDemoClass->setWindowModality(Qt::NonModal);
        GUIDemoClass->setEnabled(true);
        GUIDemoClass->resize(2050, 1050);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(GUIDemoClass->sizePolicy().hasHeightForWidth());
        GUIDemoClass->setSizePolicy(sizePolicy);
        GUIDemoClass->setMinimumSize(QSize(100, 0));
        GUIDemoClass->setMaximumSize(QSize(3000, 2000));
        QFont font;
        font.setFamily(QStringLiteral("Ubuntu"));
        font.setPointSize(11);
        GUIDemoClass->setFont(font);
        GUIDemoClass->setFocusPolicy(Qt::NoFocus);
        GUIDemoClass->setContextMenuPolicy(Qt::DefaultContextMenu);
        centralWidget = new QWidget(GUIDemoClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy1);
        groupMultiRadar = new QGroupBox(centralWidget);
        groupMultiRadar->setObjectName(QStringLiteral("groupMultiRadar"));
        groupMultiRadar->setEnabled(true);
        groupMultiRadar->setGeometry(QRect(1040, 20, 990, 71));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(groupMultiRadar->sizePolicy().hasHeightForWidth());
        groupMultiRadar->setSizePolicy(sizePolicy2);
        groupMultiRadar->setMaximumSize(QSize(1200, 200));
        groupMultiRadar->setFocusPolicy(Qt::NoFocus);
        groupMultiRadar->setContextMenuPolicy(Qt::DefaultContextMenu);
        groupMultiRadar->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        pushMultiRadarConnect = new QPushButton(groupMultiRadar);
        pushMultiRadarConnect->setObjectName(QStringLiteral("pushMultiRadarConnect"));
        pushMultiRadarConnect->setGeometry(QRect(30, 30, 150, 30));
        sizePolicy2.setHeightForWidth(pushMultiRadarConnect->sizePolicy().hasHeightForWidth());
        pushMultiRadarConnect->setSizePolicy(sizePolicy2);
        pushMultiRadarConnect->setMinimumSize(QSize(60, 20));
        pushMultiRadarConnect->setMaximumSize(QSize(200, 200));
        pushMultiRadarConnect->setCheckable(true);
        tabs_2 = new QTabWidget(centralWidget);
        tabs_2->setObjectName(QStringLiteral("tabs_2"));
        tabs_2->setEnabled(true);
        tabs_2->setGeometry(QRect(5, 5, 1030, 1020));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(1);
        sizePolicy3.setVerticalStretch(1);
        sizePolicy3.setHeightForWidth(tabs_2->sizePolicy().hasHeightForWidth());
        tabs_2->setSizePolicy(sizePolicy3);
        tabs_2->setMinimumSize(QSize(100, 100));
        tabs_2->setMaximumSize(QSize(2000, 2000));
        QFont font1;
        font1.setFamily(QStringLiteral("Ubuntu"));
        font1.setPointSize(11);
        font1.setKerning(false);
        tabs_2->setFont(font1);
        tabPPI = new QWidget();
        tabPPI->setObjectName(QStringLiteral("tabPPI"));
        tabPPI->setEnabled(false);
        sizePolicy2.setHeightForWidth(tabPPI->sizePolicy().hasHeightForWidth());
        tabPPI->setSizePolicy(sizePolicy2);
        tabPPI->setMaximumSize(QSize(30000, 16777215));
        verticalLayout_tabPPI = new QVBoxLayout(tabPPI);
        verticalLayout_tabPPI->setSpacing(6);
        verticalLayout_tabPPI->setContentsMargins(11, 11, 11, 11);
        verticalLayout_tabPPI->setObjectName(QStringLiteral("verticalLayout_tabPPI"));
        verticalLayout_tabPPI->setSizeConstraint(QLayout::SetMinimumSize);
        tabs_2->addTab(tabPPI, QString());
        tabs_3 = new QTabWidget(centralWidget);
        tabs_3->setObjectName(QStringLiteral("tabs_3"));
        tabs_3->setEnabled(true);
        tabs_3->setGeometry(QRect(1040, 580, 990, 441));
        sizePolicy3.setHeightForWidth(tabs_3->sizePolicy().hasHeightForWidth());
        tabs_3->setSizePolicy(sizePolicy3);
        tabs_3->setMinimumSize(QSize(100, 100));
        tabs_3->setMaximumSize(QSize(1800, 800));
        tabs_3->setFont(font1);
        tabBScan = new QWidget();
        tabBScan->setObjectName(QStringLiteral("tabBScan"));
        tabBScan->setEnabled(false);
        verticalLayout_tabBscan = new QVBoxLayout(tabBScan);
        verticalLayout_tabBscan->setSpacing(6);
        verticalLayout_tabBscan->setContentsMargins(11, 11, 11, 11);
        verticalLayout_tabBscan->setObjectName(QStringLiteral("verticalLayout_tabBscan"));
        tabs_3->addTab(tabBScan, QString());
        GUIDemoClass->setCentralWidget(centralWidget);

        retranslateUi(GUIDemoClass);

        tabs_2->setCurrentIndex(0);
        tabs_3->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(GUIDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *GUIDemoClass)
    {
        GUIDemoClass->setWindowTitle(QApplication::translate("GUIDemoClass", "Navico Radar SDK GUI-Demo", Q_NULLPTR));
        groupMultiRadar->setTitle(QApplication::translate("GUIDemoClass", "Multi-Radar", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        pushMultiRadarConnect->setToolTip(QApplication::translate("GUIDemoClass", "Connect to radar", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        pushMultiRadarConnect->setText(QApplication::translate("GUIDemoClass", "Connect", Q_NULLPTR));
        tabs_2->setTabText(tabs_2->indexOf(tabPPI), QApplication::translate("GUIDemoClass", "PPI", Q_NULLPTR));
        tabs_3->setTabText(tabs_3->indexOf(tabBScan), QApplication::translate("GUIDemoClass", "B-Scan", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class GUIDemoClass: public Ui_GUIDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUIDEMO_H
