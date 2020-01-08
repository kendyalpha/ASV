
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h"  // the header file of QCustomPlot. Don't forget to add it to your project, if you use an IDE, so it gets compiled.

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  void setupColorMapDemo(QCustomPlot *customPlot);

 private:
  Ui::MainWindow *ui; 
  QCPColorMap *colorMap;
  QString demoName;

  void readRadarData();
};

#endif  // MAINWINDOW_H
