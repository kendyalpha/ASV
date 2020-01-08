#include "mainwindow.h"
#include <sqlite_modern_cpp.h>
#include <QDebug>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>
#include <chrono>
#include <thread>
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow)
      {
  ui->setupUi(this);
  setGeometry(400, 250, 542, 390);

  readRadarData();


   colorMap=new QCPColorMap(ui->customPlot->xAxis, ui->customPlot->yAxis);
  setupColorMapDemo(ui->customPlot);
  setWindowTitle("QCustomPlot: " + demoName);
  statusBar()->clearMessage();
  ui->customPlot->replot();
//   std::this_thread::sleep_for(std::chrono::milliseconds(10000));
   double x, y, z;
   int nx = 200;
   int ny = 200;
   for (int xIndex = 0; xIndex < nx; ++xIndex) {
     for (int yIndex = 0; yIndex < ny; ++yIndex) {
       colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
       double r = 3 * qSqrt(x * x + y * y) + 1e-2;
       z = 2 * x * (qCos(r + 2) / r - qSin(r + 2) / r) +2;  // the B field strength of dipole radiation
                  // (modulo physical constants)
       colorMap->data()->setCell(xIndex, yIndex, z);
     }
   }


}

void MainWindow::setupColorMapDemo(QCustomPlot *customPlot) {
  demoName = "Color Map Demo";

  // configure axis rect:
  customPlot->setInteractions(
      QCP::iRangeDrag | QCP::iRangeZoom);  // this will also allow rescaling the
                                           // color scale by dragging/zooming
  customPlot->axisRect()->setupFullAxesBox(true);
  customPlot->xAxis->setLabel("x");
  customPlot->yAxis->setLabel("y");

  // set up the QCPColorMap:
  int nx = 200;
  int ny = 200;
  colorMap->data()->setSize(
      nx, ny);  // we want the color map to have nx * ny data points
  colorMap->data()->setRange(
      QCPRange(-4, 4),
      QCPRange(-4, 4));  // and span the coordinate range -4..4 in both key (x)
                         // and value (y) dimensions
  // now we assign some data, by accessing the QCPColorMapData instance of the
  // color map:
  double x, y, z;
  for (int xIndex = 0; xIndex < nx; ++xIndex) {
    for (int yIndex = 0; yIndex < ny; ++yIndex) {
      colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
      double r = 3 * qSqrt(x * x + y * y) + 1e-2;
      z = 2 * x * (qCos(r + 2) / r - qSin(r + 2) / r) ;  // the B field strength of dipole radiation
                 // (modulo physical constants)
      colorMap->data()->setCell(xIndex, yIndex, z);
    }
  }


  // add a color scale:
  QCPColorScale *colorScale = new QCPColorScale(customPlot);
  customPlot->plotLayout()->addElement(
      0, 1, colorScale);  // add it to the right of the main axis rect
  colorScale->setType(
      QCPAxis::atRight);  // scale shall be vertical bar with tick/axis labels
                          // right (actually atRight is already the default)
  colorMap->setColorScale(
      colorScale);  // associate the color map with the color scale
  colorScale->axis()->setLabel("Magnetic Field Strength");

  // set the color gradient of the color map to one of the presets:
  colorMap->setGradient(QCPColorGradient::gpPolar);
  // we could have also created a QCPColorGradient instance and added own colors
  // to the gradient, see the documentation of QCPColorGradient for what's
  // possible.

  // rescale the data dimension (color) such that all data points lie in the
  // span visualized by the color gradient:
  colorMap->rescaleDataRange();

  // make sure the axis rect and color scale synchronize their bottom and top
  // margins (so they line up):
  QCPMarginGroup *marginGroup = new QCPMarginGroup(customPlot);
  customPlot->axisRect()->setMarginGroup(QCP::msBottom | QCP::msTop,
                                         marginGroup);
  colorScale->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup);

  // rescale the key (x) and value (y) axes so the whole color map is visible:
  customPlot->rescaleAxes();
}

void MainWindow::readRadarData() {
  std::vector<double> surroundings_bearing_rad;
  std::vector<double> surroundings_range_m;
  std::vector<double> surroundings_x_m;
  std::vector<double> surroundings_y_m;
  std::vector<double> target_x;
  std::vector<double> target_y;
  std::vector<double> target_radius;

  double spoke_azimuth_deg;
  double spoke_samplerange_m;
  std::vector<uint8_t> spokedata;

  sqlite::database db("../../../data/radar.db");
  for (int _id = 1; _id != 19000; ++_id) {
    db << "SELECT bearing_rad, range_m, x_m, y_m from spoke where id = "
          "?;"
       << _id >>
        std::tie(surroundings_bearing_rad, surroundings_range_m,
                 surroundings_x_m, surroundings_y_m);
    db << "SELECT azimuth, sample_range, spokedata from radar where id = "
          "?;"
       << _id >>
        std::tie(spoke_azimuth_deg, spoke_samplerange_m, spokedata);

    db << "SELECT target_x, target_y, target_radius from target where id = "
          "?;"
       << _id >>
        std::tie(target_x, target_y, target_radius);

    for (std::size_t i = 0; i != surroundings_bearing_rad.size(); ++i)
      std::cout << " bearing_rad: " << surroundings_bearing_rad.at(i)
                << " range_m: " << surroundings_range_m.at(i)
                << " x_m: " << surroundings_x_m.at(i)
                << " y_m: " << surroundings_y_m.at(i) << std::endl;
  }
}

MainWindow::~MainWindow() {
  delete ui;
//    delete colorMap;

}
