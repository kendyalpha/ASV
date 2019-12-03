#include "marine_radar.h"
#include "./ui_marine_radar.h"

Marine_radar::Marine_radar(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Marine_radar)
{
    ui->setupUi(this);
}

Marine_radar::~Marine_radar()
{
    delete ui;
}

