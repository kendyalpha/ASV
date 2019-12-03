#include "marine_radar.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Marine_radar w;
    w.show();
    return a.exec();
}
