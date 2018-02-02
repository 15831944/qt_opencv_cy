#include <QApplication>
#include "mainwindow.h"
#include "cy_preproc.h"
#include "cy_performance.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
