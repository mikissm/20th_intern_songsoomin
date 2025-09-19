#include "hw2_pkg/main_window.hpp"
#include "hw2_pkg/qnode.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

