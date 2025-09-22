#include <QApplication>
#include "../include/hw1_pkg/main_window.hpp"

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
