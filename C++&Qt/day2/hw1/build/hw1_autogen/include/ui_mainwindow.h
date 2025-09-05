/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QSlider *bass;
    QGraphicsView *graphicsView;
    QPushButton *mani1_stop;
    QPushButton *mani1_left;
    QPushButton *mani1_right;
    QPushButton *mani2_stop;
    QPushButton *mani2_right;
    QSlider *mani_1;
    QPushButton *mani2_left;
    QPushButton *mani3_stop;
    QPushButton *mani3_left;
    QSlider *horizontalSlider_4;
    QPushButton *mani3_right;
    QPushButton *save;
    QPushButton *load;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1220, 683);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        bass = new QSlider(centralwidget);
        bass->setObjectName(QString::fromUtf8("bass"));
        bass->setGeometry(QRect(20, 100, 411, 20));
        bass->setOrientation(Qt::Horizontal);
        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(440, 0, 771, 631));
        mani1_stop = new QPushButton(centralwidget);
        mani1_stop->setObjectName(QString::fromUtf8("mani1_stop"));
        mani1_stop->setGeometry(QRect(20, 130, 89, 25));
        mani1_left = new QPushButton(centralwidget);
        mani1_left->setObjectName(QString::fromUtf8("mani1_left"));
        mani1_left->setGeometry(QRect(340, 130, 89, 25));
        mani1_right = new QPushButton(centralwidget);
        mani1_right->setObjectName(QString::fromUtf8("mani1_right"));
        mani1_right->setGeometry(QRect(180, 130, 89, 25));
        mani2_stop = new QPushButton(centralwidget);
        mani2_stop->setObjectName(QString::fromUtf8("mani2_stop"));
        mani2_stop->setGeometry(QRect(20, 280, 89, 25));
        mani2_right = new QPushButton(centralwidget);
        mani2_right->setObjectName(QString::fromUtf8("mani2_right"));
        mani2_right->setGeometry(QRect(180, 280, 89, 25));
        mani_1 = new QSlider(centralwidget);
        mani_1->setObjectName(QString::fromUtf8("mani_1"));
        mani_1->setGeometry(QRect(20, 250, 411, 20));
        mani_1->setOrientation(Qt::Horizontal);
        mani2_left = new QPushButton(centralwidget);
        mani2_left->setObjectName(QString::fromUtf8("mani2_left"));
        mani2_left->setGeometry(QRect(340, 280, 89, 25));
        mani3_stop = new QPushButton(centralwidget);
        mani3_stop->setObjectName(QString::fromUtf8("mani3_stop"));
        mani3_stop->setGeometry(QRect(20, 410, 89, 25));
        mani3_left = new QPushButton(centralwidget);
        mani3_left->setObjectName(QString::fromUtf8("mani3_left"));
        mani3_left->setGeometry(QRect(340, 410, 89, 25));
        horizontalSlider_4 = new QSlider(centralwidget);
        horizontalSlider_4->setObjectName(QString::fromUtf8("horizontalSlider_4"));
        horizontalSlider_4->setGeometry(QRect(20, 380, 411, 20));
        horizontalSlider_4->setOrientation(Qt::Horizontal);
        mani3_right = new QPushButton(centralwidget);
        mani3_right->setObjectName(QString::fromUtf8("mani3_right"));
        mani3_right->setGeometry(QRect(180, 410, 89, 25));
        save = new QPushButton(centralwidget);
        save->setObjectName(QString::fromUtf8("save"));
        save->setGeometry(QRect(100, 520, 89, 25));
        load = new QPushButton(centralwidget);
        load->setObjectName(QString::fromUtf8("load"));
        load->setGeometry(QRect(260, 520, 89, 25));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1220, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        mani1_stop->setText(QCoreApplication::translate("MainWindow", "stop", nullptr));
        mani1_left->setText(QCoreApplication::translate("MainWindow", "left", nullptr));
        mani1_right->setText(QCoreApplication::translate("MainWindow", "right", nullptr));
        mani2_stop->setText(QCoreApplication::translate("MainWindow", "stop", nullptr));
        mani2_right->setText(QCoreApplication::translate("MainWindow", "right", nullptr));
        mani2_left->setText(QCoreApplication::translate("MainWindow", "left", nullptr));
        mani3_stop->setText(QCoreApplication::translate("MainWindow", "stop", nullptr));
        mani3_left->setText(QCoreApplication::translate("MainWindow", "left", nullptr));
        mani3_right->setText(QCoreApplication::translate("MainWindow", "right", nullptr));
        save->setText(QCoreApplication::translate("MainWindow", "save", nullptr));
        load->setText(QCoreApplication::translate("MainWindow", "load", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
