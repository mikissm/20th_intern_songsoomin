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
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLineEdit *input_file_name;
    QPushButton *input_button;
    QPushButton *out_button;
    QTextEdit *name_list;
    QTextEdit *output_name;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        input_file_name = new QLineEdit(centralwidget);
        input_file_name->setObjectName(QString::fromUtf8("input_file_name"));
        input_file_name->setGeometry(QRect(180, 360, 431, 31));
        input_button = new QPushButton(centralwidget);
        input_button->setObjectName(QString::fromUtf8("input_button"));
        input_button->setGeometry(QRect(180, 420, 201, 41));
        out_button = new QPushButton(centralwidget);
        out_button->setObjectName(QString::fromUtf8("out_button"));
        out_button->setGeometry(QRect(410, 420, 201, 41));
        name_list = new QTextEdit(centralwidget);
        name_list->setObjectName(QString::fromUtf8("name_list"));
        name_list->setGeometry(QRect(180, 20, 431, 321));
        output_name = new QTextEdit(centralwidget);
        output_name->setObjectName(QString::fromUtf8("output_name"));
        output_name->setGeometry(QRect(180, 489, 431, 61));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
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
        input_file_name->setText(QString());
        input_button->setText(QCoreApplication::translate("MainWindow", "\353\254\270\354\204\234 \354\236\205\353\240\245", nullptr));
        out_button->setText(QCoreApplication::translate("MainWindow", "\353\254\270\354\204\234 \354\266\234\353\240\245", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
