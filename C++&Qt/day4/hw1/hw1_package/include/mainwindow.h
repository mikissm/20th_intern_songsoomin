#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QDateTime>
#include <QHostAddress>
#include <QNetworkInterface>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnConnect_clicked();
    void on_btnSend_clicked();
    void udpDataReceived();

private:
    Ui::MainWindow *ui;

    QUdpSocket *udpSocket;
    QHostAddress remoteAddr;
    quint16 remotePort;
    quint16 localPort;

    QString getLocalIP();
    void displayMessage(const QString &msg, const QString &from);
};

#endif // MAINWINDOW_H
