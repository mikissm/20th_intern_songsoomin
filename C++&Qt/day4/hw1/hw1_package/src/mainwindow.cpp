#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , udpSocket(new QUdpSocket(this))
{
    ui->setupUi(this);

    // 내 IP 자동 표시
    ui->lineEditLocalIP->setText(getLocalIP());
    ui->textEditRecv->setReadOnly(true);

    connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::udpDataReceived);
}

MainWindow::~MainWindow()
{
    delete ui;
}

QString MainWindow::getLocalIP()
{
    QList<QHostAddress> ipList = QNetworkInterface::allAddresses();
    for (auto &ip : ipList) {
        if (ip != QHostAddress::LocalHost && ip.protocol() == QAbstractSocket::IPv4Protocol)
            return ip.toString();
    }
    return "127.0.0.1";
}

void MainWindow::on_btnConnect_clicked()
{
    localPort = ui->lineEditLocalPort->text().toUShort();
    remoteAddr = QHostAddress(ui->lineEditRemoteIP->text());
    remotePort = ui->lineEditRemotePort->text().toUShort();

    if (udpSocket->bind(QHostAddress::Any, localPort)) {
        displayMessage("UDP 소켓 바인드 완료 (포트 " + QString::number(localPort) + ")", "SYSTEM");
    } else {
        displayMessage("UDP 소켓 바인드 실패", "SYSTEM");
    }
}

void MainWindow::on_btnSend_clicked()
{
    QString msg = ui->lineEditSend->text();
    if (msg.isEmpty()) return;

    QByteArray data = msg.toUtf8();
    udpSocket->writeDatagram(data, remoteAddr, remotePort);

    displayMessage(msg, "Me → " + remoteAddr.toString() + ":" + QString::number(remotePort));
}

void MainWindow::udpDataReceived()
{
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        QString msg = QString::fromUtf8(datagram);
        displayMessage(msg, sender.toString() + ":" + QString::number(senderPort));
    }
}

void MainWindow::displayMessage(const QString &msg, const QString &from)
{
    QString timestamp = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ");
    ui->textEditRecv->append(timestamp + "【" + from + "】 " + msg);
}
