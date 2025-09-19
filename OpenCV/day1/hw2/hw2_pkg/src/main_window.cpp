#include "hw2_pkg/main_window.hpp"
#include "hw2_pkg/qnode.hpp"

#include "ui_mainwindow.h"

#include <QPixmap>
#include <QImage>
#include <QDebug>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include <fcntl.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), isSender(false)
{
    ui->setupUi(this);

    // 버튼 슬롯 연결
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onSenderClicked);
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::onReceiverClicked);
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::onCaptureClicked);
    connect(ui->pushButton_4, &QPushButton::clicked, this, &MainWindow::onSendClicked);

    // UDP 초기화
    setupUdp();

    // 수신용 타이머
    recvTimer = new QTimer(this);
    connect(recvTimer, &QTimer::timeout, this, &MainWindow::receiveImage);
    recvTimer->start(30); // 30ms마다 수신
}

MainWindow::~MainWindow() {
    ::close(udpSocket);
    delete ui;
}

void MainWindow::setupUdp() {
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        qDebug() << "Socket creation failed";
    }

    // 기본 주소는 localhost, 포트 9000
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9000);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
}

void MainWindow::onSenderClicked() {
    isSender = true;
    ui->pushButton->setStyleSheet("background-color: green");
    ui->pushButton_2->setStyleSheet("");
}

void MainWindow::onReceiverClicked() {
    isSender = false;
    ui->pushButton_2->setStyleSheet("background-color: blue");
    ui->pushButton->setStyleSheet("");

    // 자기 IP 표시
    struct ifaddrs *ifAddrStruct = nullptr;
    getifaddrs(&ifAddrStruct);
    for (struct ifaddrs *ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
            void* tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if (strcmp(ifa->ifa_name, "lo") != 0) {
                ui->lineEdit->setText(addressBuffer);      // IP
                if(ui->lineEdit_2->text().isEmpty()) ui->lineEdit_2->setText("9000"); // 기본 포트
                break;
            }
        }
    }
    if (ifAddrStruct) freeifaddrs(ifAddrStruct);

    // 포트 읽기
    int port = ui->lineEdit_2->text().toInt();
    if (port <= 0) port = 9000;

    // 기존 소켓 닫고 새로 생성 후 바인딩
    ::close(udpSocket);
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        qDebug() << "Socket creation failed";
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udpSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        qDebug() << "Bind failed";
    }
}

void MainWindow::onCaptureClicked() {
    if (!isSender) return;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        qDebug() << "Camera open failed";
        return;
    }
    cap >> capturedImage;
    if (capturedImage.empty()) return;

    // 해상도 줄이기
    cv::resize(capturedImage, capturedImage, cv::Size(640, 480));

    showImage(capturedImage);
}

void MainWindow::onSendClicked() {
    if (!isSender || capturedImage.empty()) return;

    QString ipStr = ui->lineEdit->text();
    QString portStr = ui->lineEdit_2->text();
    if (ipStr.isEmpty() || portStr.isEmpty()) return;

    int port = portStr.toInt();

    // 송신용 소켓 생성
    int sendSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendSock < 0) {
        qDebug() << "Socket creation failed";
        return;
    }

    struct sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    inet_pton(AF_INET, ipStr.toStdString().c_str(), &destAddr.sin_addr);

    // JPEG 인코딩 (압축률 70)
    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", capturedImage, buf, params);

    // UDP 전송
    int n = sendto(sendSock, reinterpret_cast<char*>(buf.data()), buf.size(), 0,
                   reinterpret_cast<struct sockaddr*>(&destAddr), sizeof(destAddr));
    if (n < 0) qDebug() << "Send failed:" << strerror(errno);

    ::close(sendSock);
}

void MainWindow::receiveImage() {
    if (isSender) return;

    char buf[65536];
    struct sockaddr_in senderAddr;
    socklen_t len = sizeof(senderAddr);

    int n = recvfrom(udpSocket, buf, sizeof(buf), MSG_DONTWAIT,
                     (struct sockaddr*)&senderAddr, &len);
    if (n > 0) {
        cv::Mat raw(1, n, CV_8UC1, buf);
        cv::Mat img = cv::imdecode(raw, cv::IMREAD_COLOR);
        if (!img.empty()) {
            showImage(img);
        }
    }
}

void MainWindow::showImage(const cv::Mat &img) {
    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    QImage qimg((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
    ui->label->setPixmap(QPixmap::fromImage(qimg).scaled(ui->label->size(), Qt::KeepAspectRatio));
}

