#ifndef HW_UDP_GUI_HPP
#define HW_UDP_GUI_HPP

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <opencv2/opencv.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

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
    void onSenderClicked();
    void onReceiverClicked();
    void onCaptureClicked();
    void onSendClicked();
    void receiveImage();

private:
    Ui::MainWindow *ui;
    bool isSender;
    cv::Mat capturedImage;

    // UDP
    int udpSocket;
    struct sockaddr_in addr;
    QTimer* recvTimer;

    void setupUdp();
    void showImage(const cv::Mat &img);
};

#endif // HW_UDP_GUI_HPP

