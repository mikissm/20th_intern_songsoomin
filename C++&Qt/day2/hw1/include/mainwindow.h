#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QTimer>

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
    // 자동 회전 갱신
    void updateRotation();

    // Base Arm 버튼
    void rotateBaseClockwise();
    void rotateBaseCounterClockwise();
    void stopBase();

    // Shoulder Arm 버튼
    void rotateShoulderClockwise();
    void rotateShoulderCounterClockwise();
    void stopShoulder();

    // Elbow Arm 버튼
    void rotateElbowClockwise();
    void rotateElbowCounterClockwise();
    void stopElbow();

    // 저장/불러오기
    void saveState();
    void loadState();

private:
    Ui::MainWindow *ui;

    QGraphicsScene *scene;
    QGraphicsRectItem *baseArm;
    QGraphicsRectItem *shoulderArm;
    QGraphicsRectItem *elbowArm;

    QTimer *timer;

    // 각 축 회전 방향 (0: 정지, 1: 시계, -1: 반시계)
    int baseDir;
    int shoulderDir;
    int elbowDir;
};

#endif // MAINWINDOW_H
