#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 슬라이더 범위 0~359 설정
    ui->bass->setRange(0, 359);          // Base
    ui->mani_1->setRange(0, 359);        // Shoulder
    ui->horizontalSlider_4->setRange(0, 359); // Elbow


    // Scene 생성
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    scene->setSceneRect(-200, -200, 400, 400);

    // Base Arm (Axis 1, 얇게)
    baseArm = scene->addRect(-5, -50, 10, 100, QPen(Qt::blue), QBrush(Qt::blue));
    baseArm->setTransformOriginPoint(0, 50);

    // Shoulder Arm (Axis 2)
    shoulderArm = scene->addRect(-4, -50, 8, 100, QPen(Qt::red), QBrush(Qt::red));
    shoulderArm->setParentItem(baseArm);
    shoulderArm->setPos(0, -100);
    shoulderArm->setTransformOriginPoint(0, 50);

    // Elbow Arm (Axis 3)
    elbowArm = scene->addRect(-3, -50, 6, 100, QPen(Qt::green), QBrush(Qt::green));
    elbowArm->setParentItem(shoulderArm);
    elbowArm->setPos(0, -100);
    elbowArm->setTransformOriginPoint(0, 50);

    // 초기 회전 방향
    baseDir = shoulderDir = elbowDir = 0;

    // Timer (자동 회전)
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateRotation);
    timer->start(10); // 10ms마다 갱신

    // 버튼 연결
    connect(ui->mani1_left, &QPushButton::clicked, this, &MainWindow::rotateBaseCounterClockwise);
    connect(ui->mani1_right, &QPushButton::clicked, this, &MainWindow::rotateBaseClockwise);
    connect(ui->mani1_stop, &QPushButton::clicked, this, &MainWindow::stopBase);

    connect(ui->mani2_left, &QPushButton::clicked, this, &MainWindow::rotateShoulderCounterClockwise);
    connect(ui->mani2_right, &QPushButton::clicked, this, &MainWindow::rotateShoulderClockwise);
    connect(ui->mani2_stop, &QPushButton::clicked, this, &MainWindow::stopShoulder);

    connect(ui->mani3_left, &QPushButton::clicked, this, &MainWindow::rotateElbowCounterClockwise);
    connect(ui->mani3_right, &QPushButton::clicked, this, &MainWindow::rotateElbowClockwise);
    connect(ui->mani3_stop, &QPushButton::clicked, this, &MainWindow::stopElbow);

    // 슬라이더 연결 (수동 회전)
    connect(ui->bass, &QSlider::valueChanged, this, [=](int val){ baseArm->setRotation(val); });
    connect(ui->mani_1, &QSlider::valueChanged, this, [=](int val){ shoulderArm->setRotation(val); });
    connect(ui->horizontalSlider_4, &QSlider::valueChanged, this, [=](int val){ elbowArm->setRotation(val); });

    // 저장/로드 버튼
    connect(ui->save, &QPushButton::clicked, this, &MainWindow::saveState);
    connect(ui->load, &QPushButton::clicked, this, &MainWindow::loadState);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// 수동 + 자동 회전 상태 갱신
void MainWindow::updateRotation()
{
    // Base
    if(baseDir != 0) {
        int newAngle = int(baseArm->rotation()) + baseDir;
        if(newAngle >= 360) newAngle -= 360;
        if(newAngle < 0) newAngle += 360;
        baseArm->setRotation(newAngle);
        ui->bass->setValue(newAngle);  // ui 슬라이더
    }

    // Shoulder
    if(shoulderDir != 0) {
        int newAngle = int(shoulderArm->rotation()) + shoulderDir;
        if(newAngle >= 360) newAngle -= 360;
        if(newAngle < 0) newAngle += 360;
        shoulderArm->setRotation(newAngle);
        ui->mani_1->setValue(newAngle);
    }

    // Elbow
    if(elbowDir != 0) {
        int newAngle = int(elbowArm->rotation()) + elbowDir;
        if(newAngle >= 360) newAngle -= 360;
        if(newAngle < 0) newAngle += 360;
        elbowArm->setRotation(newAngle);
        ui->horizontalSlider_4->setValue(newAngle);
    }
}


// Base Arm
void MainWindow::rotateBaseClockwise(){ baseDir = 1; }
void MainWindow::rotateBaseCounterClockwise(){ baseDir = -1; }
void MainWindow::stopBase(){ baseDir = 0; }

// Shoulder Arm
void MainWindow::rotateShoulderClockwise(){ shoulderDir = 1; }
void MainWindow::rotateShoulderCounterClockwise(){ shoulderDir = -1; }
void MainWindow::stopShoulder(){ shoulderDir = 0; }

// Elbow Arm
void MainWindow::rotateElbowClockwise(){ elbowDir = 1; }
void MainWindow::rotateElbowCounterClockwise(){ elbowDir = -1; }
void MainWindow::stopElbow(){ elbowDir = 0; }

// 저장
void MainWindow::saveState()
{
    QFile file("robot_state.txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        QMessageBox::warning(this,"Error","Cannot open file to save");
        return;
    }
    QTextStream out(&file);
    out << baseArm->rotation() << " " << baseDir << "\n";
    out << shoulderArm->rotation() << " " << shoulderDir << "\n";
    out << elbowArm->rotation() << " " << elbowDir << "\n";
    file.close();
}

// 불러오기
void MainWindow::loadState()
{
    QFile file("robot_state.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        QMessageBox::warning(this,"Error","Cannot open file to load");
        return;
    }
    QTextStream in(&file);
    double angle;
    int dir;
    in >> angle >> dir; baseArm->setRotation(angle); baseDir = dir; ui->bass->setValue(angle);
    in >> angle >> dir; shoulderArm->setRotation(angle); shoulderDir = dir; ui->mani_1->setValue(angle);
    in >> angle >> dir; elbowArm->setRotation(angle); elbowDir = dir; ui->horizontalSlider_4->setValue(angle);
    file.close();
}
