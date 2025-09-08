#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QTimer>
#include <QFile>
#include <QTextStream>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , shiftOn(false)
    , koreanMode(false)
    , symbolMode(false)
    , lastButton(nullptr)
    , lastPressTime(0)
    , multiTapIndex(0)
{
    ui->setupUi(this);

    // 버튼 텍스트 설정
    ui->button1->setText(".,?!");
    ui->button2->setText("ABC");
    ui->button3->setText("DEF");
    ui->button4->setText("GHI");
    ui->button5->setText("JKL");
    ui->button6->setText("MNO");
    ui->button7->setText("PQRS");
    ui->button8->setText("TUV");
    ui->button9->setText("WXYZ");
    ui->button0->setText("0");

    // 버튼 연결
    QList<QPushButton*> buttons = {
        ui->button1, ui->button2, ui->button3, ui->button4,
        ui->button5, ui->button6, ui->button7, ui->button8,
        ui->button9, ui->button0
    };

    for (QPushButton* btn : buttons) {
        connect(btn, &QPushButton::clicked, this, &MainWindow::handleButtonClicked);
    }

    // 특수 버튼
    connect(ui->back_space, &QPushButton::clicked, this, &MainWindow::handleBackspace);
    connect(ui->enter, &QPushButton::clicked, this, &MainWindow::handleEnter);
    connect(ui->space, &QPushButton::clicked, this, &MainWindow::handleSpace);
    connect(ui->reset, &QPushButton::clicked, this, &MainWindow::handleClearFile);
    connect(ui->shift, &QPushButton::clicked, this, &MainWindow::handleShift);
    connect(ui->change_languge, &QPushButton::clicked, this, &MainWindow::handleChangeLanguage);
    connect(ui->boutton_symbol, &QPushButton::clicked, this, &MainWindow::handleSymbol);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ------------------ 버튼 동작 ------------------

// 버튼 클릭 (멀티탭 방식)
void MainWindow::handleButtonClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    QString chars = btn->text();
    qint64 now = QDateTime::currentMSecsSinceEpoch();

    if (lastButton == btn && now - lastPressTime < 1000) {
        multiTapIndex = (multiTapIndex + 1) % chars.length();

        QString content = ui->textEdit->toPlainText();
        if (!content.isEmpty()) {
            content.chop(1);  // 마지막 문자 지우기
        }

        QChar ch = chars.at(multiTapIndex);
        if (shiftOn) {
            ch = ch.toUpper();
        } else {
            ch = ch.toLower();
        }

        content.append(ch);
        ui->textEdit->setPlainText(content);
        ui->textEdit->moveCursor(QTextCursor::End);
    } else {
        // 새 입력
        multiTapIndex = 0;
        QChar ch = chars.at(multiTapIndex);
        if (shiftOn) {
            ch = ch.toUpper();
        } else {
            ch = ch.toLower();
        }

        ui->textEdit->insertPlainText(QString(ch));
    }

    lastButton = btn;
    lastPressTime = now;
}

// 백스페이스
void MainWindow::handleBackspace()
{
    QString content = ui->textEdit->toPlainText();
    if (!content.isEmpty()) {
        content.chop(1);
        ui->textEdit->setPlainText(content);
        ui->textEdit->moveCursor(QTextCursor::End);
    }
}

// 엔터
void MainWindow::handleEnter()
{
    QString content = ui->textEdit->toPlainText().trimmed();

    if (!content.isEmpty()) {
        QFile file("log.txt"); 
        if (file.open(QIODevice::Append | QIODevice::Text)) {
            QTextStream out(&file);
            out << content << "\n";
            file.close();
            ui->statusbar->showMessage("추가되었습니다");
        } else {
            ui->statusbar->showMessage("파일 열기 실패");
        }
    }

    ui->textEdit->clear();  // 화면 비우기
}

// 스페이스
void MainWindow::handleSpace()
{
    ui->textEdit->insertPlainText(" ");
}

// 파일 초기화
void MainWindow::handleClearFile()
{
    QFile file("log.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.close();
        ui->statusbar->showMessage("파일이 초기화되었습니다");
    } else {
        ui->statusbar->showMessage("파일 초기화 실패");
    }
}


// Shift
void MainWindow::handleShift()
{
    shiftOn = !shiftOn;

    if (shiftOn) {
        ui->shift->setStyleSheet("background-color: lightblue;");
    } else {
        ui->shift->setStyleSheet("");
    }
}

// 한/영 전환 (기본은 영어)
void MainWindow::handleChangeLanguage()
{
    koreanMode = !koreanMode;
    if (koreanMode) {
        ui->button1->setText("ㅣ");
        ui->button2->setText("ㆍ");
        ui->button3->setText("ㅡ");
        ui->button4->setText("ㄱㅋㄲ");
        ui->button5->setText("ㄴㄹ");
        ui->button6->setText("ㄷㅌㄸ");
        ui->button7->setText("ㅂㅍㅃ");
        ui->button8->setText("ㅅㅎㅆ");
        ui->button9->setText("ㅈㅊㅉ");
        ui->button0->setText("ㅇㅁㅎ");
        ui->statusbar->showMessage("Korean Mode");
    } else {
        ui->button1->setText(".,?!");
        ui->button2->setText("ABC");
        ui->button3->setText("DEF");
        ui->button4->setText("GHI");
        ui->button5->setText("JKL");
        ui->button6->setText("MNO");
        ui->button7->setText("PQRS");
        ui->button8->setText("TUV");
        ui->button9->setText("WXYZ");
        ui->button0->setText("0");
        ui->statusbar->showMessage("English Mode");
    }
}

void MainWindow::handleSymbol()
{
    symbolMode = !symbolMode;
    if (symbolMode) {
        ui->button1->setText("1");
        ui->button2->setText("2");
        ui->button3->setText("3");
        ui->button4->setText("4");
        ui->button5->setText("5");
        ui->button6->setText("6");
        ui->button7->setText("7");
        ui->button8->setText("8");
        ui->button9->setText("9");
        ui->button0->setText("0");
        ui->statusbar->showMessage("Number Mode");
    } else {
        ui->button1->setText("!@#$%");
        ui->button2->setText("^&*()");
        ui->button3->setText("_+-=~");
        ui->button4->setText("`|\\/:");
        ui->button5->setText(";\"'<>");
        ui->button6->setText(",.?!");
        ui->button7->setText("…°·•");
        ui->button8->setText("‣․€£");
        ui->button9->setText("¥₩¢©");
        ui->button0->setText("®™℠¶");
        ui->statusbar->showMessage("Symbol Mode");
    }
}

