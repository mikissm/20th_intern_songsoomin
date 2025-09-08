#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QTextEdit>
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
    void handleButtonClicked();   // 숫자/문자 버튼 클릭 처리
    void handleBackspace();
    void handleEnter();
    void handleSpace();
    void handleClearFile();
    void handleShift();
    void handleSymbol();
    void handleChangeLanguage();

private:
    Ui::MainWindow *ui;
    bool shiftOn;
    bool koreanMode;
    bool symbolMode;

    QPushButton* lastButton;
    qint64 lastPressTime;
    int multiTapIndex;
};

#endif // MAINWINDOW_H
