#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>

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
    void enqueue();       // 문서 입력
    void dequeue();       // 문서 출력
    void updateDisplay(); // 대기열 표시 갱신

private:
    Ui::MainWindow *ui;
    std::vector<QString> queue; // Queue 자료구조
};
#endif // MAINWINDOW_H
