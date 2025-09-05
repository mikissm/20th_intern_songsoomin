#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->input_file_name->setPlaceholderText("여기에 파일 이름을 입력하세요");


    connect(ui->input_button, &QPushButton::clicked, this, &MainWindow::enqueue);
    connect(ui->out_button, &QPushButton::clicked, this, &MainWindow::dequeue);

    ui->name_list->setReadOnly(true);
    ui->output_name->setReadOnly(true);
    ui->name_list->setText("=== 현재 대기중인 문서 ===");
    ui->output_name->append("문서 출력 창");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::enqueue()
{
    QString doc = ui->input_file_name->text();
    if (!doc.isEmpty()) {
        queue.push_back(doc); // 큐 끝에 추가
        ui->input_file_name->clear();
        updateDisplay();
    }
}

void MainWindow::dequeue()
{
    if (!queue.empty()) {
        // 맨 앞 문서 가져오기
        QString printedDoc = queue.front();
        queue.erase(queue.begin()); // FIFO 제거

        // 출력창에 표시 (이전 출력 내용은 덮어씀)
        ui->output_name->clear();
        ui->output_name->append("출력된 문서: " + printedDoc);

        updateDisplay();
    } else {
        ui->output_name->setText("대기열이 비어 있습니다.");
    }
}

void MainWindow::updateDisplay()
{
    ui->name_list->clear();
    ui->name_list->append("=== 현재 대기중인 문서 ===");

    for (const auto &doc : queue) {
        ui->name_list->append(doc);
    }
}
