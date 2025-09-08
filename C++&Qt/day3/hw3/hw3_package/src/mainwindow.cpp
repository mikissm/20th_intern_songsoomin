#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMouseEvent>
#include <QRandomGenerator>
#include <QGraphicsRectItem>
#include <QScrollBar>
#include <cmath>
#include <algorithm>

// A*
#include <queue>
#include <limits>

struct PQNode {
    int idx;
    int f, g;
    bool operator<(const PQNode& other) const { return f > other.f; }
};

SearchResult AStarPathfinder::solve(const GridModel& grid) {
    SearchResult out;
    QElapsedTimer t; t.start();

    if (grid.start.x()<0 || grid.goal.x()<0) { out.found = false; out.elapsed_ms = t.elapsed(); return out; }

    const int R = grid.rows, C = grid.cols;
    const int startIdx = grid.idx(grid.start.y(), grid.start.x());
    const int goalIdx  = grid.idx(grid.goal.y(),  grid.goal.x());

    QVector<int> g(R*C, std::numeric_limits<int>::max());
    QVector<int> parent(R*C, -1);
    QVector<bool> closed(R*C, false);
    std::priority_queue<PQNode> open;

    g[startIdx] = 0;
    open.push({startIdx, manhattan(grid.start.y(), grid.start.x(), grid.goal.y(), grid.goal.x()), 0});

    auto neighbors = [&](int r, int c){
        static const int dr[4] = {-1,1,0,0};
        static const int dc[4] = {0,0,-1,1};
        QVector<QPoint> nb;
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if (grid.inBounds(nr,nc) && grid.at(nr,nc)!=CellType::Block) nb.append(QPoint(nc,nr));
        }
        return nb;
    };

    while(!open.empty()){
        auto cur = open.top(); open.pop();
        if (closed[cur.idx]) continue;
        closed[cur.idx] = true;
        out.visited_order.append(cur.idx);

        if (cur.idx == goalIdx) {
            out.found = true;
            break;
        }

        int r = cur.idx / C, c = cur.idx % C;
        for (auto p : neighbors(r,c)) {
            int ni = grid.idx(p.y(), p.x());
            if (closed[ni]) continue;
            int tentative = g[cur.idx] + 1;
            if (tentative < g[ni]) {
                g[ni] = tentative;
                parent[ni] = cur.idx;
                int h = manhattan(p.y(), p.x(), grid.goal.y(), grid.goal.x());
                open.push({ni, tentative + h, tentative});
            }
        }
    }

    if (out.found) {
        int cur = goalIdx;
        QVector<int> rev;
        while (cur != -1) { rev.append(cur); cur = parent[cur]; }
        std::reverse(rev.begin(), rev.end());
        out.path = rev;
    }
    out.elapsed_ms = t.elapsed();
    return out;
}

// Dijkstra
struct DJNode {
    int idx;
    int dist;
    bool operator<(const DJNode& other) const { return dist > other.dist; }
};

SearchResult DijkstraPathfinder::solve(const GridModel& grid) {
    SearchResult out;
    QElapsedTimer t; t.start();

    if (grid.start.x()<0 || grid.goal.x()<0) { out.found = false; out.elapsed_ms = t.elapsed(); return out; }

    const int R = grid.rows, C = grid.cols;
    const int startIdx = grid.idx(grid.start.y(), grid.start.x());
    const int goalIdx  = grid.idx(grid.goal.y(),  grid.goal.x());

    QVector<int> dist(R*C, std::numeric_limits<int>::max());
    QVector<int> parent(R*C, -1);
    QVector<bool> visited(R*C, false);
    std::priority_queue<DJNode> pq;

    dist[startIdx] = 0;
    pq.push({startIdx, 0});

    auto neighbors = [&](int r, int c){
        static const int dr[4] = {-1,1,0,0};
        static const int dc[4] = {0,0,-1,1};
        QVector<QPoint> nb;
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if (grid.inBounds(nr,nc) && grid.at(nr,nc)!=CellType::Block) nb.append(QPoint(nc,nr));
        }
        return nb;
    };

    while(!pq.empty()){
        auto cur = pq.top(); pq.pop();
        if (visited[cur.idx]) continue;
        visited[cur.idx] = true;
        out.visited_order.append(cur.idx);

        if (cur.idx == goalIdx) { out.found = true; break; }

        int r = cur.idx / C, c = cur.idx % C;
        for (auto p : neighbors(r,c)) {
            int ni = grid.idx(p.y(), p.x());
            if (visited[ni]) continue;
            int nd = dist[cur.idx] + 1;
            if (nd < dist[ni]) {
                dist[ni] = nd;
                parent[ni] = cur.idx;
                pq.push({ni, nd});
            }
        }
    }

    if (out.found) {
        int cur = goalIdx;
        QVector<int> rev;
        while (cur != -1) { rev.append(cur); cur = parent[cur]; }
        std::reverse(rev.begin(), rev.end());
        out.path = rev;
    }
    out.elapsed_ms = t.elapsed();
    return out;
}

// ---------------- MainWindow ----------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    sceneA = new QGraphicsScene(this);
    sceneD = new QGraphicsScene(this);
    ui->A_star->setScene(sceneA);
    ui->dijkstra->setScene(sceneD);

    ui->A_star->viewport()->installEventFilter(this);
    ui->dijkstra->viewport()->installEventFilter(this);

    setupSpinRanges();

    connect(&timerA, &QTimer::timeout, this, [this](){
        if (stepA < visitedA.size()) {
            stepA++;
            redrawScenes(false);
        } else if (stepA == visitedA.size()) {
            stepA++;
            redrawScenes(false);
        } else {
            timerA.stop();
        }
    });
    connect(&timerD, &QTimer::timeout, this, [this](){
        if (stepD < visitedD.size()) {
            stepD++;
            redrawScenes(false);
        } else if (stepD == visitedD.size()) {
            stepD++;
            redrawScenes(false);
        } else {
            timerD.stop();
        }
    });

    ui->start->setStyleSheet("background-color: green;");
    ui->reset->setStyleSheet("background-color: red;");

    setStatus("Ready. ROW/COL 설정 후 creat_map을 누르세요.");
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupSpinRanges(){
    ui->row->setRange(MIN_ROWS, MAX_ROWS);
    ui->spinBox_2->setRange(MIN_COLS, MAX_COLS);
    ui->how_many_block->setRange(MIN_PCT, MAX_PCT);
    ui->how_many_block->setSuffix("%");
}

void MainWindow::resetScenes(){
    sceneA->clear();
    sceneD->clear();
    stepA = stepD = 0;
    visitedA.clear(); visitedD.clear();
    pathA.clear(); pathD.clear();
}

void MainWindow::redrawScenes(bool clearExtraLayers){
    sceneA->setSceneRect(0,0, ui->A_star->viewport()->width(), ui->A_star->viewport()->height());
    sceneD->setSceneRect(0,0, ui->dijkstra->viewport()->width(), ui->dijkstra->viewport()->height());

    if (grid.rows>0 && grid.cols>0){
        cellSizeA = std::floor(std::min(
            (sceneA->width()-2*margin)/qreal(grid.cols),
            (sceneA->height()-2*margin)/qreal(grid.rows)
            ));
        cellSizeD = std::floor(std::min(
            (sceneD->width()-2*margin)/qreal(grid.cols),
            (sceneD->height()-2*margin)/qreal(grid.rows)
            ));

        if (cellSizeA < 3) cellSizeA = 3;
        if (cellSizeD < 3) cellSizeD = 3;
    }

    if (clearExtraLayers) { sceneA->clear(); sceneD->clear(); }

    QVector<int> curPathA = (stepA > visitedA.size()) ? pathA : QVector<int>{};
    QVector<int> curPathD = (stepD > visitedD.size()) ? pathD : QVector<int>{};

    drawScene(sceneA, cellSizeA, visitedA.mid(0, stepA), curPathA);
    drawScene(sceneD, cellSizeD, visitedD.mid(0, stepD), curPathD);
}


void MainWindow::drawScene(QGraphicsScene* s, qreal cellSize, const QVector<int>& visited, const QVector<int>& path){
    s->clear();

    const int R = grid.rows, C = grid.cols;
    for(int r=0;r<R;r++){
        for(int c=0;c<C;c++){
            QRectF rect(margin + c*cellSize, margin + r*cellSize, cellSize, cellSize);
            QBrush brush(Qt::white);
            CellType t = grid.at(r,c);
            if (t == CellType::Block) brush = QBrush(Qt::black);
            else if (t == CellType::Start) brush = QBrush(Qt::green);
            else if (t == CellType::End)   brush = QBrush(Qt::red);
            auto *item = s->addRect(rect, QPen(Qt::lightGray), brush);
            item->setZValue(0);
        }
    }
    for (int idx : visited){
        int r = idx / C, c = idx % C;
        if (grid.at(r,c)==CellType::Start || grid.at(r,c)==CellType::End) continue;
        QRectF rect(margin + c*cellSize, margin + r*cellSize, cellSize, cellSize);
        auto *item = s->addRect(rect.adjusted(1,1,-1,-1), QPen(Qt::NoPen), QBrush(Qt::magenta));
        item->setOpacity(0.6);
        item->setZValue(1);
    }
    if (!path.isEmpty()){
        for (int idx : path){
            int r = idx / C, c = idx % C;
            QRectF rect(margin + c*cellSize, margin + r*cellSize, cellSize, cellSize);
            auto *item = s->addRect(rect.adjusted(1,1,-1,-1), QPen(Qt::NoPen), QBrush(Qt::blue));
            item->setOpacity(0.8);
            item->setZValue(2);
        }
    }
}

QPoint MainWindow::cellFromPos(QGraphicsView* view, qreal cellSize, const QPointF& scenePos) const {
    qreal x = scenePos.x() - margin;
    qreal y = scenePos.y() - margin;
    int c = int(x / cellSize);
    int r = int(y / cellSize);
    if (!grid.inBounds(r,c)) return QPoint(-1,-1);
    return QPoint(c,r);
}

void MainWindow::setStartAt(int r, int c){
    if (!grid.inBounds(r,c)) return;
    if (grid.start.x() >= 0) grid.set(grid.start.y(), grid.start.x(), CellType::Empty);
    if (grid.at(r,c)==CellType::Block || grid.at(r,c)==CellType::End) return;
    grid.set(r,c, CellType::Start);
    grid.start = QPoint(c,r);
    setStatus(QString("Start: (%1,%2)").arg(r).arg(c));
    redrawScenes();
}

void MainWindow::setEndAt(int r, int c){
    if (!grid.inBounds(r,c)) return;
    if (grid.goal.x() >= 0) grid.set(grid.goal.y(), grid.goal.x(), CellType::Empty);
    if (grid.at(r,c)==CellType::Block || grid.at(r,c)==CellType::Start) return;
    grid.set(r,c, CellType::End);
    grid.goal = QPoint(c,r);
    setStatus(QString("End: (%1,%2)").arg(r).arg(c));
    redrawScenes();
}

void MainWindow::toggleObstacleAt(int r, int c){
    if (!grid.inBounds(r,c)) return;
    auto t = grid.at(r,c);
    if (t == CellType::Empty) {
        if (grid.start == QPoint(c,r) || grid.goal == QPoint(c,r)) return;
        grid.set(r,c, CellType::Block);
    } else if (t == CellType::Block) {
        grid.set(r,c, CellType::Empty);
    } else {
        return;
    }
    redrawScenes();
}

void MainWindow::setStatus(const QString& msg){
    ui->state_text->setText(msg);
}

// 연결
void MainWindow::on_creat_map_clicked(){
    int r = ui->row->value();
    int c = ui->spinBox_2->value();
    grid.init(r,c);
    resetScenes();
    redrawScenes();
    setStatus(QString("맵 생성: %1 x %2 격자").arg(r).arg(c));
}

void MainWindow::on_set_start_point_clicked(){
    m_mode = EditMode::SetStart;
    ui->set_start_point->setStyleSheet("background-color: blue;");
    ui->set_end_point->setStyleSheet("");
    ui->set_random_block->setStyleSheet("");
    setStatus("시작 지점을 클릭하여 설정하세요.");
}

void MainWindow::on_set_end_point_clicked(){
    m_mode = EditMode::SetEnd;
    ui->set_start_point->setStyleSheet("");
    ui->set_end_point->setStyleSheet("background-color: blue;");
    ui->set_random_block->setStyleSheet("");
    setStatus("끝 지점을 클릭하여 설정하세요.");
}

void MainWindow::on_set_random_block_clicked(){
    m_mode = EditMode::ToggleObstacle;
    ui->set_start_point->setStyleSheet("");
    ui->set_end_point->setStyleSheet("");
    ui->set_random_block->setStyleSheet("background-color: blue;");
    setStatus("빈 칸 클릭: 장애물 추가 / 장애물 클릭: 제거");
}

void MainWindow::on_creat_random_block_clicked(){
    if (grid.rows==0) { setStatus("먼저 creat_map으로 맵을 만드세요."); return; }
    int pct = ui->how_many_block->value();
    pct = qBound(MIN_PCT, pct, MAX_PCT);
    int total = grid.rows * grid.cols;
    int toPlace = (total * pct) / 100;

    for (int r=0;r<grid.rows;r++){
        for(int c=0;c<grid.cols;c++){
            if (grid.at(r,c)==CellType::Block) grid.set(r,c, CellType::Empty);
        }
    }

    int placed = 0;
    while (placed < toPlace) {
        int r = QRandomGenerator::global()->bounded(grid.rows);
        int c = QRandomGenerator::global()->bounded(grid.cols);
        if (grid.at(r,c)==CellType::Empty && QPoint(c,r)!=grid.start && QPoint(c,r)!=grid.goal){
            grid.set(r,c, CellType::Block);
            placed++;
        }
    }
    redrawScenes();
    setStatus(QString("무작위 장애물 배치: %1% (%2개)").arg(pct).arg(toPlace));
}

void MainWindow::on_start_clicked(){
    if (grid.rows==0) { setStatus("맵이 없습니다. creat_map을 먼저 누르세요."); return; }
    if (grid.start.x()<0 || grid.goal.x()<0) {
        setStatus("시작/끝 지점이 설정되지 않았습니다.");
        return;
    }

    AStarPathfinder astar;
    DijkstraPathfinder dijkstra;
    auto resA = astar.solve(grid);
    auto resD = dijkstra.solve(grid);

    visitedA = resA.visited_order;
    visitedD = resD.visited_order;
    pathA = resA.found ? resA.path : QVector<int>{};
    pathD = resD.found ? resD.path : QVector<int>{};
    stepA = stepD = 0;
    redrawScenes();

    QString msg;
    if (!resA.found && !resD.found) {
        msg = QString("길이 막혔습니다. (A*: %1ms, Dijkstra: %2ms)").arg(resA.elapsed_ms).arg(resD.elapsed_ms);
    } else {
        msg = QString("시작!  A*: %1ms (%2칸 방문), Dijkstra: %3ms (%4칸 방문)")
                  .arg(resA.elapsed_ms).arg(resA.visited_order.size())
                  .arg(resD.elapsed_ms).arg(resD.visited_order.size());
        if (resA.found) msg += QString("\nA* 경로 길이: %1").arg(resA.path.size());
        if (resD.found) msg += QString("\nDijkstra 경로 길이: %1").arg(resD.path.size());
    }
    setStatus(msg);

    timerA.start(10);
    timerD.start(10);
}

void MainWindow::on_reset_clicked(){
    if (grid.rows==0) { setStatus("초기화할 맵이 없습니다."); return; }
    int r = grid.rows, c = grid.cols;
    grid.init(r,c);
    resetScenes();
    redrawScenes();
    setStatus("리셋 완료. 맵이 비워졌습니다.");
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event){
    if (event->type() == QEvent::MouseButtonPress){
        if (grid.rows==0) return false;
        auto *view = (watched == ui->A_star->viewport()) ? ui->A_star :
                         (watched == ui->dijkstra->viewport()) ? ui->dijkstra : nullptr;
        if (!view) return false;
        auto *me = static_cast<QMouseEvent*>(event);
        QPointF scenePos = view->mapToScene(me->pos());
        qreal cs = (view == ui->A_star) ? cellSizeA : cellSizeD;
        QPoint cell = cellFromPos(view, cs, scenePos);
        if (cell.x()<0) return false;
        int r = cell.y(), c = cell.x();

        switch (m_mode){
        case EditMode::SetStart: setStartAt(r,c); break;
        case EditMode::SetEnd:   setEndAt(r,c);   break;
        case EditMode::ToggleObstacle: toggleObstacleAt(r,c); break;
        default: break;
        }
        return true;
    }
    return QMainWindow::eventFilter(watched, event);
}
