#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QVector>
#include <QTimer>
#include <QElapsedTimer>
#include <QPoint>
#include <QSet>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 공유 맵
enum class CellType : unsigned char {
    Empty = 0,
    Block = 1,
    Start = 2,
    End   = 3
};

struct GridModel {
    int rows = 0;
    int cols = 0;
    QVector<CellType> cells;
    QPoint start{-1, -1};
    QPoint goal {-1, -1};

    void init(int r, int c) {
        rows = r; cols = c;
        cells.fill(CellType::Empty, rows*cols);
        start = QPoint(-1,-1);
        goal  = QPoint(-1,-1);
    }
    inline bool inBounds(int r, int c) const { return r>=0 && c>=0 && r<rows && c<cols; }
    inline int idx(int r, int c) const { return r*cols + c; }
    inline CellType at(int r, int c) const { return cells[idx(r,c)]; }
    inline void set(int r, int c, CellType t){ cells[idx(r,c)] = t; }
};

// 경로 출력
struct SearchResult {
    bool found = false;
    QVector<int> visited_order;
    QVector<int> path;
    qint64 elapsed_ms = 0;
};

// 알고리즘
class Pathfinder {
public:
    virtual ~Pathfinder() = default;
    virtual SearchResult solve(const GridModel& grid) = 0;
protected:
    static inline int manhattan(int r1, int c1, int r2, int c2){
        return qAbs(r1-r2) + qAbs(c1-c2);
    }
};

class AStarPathfinder : public Pathfinder {
public:
    SearchResult solve(const GridModel& grid) override;
};

class DijkstraPathfinder : public Pathfinder {
public:
    SearchResult solve(const GridModel& grid) override;
};

// MainWindow
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_creat_map_clicked();
    void on_set_start_point_clicked();
    void on_set_end_point_clicked();
    void on_creat_random_block_clicked();
    void on_set_random_block_clicked();
    void on_start_clicked();
    void on_reset_clicked();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    enum class EditMode { None, SetStart, SetEnd, ToggleObstacle } m_mode = EditMode::None;

    Ui::MainWindow *ui;
    QGraphicsScene* sceneA = nullptr;
    QGraphicsScene* sceneD = nullptr;

    GridModel grid;
    QVector<int> visitedA, visitedD, pathA, pathD;
    int stepA = 0, stepD = 0;
    QTimer timerA, timerD;

    QVector<int> visitedOrder;
    QVector<int> finalPath;
    int stepIndex = 0;
    QTimer *animTimer = nullptr;

    qreal cellSizeA = 10.0, cellSizeD = 10.0;
    qreal margin = 5.0;

    const int MIN_ROWS = 5,  MAX_ROWS = 100;
    const int MIN_COLS = 5,  MAX_COLS = 100;
    const int MIN_PCT  = 0,  MAX_PCT  = 80;

    void resetScenes();
    void redrawScenes(bool clearExtraLayers = true);
    void drawScene(QGraphicsScene* s, qreal cellSize, const QVector<int>& visited, const QVector<int>& path);
    QPoint cellFromPos(QGraphicsView* view, qreal cellSize, const QPointF& scenePos) const;
    void setStartAt(int r, int c);
    void setEndAt(int r, int c);
    void toggleObstacleAt(int r, int c);
    void setStatus(const QString& msg);
    void setupSpinRanges();
    void animateResults(const SearchResult& resA, const SearchResult& resD);
};
