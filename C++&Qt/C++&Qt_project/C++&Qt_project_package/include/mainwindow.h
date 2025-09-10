#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QUdpSocket>
#include <QHostAddress>
#include <QHash>
#include <QRectF>
#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QWidget;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsRectItem;
class QTimer;
class Weapon;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;
    void resizeEvent(QResizeEvent* e) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void showEvent(QShowEvent* e) override;

private slots:
    void updateMovement();
    void updateBullets();
    void udpDataReceived();
    void sendChat();
    void timedSpawnTick(); // 5초마다(호스트만) 무기 스폰

private:
    // UI/씬
    void updateWeaponUi();
    void updateHpUi();

    // 맵
    void buildRandomMap(quint32 seed, qreal W, qreal H);
    void setupMiniMap();
    bool collidesBlockingPlayer(const QGraphicsEllipseItem* who) const;
    bool bulletHitObstacle(QGraphicsEllipseItem* bulletItem) const;
    void updateHideOpacity();
    void updateMiniMapView();
    void updateMiniViewRect();

    // 총알
    void spawnBullet(const QString& kindOrName, const QPointF& start, const QPointF& target, bool fromPeer);

    // 채팅
    void appendChatMessage(const QString& sender, const QString& text);

    // 드랍 동기화
    struct WeaponPickup {
        QGraphicsEllipseItem* item;
        enum class Kind { MULTI, 저격총, SHOTGUN } kind;
        quint64 id;
    };

    void spawnPickupLocal(quint64 id, WeaponPickup::Kind kind, const QPointF& pos);
    void removePickupById(quint64 id);
    static QString kindKeyFromPickupKind(WeaponPickup::Kind k);
    static WeaponPickup::Kind pickupKindFromKey(const QString& key);

    // ID 생성
    quint64 nextLocalDropId();

    // 네트워킹
    QUdpSocket* udp = nullptr;
    QHostAddress peerAddr;
    quint16 localPort = 0;
    quint16 peerPort  = 0;
    bool isHost       = false;

    // UI / 씬
    Ui::MainWindow *ui = nullptr;
    QGraphicsScene* scene = nullptr;

    // Player (나)
    QPointF playerPos;
    QGraphicsEllipseItem* playerItem = nullptr;
    qreal playerRadius = 12;

    // Peer (상대)
    QPointF peerPos;
    QGraphicsEllipseItem* peerItem = nullptr;
    qreal peerRadius = 12;

    // Aim / 이동
    QPointF mousePos;
    QGraphicsLineItem* aimLineItem = nullptr;
    bool aimVisible = false;
    bool movingUp=false, movingDown=false, movingLeft=false, movingRight=false;
    QTimer* moveTimer = nullptr;

    // 랜덤 맵 요소
    bool mapReady = false;
    quint32 mapSeed = 0;
    qreal mapW = 0, mapH = 0;
    std::vector<QGraphicsRectItem*> walls;
    std::vector<QGraphicsEllipseItem*> waters;
    std::vector<QGraphicsEllipseItem*> bushes;
    QPointF findSafeSpawn(qreal radius, int tries = 300) const;
    void    teleportPlayerTo(const QPointF& p, bool broadcast = true);

    // 무기
    QStringList weaponNames;
    Weapon* currentWeaponPtr = nullptr;
    std::unique_ptr<Weapon> basicWeapon;
    std::unique_ptr<Weapon> itemWeapon;

    // 드랍된 무기들
    std::vector<WeaponPickup*> droppedWeapons;
    QHash<quint64, WeaponPickup*> pickupById;
    quint32 localDropCounter = 1;
    QTimer* spawnTimer = nullptr; // 5초 타이머(호스트만 동작)

    // 총알 & HP
    struct Bullet {
        QGraphicsEllipseItem* item;
        QPointF vel;
        qreal remaining;
        int damage;
        bool fromPeer;
    };
    std::vector<Bullet*> bullets;
    QTimer* bulletTimer = nullptr;

    int myHP  = 100;
    int peerHP = 100;

    // 무적
    qint64 myInvulnUntilMs = 0;
    QTimer* invulnBlinkTimer = nullptr;
    bool invulnBlinkOn = false;
    QPen playerDefaultPen; // 기본 펜 저장(생성 시 채워줌)

    void startMyInvulnerability(int ms = 2000);
    void startNewGame();
    void checkGameOver();

    // 게임 상태
    bool gameStarted       = false;
    bool inGameOver        = false;
    bool restartReadyLocal = false;
    bool restartReadyPeer  = false;
    bool peerActive        = false;

    void appendWinnerMessage(const QString& winnerKey);
};
