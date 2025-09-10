#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "weapon.h"
#include "connectiondialog.h"

#include <QPen>
#include <QBrush>
#include <QTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QRandomGenerator>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QTime>
#include <QtMath>
#include <cmath>
#include <QCursor>
#include <QMessageBox>

#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>

// 유틸
static inline QPointF unitVec(const QPointF& v){
    const qreal len = std::hypot(v.x(), v.y());
    return (len > 0.0001) ? QPointF(v.x()/len, v.y()/len) : QPointF(1,0);
}

static QString kindFromName(const QString& wname){
    if (wname == "따발총") return "multi";
    if (wname == "저격총") return "sniper";
    if (wname == "샷건")   return "shotgun";
    if (wname == "기본총") return "basic";
    return "basic";
}

// kind → 탄속/데미지/사거리
static void paramsForKind(const QString& kind, qreal& speed, int& damage, qreal& range){
    if (kind == "sniper")    { speed=20.0; damage=40; range=2000.0; }
    else if (kind=="multi")  { speed=9.0;  damage=8;  range=900.0;  }
    else if (kind=="shotgun"){ speed=20.0; damage=7;  range=200.0;  }
    else /* 기본총 */         { speed=10.0; damage=10; range=1000.0; }
}

static QPointF rotateVec(const QPointF& v, qreal deg){
    const qreal r = qDegreesToRadians(deg);
    return QPointF(v.x()*std::cos(r) - v.y()*std::sin(r),
                   v.x()*std::sin(r) + v.y()*std::cos(r));
}

// 실수 난수
static inline qreal randRange(qreal a, qreal b) {
    return a + (b - a) * QRandomGenerator::global()->generateDouble();
}

// ------------------------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Battle Robit");

    // 네트워크 설정
    ConnectionDialog dlg(this);
    if(dlg.exec()!=QDialog::Accepted){
        close(); return;
    }
    localPort = dlg.localPort();
    peerPort  = dlg.peerPort();

    if (!dlg.peerIp().trimmed().isEmpty())
        peerAddr = QHostAddress(dlg.peerIp().trimmed());
    else
        peerAddr = QHostAddress();

    isHost = dlg.isHost();

    udp = new QUdpSocket(this);
    if(!udp->bind(QHostAddress::AnyIPv4, localPort)) {
        ui->statusbar->showMessage("UDP 바인드 실패: 포트 사용중일 수 있음");
    }
    connect(udp, &QUdpSocket::readyRead, this, &MainWindow::udpDataReceived);

    // 씬/UI
    scene = new QGraphicsScene(this);
    mapW = ui->mainView->width() * 3.0;
    mapH = ui->mainView->height() * 3.0;
    scene->setSceneRect(0,0,mapW,mapH);
    ui->mainView->setScene(scene);
    ui->mainView->setRenderHint(QPainter::Antialiasing);
    ui->mainView->viewport()->installEventFilter(this);
    ui->mainView->setMouseTracking(true);

    connect(scene, &QGraphicsScene::sceneRectChanged, this, [this](const QRectF&){
        updateMiniMapView();
    });

    // 미니맵
    ui->miniView->setScene(scene);
    ui->miniView->setRenderHint(QPainter::Antialiasing);
    ui->miniView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->miniView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->miniView->setInteractive(false);

    // 내 플레이어
    playerPos = QPointF(scene->width()/2, scene->height()/2);
    playerItem = scene->addEllipse(playerPos.x()-playerRadius, playerPos.y()-playerRadius,
                                   playerRadius*2, playerRadius*2,
                                   QPen(Qt::blue), QBrush(Qt::blue));
    playerDefaultPen = playerItem->pen();

    // 상대 플레이어
    peerPos = QPointF(playerPos.x()+60, playerPos.y());
    peerItem = scene->addEllipse(peerPos.x()-peerRadius, peerPos.y()-peerRadius,
                                 peerRadius*2, peerRadius*2,
                                 QPen(Qt::darkMagenta), QBrush(Qt::darkMagenta));
    peerItem->setVisible(false);

    // 조준선
    aimLineItem = scene->addLine(QLineF(), QPen(Qt::gray,2,Qt::DashLine));
    aimLineItem->setZValue(-1);
    aimLineItem->setVisible(aimVisible);

    // 이동 타이머
    moveTimer = new QTimer(this);
    connect(moveTimer, &QTimer::timeout, this, &MainWindow::updateMovement);
    moveTimer->start(16);

    // 무기 이름
    weaponNames = QStringList({"기본총","따발총","저격총","샷건"});
    ui->textBrowser->append("무기 목록:");
    for (auto& n: weaponNames) ui->textBrowser->append(" - " + n);

    // 기본총
    basicWeapon = std::make_unique<Weapon>(Weapon::BASIC,"기본총",10.0,30,1200,10,1200.0);
    currentWeaponPtr = basicWeapon.get();
    updateWeaponUi();

    // 총알 갱신 타이머
    bulletTimer = new QTimer(this);
    connect(bulletTimer, &QTimer::timeout, this, &MainWindow::updateBullets);
    bulletTimer->start(16);

    // 채팅 UI
    ui->pushButton->setText("보내기");
    ui->textBrowser_2->setReadOnly(true);
    ui->textBrowser_3->setReadOnly(false);
    ui->textBrowser_3->setAcceptRichText(false);
    ui->textBrowser_3->setPlaceholderText("메시지를 입력하고 Enter 또는 [보내기]를 누르세요");
    ui->textBrowser_3->installEventFilter(this);
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::sendChat);

    // 5초 스폰 타이머(호스트만 가동)
    spawnTimer = new QTimer(this);
    connect(spawnTimer, &QTimer::timeout, this, &MainWindow::timedSpawnTick);

    // 맵 시드 동기화
    if (isHost) {
        mapSeed = QRandomGenerator::global()->generate();
        buildRandomMap(mapSeed, mapW, mapH);
        setupMiniMap();
        {
            QPointF safe = findSafeSpawn(playerRadius);
            teleportPlayerTo(safe, /*broadcast=*/true);
            startMyInvulnerability(2000);
        }
        spawnTimer->start(5000);

        if (udp && !peerAddr.isNull() && peerPort) {
            QJsonObject o{{"type","map_seed"},{"seed",(double)mapSeed},{"w",mapW},{"h",mapH}};
            udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
        } else {
            ui->statusbar->showMessage("상대 주소 미설정: 첫 패킷 수신 시 자동 학습 후 맵 전송", 2000);
        }
    } else {
        appendChatMessage("시스템","맵 시드 수신 대기 중...");
    }

    if (udp && !peerAddr.isNull() && peerPort) {
        QJsonObject hello{{"type","hello"},{"msg","joined"},{"port", (int)localPort},{"host",isHost}};
        udp->writeDatagram(QJsonDocument(hello).toJson(QJsonDocument::Compact), peerAddr, peerPort);
    }

    updateHpUi();
    ui->mainView->centerOn(playerItem);

    updateMiniMapView();
}

MainWindow::~MainWindow(){ delete ui; }

// 맵 생성
void MainWindow::buildRandomMap(quint32 seed, qreal W, qreal H)
{
    for (auto* r: walls)   { scene->removeItem(r); delete r; }
    for (auto* e: waters)  { scene->removeItem(e); delete e; }
    for (auto* e: bushes)  { scene->removeItem(e); delete e; }
    walls.clear(); waters.clear(); bushes.clear();

    scene->setSceneRect(0,0,W,H);

    QRandomGenerator gen(seed);

    // 외곽 벽
    const qreal t=24.0;
    auto addWall = [&](qreal x,qreal y,qreal w,qreal h){
        auto* r = scene->addRect(x,y,w,h,QPen(Qt::NoPen),QBrush(QColor("Black")));
        r->setZValue(5);
        walls.push_back(r);
    };
    addWall(0,0,W,t);
    addWall(0,H-t,W,t);
    addWall(0,t,t,H-2*t);
    addWall(W-t,t,t,H-2*t);

    // 랜덤 벽
    int wallN = int(gen.bounded(30, 40));
    for(int i=0;i<wallN;i++){
        qreal ww = gen.bounded(140, 360);
        qreal hh = gen.bounded(20,  60);
        if (gen.bounded(0,2)) std::swap(ww,hh);
        qreal x = gen.bounded(int(t+20), int(W-t-ww-20));
        qreal y = gen.bounded(int(t+20), int(H-t-hh-20));
        addWall(x,y,ww,hh);
    }

    // 물 웅덩이
    int waterN = int(gen.bounded(6,11));
    for(int i=0;i<waterN;i++){
        qreal r = gen.bounded(40,90);
        qreal x = gen.bounded(int(t+ r+10), int(W-t- r-10));
        qreal y = gen.bounded(int(t+ r+10), int(H-t- r-10));
        auto* e = scene->addEllipse(x-r,y-r,2*r,2*r, QPen(Qt::NoPen), QBrush(QColor(70,170,255,160)));
        e->setZValue(-1.5);
        waters.push_back(e);
    }

    // 덤불
    int bushN = int(gen.bounded(10,17));
    for(int i=0;i<bushN;i++){
        qreal r = gen.bounded(35,80);
        qreal x = gen.bounded(int(t+ r+10), int(W-t- r-10));
        qreal y = gen.bounded(int(t+ r+10), int(H-t- r-10));
        auto* e = scene->addEllipse(x-r,y-r,2*r,2*r, QPen(Qt::NoPen), QBrush(QColor(40,180,70,140)));
        e->setZValue(-1.2);
        bushes.push_back(e);
    }

    mapReady = true;

    updateMiniMapView();
}

void MainWindow::setupMiniMap()
{
    ui->miniView->setScene(scene);
    ui->miniView->setRenderHint(QPainter::Antialiasing);
    ui->miniView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->miniView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->miniView->setInteractive(false);

    ui->miniView->fitInView(scene->sceneRect().adjusted(-50,-50,50,50), Qt::KeepAspectRatio);
}

bool MainWindow::collidesBlockingPlayer(const QGraphicsEllipseItem* who) const
{
    for (auto* r: walls)  if (who->collidesWithItem(r)) return true;
    for (auto* e: waters) if (who->collidesWithItem(e)) return true;
    return false;
}

bool MainWindow::bulletHitObstacle(QGraphicsEllipseItem* bulletItem) const
{
    for (auto* r: walls)  if (bulletItem->collidesWithItem(r)) return true;
    return false;
}

void MainWindow::updateHideOpacity()
{
    bool inBush = false;
    for (auto* e: bushes) if (playerItem->collidesWithItem(e)) { inBush=true; break; }
    playerItem->setOpacity(inBush ? 0.5 : 1.0);

    bool peerInBush = false;
    for (auto* e: bushes) if (peerItem->collidesWithItem(e)) { peerInBush=true; break; }
    peerItem->setOpacity(peerInBush ? 0.0 : 1.0);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    // 채팅 입력창: Enter로 전송
    if (obj == ui->textBrowser_3 && event->type() == QEvent::KeyPress) {
        auto* ke = static_cast<QKeyEvent*>(event);
        if ((ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter)
            && !(ke->modifiers() & Qt::ShiftModifier)) {
            sendChat();
            return true;
        }
        if (ke->key() == Qt::Key_Escape) {
            ui->mainView->setFocus(Qt::OtherFocusReason);
            return true;
        }
        return false;
    }

    // 마우스 처리
    if (obj == ui->mainView->viewport()) {
        if (event->type() == QEvent::MouseMove) {
            auto* me = static_cast<QMouseEvent*>(event);
            mousePos = ui->mainView->mapToScene(me->pos());
            if (aimVisible) {
                QLineF line(playerPos, mousePos);
                line.setLength(1000);
                aimLineItem->setLine(line);
            }
            return true;
        } else if (event->type() == QEvent::MouseButtonPress) {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::LeftButton) {
                aimVisible = !aimVisible;
                aimLineItem->setVisible(aimVisible);
                return true;
            } else if (me->button() == Qt::RightButton) {
                if (!currentWeaponPtr || !mapReady) return true;

                if (!currentWeaponPtr->fire(scene, playerPos, mousePos, this)) {
                    ui->textBrowser->setText("발사 실패: 탄 없음/장전중 (R)");
                } else {
                    updateWeaponUi();
                    const QString wname = currentWeaponPtr->name();
                    spawnBullet(wname, playerPos, mousePos, /*fromPeer=*/false);

                    if (udp) {
                        const QString kind = kindFromName(wname);
                        QJsonObject o{{"type","fire"},
                                      {"x", playerPos.x()},
                                      {"y", playerPos.y()},
                                      {"tx", mousePos.x()},
                                      {"ty", mousePos.y()},
                                      {"weapon", wname},
                                      {"kind", kind}};
                        udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                                           peerAddr, peerPort);
                    }
                }
                return true;
            }
        }
    }

    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->isAutoRepeat()) return;

    // T : 채팅 입력
    if (event->key() == Qt::Key_T) {
        if (QApplication::focusWidget() != ui->textBrowser_3) {
            ui->textBrowser_3->setFocus(Qt::ShortcutFocusReason);
            auto c = ui->textBrowser_3->textCursor();
            c.movePosition(QTextCursor::End);
            ui->textBrowser_3->setTextCursor(c);
            return;
        }
    }

    switch(event->key()) {
    case Qt::Key_W: movingUp=true; break;
    case Qt::Key_S: movingDown=true; break;
    case Qt::Key_A: movingLeft=true; break;
    case Qt::Key_D: movingRight=true; break;

    case Qt::Key_1:
        currentWeaponPtr = basicWeapon.get();
        updateWeaponUi();
        break;
    case Qt::Key_2:
        if(itemWeapon) { currentWeaponPtr=itemWeapon.get(); updateWeaponUi(); }
        else ui->textBrowser->setText("아이템 총 없음 (F로 습득)");
        break;
    case Qt::Key_R:
        if(currentWeaponPtr)
            currentWeaponPtr->reload(this,[this](const QString&s){ui->textBrowser->setText(s);});
        break;

    case Qt::Key_F: {
        if (!mapReady) break;

        WeaponPickup* hit = nullptr;
        for (auto* pk : droppedWeapons) {
            if (pk && pk->item && playerItem->collidesWithItem(pk->item)) {
                hit = pk;
                break;
            }
        }

        if (hit) {
            // 1. 기존 무기 버리기 (기본총 제외)
            if (currentWeaponPtr != basicWeapon.get() && itemWeapon) {
                auto k = itemWeapon->kind();
                WeaponPickup::Kind pk = WeaponPickup::Kind::MULTI;
                if (k == Weapon::저격총)      pk = WeaponPickup::Kind::저격총;
                else if (k == Weapon::SHOTGUN) pk = WeaponPickup::Kind::SHOTGUN;

                const quint64 id = nextLocalDropId();
                spawnPickupLocal(id, pk, playerPos);

                if (udp) {
                    QJsonObject o{
                        {"type", "spawn_weapon"},
                        {"id",   (double)id},
                        {"kind", kindKeyFromPickupKind(pk)},
                        {"x",    playerPos.x()},
                        {"y",    playerPos.y()}
                    };
                    udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                                       peerAddr, peerPort);
                }
                itemWeapon.reset();
            }

            // 2. 새 무기 장착
            if (hit->kind == WeaponPickup::Kind::MULTI)
                itemWeapon = std::make_unique<Weapon>(Weapon::MULTI, "따발총", 9.0, 20, 1400, 8, 900);
            else if (hit->kind == WeaponPickup::Kind::저격총)
                itemWeapon = std::make_unique<Weapon>(Weapon::저격총, "저격총", 20.0, 5, 2500, 40, 2000);
            else
                itemWeapon = std::make_unique<Weapon>(Weapon::SHOTGUN, "샷건", 20.0, 8, 1800, 7, 200);

            // 바로 발사 못 하게
            itemWeapon->setAmmo(0);

            itemWeapon->onPicked([this](const QString& s) {
                ui->textBrowser->setText(s);
            });
            currentWeaponPtr = itemWeapon.get();
            updateWeaponUi();

            removePickupById(hit->id);

            if (udp) {
                QJsonObject o{{"type", "pickup_weapon"}, {"id", (double)hit->id}};
                udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                                   peerAddr, peerPort);
            }

            ui->textBrowser->setText("새 무기를 획득했습니다. R키로 장전하세요!");
        }
        break;
    }


    case Qt::Key_Q: {
        if (!mapReady) break;
        if(currentWeaponPtr!=basicWeapon.get() && itemWeapon) {
            auto k = itemWeapon->kind();
            WeaponPickup::Kind pk = WeaponPickup::Kind::MULTI;
            if(k==Weapon::저격총) pk = WeaponPickup::Kind::저격총;
            else if(k==Weapon::SHOTGUN) pk = WeaponPickup::Kind::SHOTGUN;

            const quint64 id = nextLocalDropId();
            spawnPickupLocal(id, pk, playerPos);

            if(udp){
                QJsonObject o{{"type","spawn_weapon"},
                              {"id", (double)id},
                              {"kind", kindKeyFromPickupKind(pk)},
                              {"x", playerPos.x()},
                              {"y", playerPos.y()}};
                udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
            }

            itemWeapon.reset();
            currentWeaponPtr=basicWeapon.get();
            updateWeaponUi();
        }
        break;
    }
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    if(event->isAutoRepeat()) return;
    if(event->key()==Qt::Key_W) movingUp=false;
    else if(event->key()==Qt::Key_S) movingDown=false;
    else if(event->key()==Qt::Key_A) movingLeft=false;
    else if(event->key()==Qt::Key_D) movingRight=false;
}

void MainWindow::updateMovement() {
    if(!mapReady) {
        ui->mainView->centerOn(playerItem);
        updateMiniViewRect();
        return;
    }

    qreal dx=0,dy=0, speed=5;
    if(movingUp) dy-=speed;
    if(movingDown) dy+=speed;
    if(movingLeft) dx-=speed;
    if(movingRight) dx+=speed;

    if(dx||dy) {
        QPointF newPos = playerPos + QPointF(dx,dy);
        playerItem->setRect(newPos.x()-playerRadius,newPos.y()-playerRadius,playerRadius*2,playerRadius*2);

        if (collidesBlockingPlayer(playerItem)) {
            playerItem->setRect(playerPos.x()-playerRadius,playerPos.y()-playerRadius,playerRadius*2,playerRadius*2);
        } else {
            playerPos = newPos;

            QRectF bounds=scene->sceneRect();
            if(playerPos.x()-playerRadius<bounds.left()) playerPos.setX(bounds.left()+playerRadius);
            if(playerPos.x()+playerRadius>bounds.right()) playerPos.setX(bounds.right()-playerRadius);
            if(playerPos.y()-playerRadius<bounds.top()) playerPos.setY(bounds.top()+playerRadius);
            if(playerPos.y()+playerRadius>bounds.bottom()) playerPos.setY(bounds.bottom()-playerRadius);

            playerItem->setRect(playerPos.x()-playerRadius,playerPos.y()-playerRadius,playerRadius*2,playerRadius*2);

            if(udp){
                QJsonObject o{{"type","pos"},{"x",playerPos.x()},{"y",playerPos.y()}};
                udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
            }
        }
    }

    // 조준선 갱신
    const QPoint vp = ui->mainView->viewport()->mapFromGlobal(QCursor::pos());
    mousePos = ui->mainView->mapToScene(vp);

    if (aimVisible) {
        QLineF line(playerPos, mousePos);
        line.setLength(1000);
        aimLineItem->setLine(line);
    }

    updateHideOpacity();

    // 카메라: 플레이어를 따라다니기
    ui->mainView->centerOn(playerItem);

    // 미니맵 갱신
    updateMiniViewRect();
}

void MainWindow::updateWeaponUi() {
    if(!currentWeaponPtr){ ui->textBrowser->setText("무기 없음"); return; }
    ui->textBrowser->setText(
        QString("무기:%1 | 탄:%2/%3 | 데미지:%4 | 속도:%5 | 사거리:%6 | 내HP:%7 / 상대HP:%8")
            .arg(currentWeaponPtr->name())
            .arg(currentWeaponPtr->ammo()).arg(currentWeaponPtr->magSize())
            .arg(currentWeaponPtr->damage())
            .arg(currentWeaponPtr->bulletSpeed())
            .arg(currentWeaponPtr->rangePx())
            .arg(myHP).arg(peerHP)
        );
}

void MainWindow::updateHpUi() {
    ui->statusbar->showMessage(QString("내 HP: %1   |   상대 HP: %2").arg(myHP).arg(peerHP), 1500);
    updateWeaponUi();
}

// 총알 스폰
void MainWindow::spawnBullet(const QString& kindOrName, const QPointF& start, const QPointF& target, bool fromPeer)
{
    QString kind = kindOrName;
    if (kind != "multi" && kind != "sniper" && kind != "shotgun" && kind != "basic")
        kind = kindFromName(kindOrName);

    qreal speed; int damage; qreal range;
    paramsForKind(kind, speed, damage, range);

    auto radiusForKind = [&](const QString& k)->qreal {
        if (k == "sniper")  return 7.0;
        if (k == "multi")   return 3.0;
        if (k == "shotgun") return 2.5;
        return 3.0;
    };
    auto brushForKind = [&](const QString& k)->QBrush {
        if (k == "sniper")  return QBrush(Qt::darkRed);
        if (k == "multi")   return QBrush(Qt::darkRed);
        if (k == "shotgun") return QBrush(Qt::darkRed);
        return QBrush(Qt::darkRed);
    };

    const qreal r = radiusForKind(kind);
    const QBrush br = brushForKind(kind);

    QPointF dir0 = unitVec(target - start);

    auto spawnOne = [&](qreal jitterDeg){
        QPointF d   = (jitterDeg != 0.0 ? rotateVec(dir0, jitterDeg) : dir0);
        QPointF vel = d * speed;

        const qreal muzzleOffset = (fromPeer ? peerRadius : playerRadius) + r + 2.0;

        auto* item = scene->addEllipse(-r, -r, 2*r, 2*r, QPen(Qt::NoPen), br);
        item->setPos(start + d * muzzleOffset);
        item->setZValue(20);
        bullets.push_back(new Bullet{ item, vel, range, damage, fromPeer });
    };

    if (kind == "multi") {
        const qreal spreadDeg = 6.0;
        spawnOne(-spreadDeg);
        spawnOne(+spreadDeg);
    } else if (kind == "shotgun") {
        const int   count     = 6;
        const qreal spreadDeg = 12.0;
        for (int i=0; i<count; ++i)
            spawnOne(randRange(-spreadDeg, +spreadDeg));
    } else {
        spawnOne(0.0);
    }
}


void MainWindow::updateBullets()
{
    if (bullets.empty()) return;

    for (size_t i = 0; i < bullets.size(); ) {
        Bullet* b = bullets[i];

        const QPointF v = b->vel;
        const qreal step = std::hypot(v.x(), v.y());

        const qreal kMaxStep = 6.0;
        const int substeps = std::max(1, int(std::ceil(step / kMaxStep)));
        const QPointF delta(v.x() / substeps, v.y() / substeps);

        bool remove = false;
        for (int s = 0; s < substeps && !remove; ++s) {
            b->item->moveBy(delta.x(), delta.y());
            const qreal d = std::hypot(delta.x(), delta.y());
            b->remaining -= d;

            if (b->remaining <= 0) { remove = true; break; }

            if (bulletHitObstacle(b->item)) { remove = true; break; }

            QGraphicsEllipseItem* target = nullptr;
            if (b->fromPeer) {
                target = playerItem;   // 상대가 쏜 총알 → 내 캐릭터 충돌 체크
            } else if (peerActive) {
                target = peerItem;     // 내가 쏜 총알 → 상대가 활성화된 경우만 충돌 체크
            }

            if (target && target->collidesWithItem(b->item)) {
                if (inGameOver) {
                    remove = true;
                } else {
                    if (b->fromPeer) {
                        myHP -= b->damage; if (myHP < 0) myHP = 0;
                        updateHpUi();
                        checkGameOver();
                        if (udp) {
                            QJsonObject o{{"type","hp"},{"hp", myHP}};
                            udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
                        }
                    } else {
                        peerHP -= b->damage; if (peerHP < 0) peerHP = 0;
                        updateHpUi();
                        checkGameOver();

                        if (udp) {
                            QJsonObject o{{"type","hit"},{"dmg", b->damage}};
                            udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
                        }
                    }
                    remove = true;
                }
                break;
            }

        }

        if (remove) {
            scene->removeItem(b->item);
            delete b->item;
            delete b;
            bullets.erase(bullets.begin() + i);
        } else {
            ++i;
        }
    }
}

// 드랍 동기화
QString MainWindow::kindKeyFromPickupKind(WeaponPickup::Kind k){
    switch(k){
    case WeaponPickup::Kind::MULTI:   return "multi";
    case WeaponPickup::Kind::저격총:  return "sniper";
    case WeaponPickup::Kind::SHOTGUN: return "shotgun";
    }
    return "multi";
}

MainWindow::WeaponPickup::Kind MainWindow::pickupKindFromKey(const QString& key){
    if(key=="sniper")  return WeaponPickup::Kind::저격총;
    if(key=="shotgun") return WeaponPickup::Kind::SHOTGUN;
    return WeaponPickup::Kind::MULTI; // default multi
}

void MainWindow::spawnPickupLocal(quint64 id, WeaponPickup::Kind kind, const QPointF& pos)
{
    QColor color=Qt::red;
    if(kind==WeaponPickup::Kind::저격총) color=Qt::blue;
    else if(kind==WeaponPickup::Kind::SHOTGUN) color=QColor("#ff9500");

    auto* e=scene->addEllipse(-8,-8,16,16,QPen(),QBrush(color));
    e->setPos(pos);

    auto* pk = new WeaponPickup{e, kind, id};
    droppedWeapons.push_back(pk);
    pickupById.insert(id, pk);
}

void MainWindow::removePickupById(quint64 id)
{
    auto it = pickupById.find(id);
    if (it == pickupById.end()) return;
    WeaponPickup* pk = it.value();

    if (pk && pk->item) {
        scene->removeItem(pk->item);
        delete pk->item;
    }
    for (auto vit = droppedWeapons.begin(); vit != droppedWeapons.end(); ++vit) {
        if (*vit == pk) { droppedWeapons.erase(vit); break; }
    }
    delete pk;
    pickupById.remove(id);
}

quint64 MainWindow::nextLocalDropId()
{
    return (quint64(localPort) << 32) | quint64(localDropCounter++);
}

// 5초마다 총 스폰(호스트만)
void MainWindow::timedSpawnTick()
{
    if (!isHost || !mapReady) return;

    int t = QRandomGenerator::global()->bounded(1,4);
    WeaponPickup::Kind kind = WeaponPickup::Kind::MULTI;
    if (t==2) kind = WeaponPickup::Kind::저격총;
    else if (t==3) kind = WeaponPickup::Kind::SHOTGUN;

    QRectF R = scene->sceneRect().adjusted(40,40,-40,-40);
    QPointF pos;
    for (int tries=0; tries<100; ++tries) {
        QPointF p(
            QRandomGenerator::global()->bounded(int(R.left()),  int(R.right())),
            QRandomGenerator::global()->bounded(int(R.top()),   int(R.bottom()))
            );
        auto* ghost = new QGraphicsEllipseItem(-8,-8,16,16);
        ghost->setPos(p);
        bool bad=false;
        for (auto* r: walls)  if (ghost->collidesWithItem(r)) { bad=true; break; }
        for (auto* e: waters) if (ghost->collidesWithItem(e)) { bad=true; break; }
        delete ghost;
        if (!bad) { pos=p; break; }
    }
    if (pos.isNull()) pos = QPointF(R.center());

    const quint64 id = nextLocalDropId();

    spawnPickupLocal(id, kind, pos);

    if (udp && !peerAddr.isNull() && peerPort) {
        QJsonObject o{{"type","spawn_weapon"},
                      {"id",(double)id},
                      {"kind", kindKeyFromPickupKind(kind)},
                      {"x", pos.x()},
                      {"y", pos.y()}};
        udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
    } else {
        ui->statusbar->showMessage("상대 주소 미설정: 스폰 브로드캐스트 보류", 1000);
    }
}

// 채팅
void MainWindow::sendChat()
{
    if (!udp) return;
    const QString msg = ui->textBrowser_3->toPlainText().trimmed();
    if (msg.isEmpty()) return;

    appendChatMessage("나", msg);

    if (udp && !peerAddr.isNull() && peerPort) {
        QJsonObject o{{"type","chat"},{"msg", msg}};
        udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
    } else {
        ui->statusbar->showMessage("상대 주소 미설정: 채팅 전송 보류", 1000);
    }

    ui->textBrowser_3->clear();

    ui->mainView->setFocus(Qt::OtherFocusReason);
}

void MainWindow::appendChatMessage(const QString& sender, const QString& text)
{
    const QString ts = QTime::currentTime().toString("HH:mm:ss");
    ui->textBrowser_2->append(QString("[%1] %2: %3").arg(ts, sender, text));
}

void MainWindow::appendWinnerMessage(const QString& winnerKey)
{
    QString who = "무승부";
    if (winnerKey == "host") {
        who = isHost ? "나(호스트)" : "상대(호스트)";
    } else if (winnerKey == "peer") {
        who = isHost ? "상대(참가자)" : "나(참가자)";
    }

    appendChatMessage("시스템", QString("게임 종료 — 승자: %1").arg(who));
}


// HP 체크
void MainWindow::checkGameOver()
{
    // 호스트에서만 게임 오버 판단
    if (!isHost) return;

    // 이미 게임오버 처리 중이면 무시
    if (inGameOver) return;

    const bool myDead   = (myHP   <= 0);
    const bool peerDead = (peerActive && peerHP <= 0);

    if (!myDead && !peerDead) return; // 아직 안 죽음

    inGameOver = true; // 재진입 금지

    // 승자 키 결정
    QString winnerKey = "draw";
    if ( myDead && !peerDead) winnerKey = "peer";
    if (!myDead &&  peerDead) winnerKey = "host";

    // 채팅창 표기
    appendWinnerMessage(winnerKey);

    // 팝업
    QMessageBox::information(this, "게임 종료",
                             (winnerKey=="draw") ? "무승부! 재시작 준비" :
                                 (winnerKey=="host") ? "내 승리! 재시작 준비" : "상대 승리! 재시작 준비");

    if (udp && !peerAddr.isNull() && peerPort) {
        QJsonObject go{{"type","game_over"}, {"winner", winnerKey}};
        udp->writeDatagram(QJsonDocument(go).toJson(QJsonDocument::Compact),
                           peerAddr, peerPort);
    }

    restartReadyLocal = false;
    restartReadyPeer  = false;

    if (udp && !peerAddr.isNull() && peerPort) {
        QJsonObject o{{"type","restart_request"}};
        udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                           peerAddr, peerPort);
    }
}


// 새 게임
void MainWindow::startNewGame()
{
    // 초기화
    inGameOver        = false;
    restartReadyLocal = false;
    restartReadyPeer  = false;
    gameStarted       = true;
    peerActive        = false;

    // 입력 초기화
    movingUp = movingDown = movingLeft = movingRight = false;
    aimVisible = false;
    if (aimLineItem) aimLineItem->setVisible(false);
    if (ui->textBrowser_3->hasFocus()) ui->textBrowser_3->clearFocus();
    ui->mainView->setFocus(Qt::OtherFocusReason);

    // HP 초기화
    myHP   = 100;
    peerHP = 100;
    updateHpUi();

    // 총알 정리 ---
    for (auto* b : bullets) {
        if (b && b->item) { scene->removeItem(b->item); delete b->item; }
        delete b;
    }
    bullets.clear();

    // 무기 리셋
    itemWeapon.reset();
    currentWeaponPtr = basicWeapon.get();
    if (currentWeaponPtr) currentWeaponPtr->setAmmo(currentWeaponPtr->magSize());
    updateWeaponUi();

    if (isHost) {
        // 1) 새 맵 먼저 생성/배포
        mapSeed = QRandomGenerator::global()->generate();
        buildRandomMap(mapSeed, mapW, mapH);
        setupMiniMap();

        // 2) "새 맵" 기준으로 안전스폰 계산 후 텔레포트
        QPointF safe = findSafeSpawn(playerRadius);
        teleportPlayerTo(safe, /*broadcast=*/true);
        startMyInvulnerability(2000);

        // 3) 맵시드/HP 동기화 전송
        if (udp && !peerAddr.isNull() && peerPort) {
            QJsonObject seed{{"type","map_seed"},{"seed",(double)mapSeed},{"w",mapW},{"h",mapH}};
            udp->writeDatagram(QJsonDocument(seed).toJson(QJsonDocument::Compact), peerAddr, peerPort);

            QJsonObject sync{
                {"type","hp_sync"},
                {"host", true},
                {"my",   myHP},
                {"peer", peerHP}
            };
            udp->writeDatagram(QJsonDocument(sync).toJson(QJsonDocument::Compact), peerAddr, peerPort);
        }

    } else {
        mapReady = false;
        peerItem->setVisible(false);
        appendChatMessage("시스템","맵 시드 수신 대기 중...");
    }

    QMessageBox::information(this, "게임 시작", "새 게임이 시작되었습니다!");
}


// UDP 수신
void MainWindow::udpDataReceived() {
    while (udp && udp->hasPendingDatagrams()) {
        QByteArray buf; buf.resize(int(udp->pendingDatagramSize()));
        QHostAddress from; quint16 port;
        udp->readDatagram(buf.data(), buf.size(), &from, &port);

        if (peerAddr.isNull() || peerPort == 0) {
            peerAddr = from;
            peerPort = port;
            ui->statusbar->showMessage(QString("상대 주소 학습: %1:%2")
                                           .arg(peerAddr.toString()).arg(peerPort), 1500);
            if (isHost && mapReady && udp) {
                QJsonObject o{{"type","map_seed"},{"seed",(double)mapSeed},{"w",mapW},{"h",mapH}};
                udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
            }
        }

        const auto doc = QJsonDocument::fromJson(buf);
        if (!doc.isObject()) continue;
        const auto o = doc.object();
        const auto type = o.value("type").toString();

        if (type=="map_seed") {
            mapSeed = (quint32)o.value("seed").toDouble();
            mapW = o.value("w").toDouble(mapW);
            mapH = o.value("h").toDouble(mapH);
            buildRandomMap(mapSeed, mapW, mapH);
            setupMiniMap();
            appendChatMessage("시스템","맵 동기화 완료");

            peerItem->setVisible(false);

            QPointF safe = findSafeSpawn(playerRadius);
            teleportPlayerTo(safe, /*broadcast=*/true);
            startMyInvulnerability(2000);

        } else if (type=="pos") {
            peerPos.setX(o.value("x").toDouble(peerPos.x()));
            peerPos.setY(o.value("y").toDouble(peerPos.y()));
            peerItem->setRect(peerPos.x()-peerRadius, peerPos.y()-peerRadius, peerRadius*2, peerRadius*2);

            peerItem->setVisible(true);
            peerActive = true;

        } else if (type=="fire") {
            const QPointF start(o.value("x").toDouble(), o.value("y").toDouble());
            const QPointF target(o.value("tx").toDouble(), o.value("ty").toDouble());
            const QString kind = o.value("kind").toString();
            const QString name = o.value("weapon").toString();
            if (mapReady) spawnBullet(!kind.isEmpty()? kind : name, start, target, /*fromPeer=*/true);
            ui->statusbar->showMessage("상대 발사", 300);

        } else if (type=="hit") {
            const int dmg = o.value("dmg").toInt(0);
            if (dmg > 0 && !inGameOver) {
                const qint64 now = QDateTime::currentMSecsSinceEpoch();
                if (now >= myInvulnUntilMs) {
                    myHP -= dmg;
                    if (myHP < 0) myHP = 0;
                }
                updateHpUi();
                checkGameOver();
                if (udp && !peerAddr.isNull() && peerPort) {
                    QJsonObject reply{{"type","hp"},{"hp", myHP}};
                    udp->writeDatagram(QJsonDocument(reply).toJson(QJsonDocument::Compact), peerAddr, peerPort);
                }
            }

        }  else if (type=="hp") {
            peerHP = o.value("hp").toInt(peerHP);
            updateHpUi();
            checkGameOver();

        } else if (type=="chat") {
            const QString msg = o.value("msg").toString();
            if (!msg.isEmpty()) appendChatMessage("상대", msg);

        } else if (type=="spawn_weapon") {
            const quint64 id = (quint64) o.value("id").toDouble();
            const QString k  = o.value("kind").toString();
            const QPointF pos(o.value("x").toDouble(), o.value("y").toDouble());
            spawnPickupLocal(id, pickupKindFromKey(k), pos);

        } else if (type=="pickup_weapon") {
            const quint64 id = (quint64) o.value("id").toDouble();
            removePickupById(id);

        } else if (type=="hello") {
            ui->statusbar->showMessage(
                QString("상대 접속 확인 (상대 host=%1)").arg(o.value("host").toBool(false)), 800);

            // 참가자 들어오면 게임 시작 여부 묻기
            QMessageBox::StandardButton reply =
                QMessageBox::question(this, "게임 시작", "상대가 들어왔습니다. 게임을 시작할까요?",
                                      QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                restartReadyLocal = true;
                if (udp) {
                    QJsonObject o{{"type","start_ready"}};
                    udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                                       peerAddr, peerPort);
                }
            }

        } else if (type=="restart_request") {
            // 상대가 재시작 요청  팝업 ->띄우기
            QMessageBox::StandardButton reply =
                QMessageBox::question(this, "재시작 요청", "상대가 재시작을 요청했습니다. 수락하시겠습니까?",
                                      QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                restartReadyLocal = true;
                if (udp) {
                    QJsonObject o{{"type","restart_ready"}};
                    udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact),
                                       peerAddr, peerPort);
                }
            }
        }

        else if (type=="restart_ready") {
            restartReadyPeer = true;
            if (restartReadyLocal || restartReadyPeer) {
                startNewGame();
            }
        }

        else if (type=="start_ready") {
            QMessageBox::information(this, "게임 시작", "새 게임이 시작되었습니다!");
        }
        else if (type=="hp_sync") {
            // 호스트가 재시작 시 보내주는 HP 초기 동기화
            const bool senderIsHost = o.value("host").toBool(false);

            // 참가자만 적용
            if (!isHost && senderIsHost) {
                // 호스트 관점에서 my=호스트 HP, peer=참가자 HP → 참가자 관점으로 뒤집어서 반영
                peerHP = o.value("my").toInt(100);   // 상대(호스트) HP
                myHP   = o.value("peer").toInt(100); // 내(참가자) HP
                updateHpUi();
            }
        }
        else if (type=="game_over") {
            const QString winnerKey = o.value("winner").toString("draw");

            // 채팅창에 승자 표시
            appendWinnerMessage(winnerKey);

            // 수신측도 게임오버 고정
            inGameOver = true;

            // 팝업
            QMessageBox::information(this, "게임 종료",
                                     (winnerKey=="draw") ? "무승부! 재시작 준비" :
                                         // 내 관점
                                         ((winnerKey=="host") == isHost ? "내 승리! 재시작 준비" : "상대 승리! 재시작 준비"));
        }


    }
}



void MainWindow::updateMiniMapView()
{
    if (!scene) return;
    ui->miniView->resetTransform();
    ui->miniView->fitInView(scene->sceneRect().adjusted(-50,-50,50,50), Qt::KeepAspectRatio);
    updateMiniViewRect();
}

void MainWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{ updateMiniMapView(); });
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
    updateMiniMapView();
}

void MainWindow::updateMiniViewRect()
{
    // 추가하려다가 만 기능
}

QPointF MainWindow::findSafeSpawn(qreal radius, int tries) const
{
    if (!scene) return QPointF(0, 0);

    const qreal margin = 40.0 + radius;
    QRectF R = scene->sceneRect().adjusted(margin, margin, -margin, -margin);

    QPointF firstPlayerPos;
    bool foundFirstPlayer = false;

    // 첫 번째 플레이어의 안전한 위치 찾기
    for (int i = 0; i < tries; ++i) {
        QPointF p(
            QRandomGenerator::global()->bounded(int(R.left()), int(R.right())),
            QRandomGenerator::global()->bounded(int(R.top()), int(R.bottom()))
            );

        auto* ghost = new QGraphicsEllipseItem(-radius, -radius, 2 * radius, 2 * radius);
        ghost->setPos(p);

        bool bad = false;
        for (auto* r : walls) {
            if (ghost->collidesWithItem(r)) {
                bad = true;
                break;
            }
        }

        if (!bad) {
            for (auto* e : waters) {
                if (ghost->collidesWithItem(e)) {
                    bad = true;
                    break;
                }
            }
        }

        if (!bad) {
            // 플레이어와 너무 가까운 위치가 아닌지 확인
            if (playerItem && (playerPos - p).manhattanLength() < (2 * radius + 50)) {
                bad = true;
            }
        }

        delete ghost;

        if (!bad) {
            firstPlayerPos = p;
            foundFirstPlayer = true;
            break;
        }
    }

    // 첫 번째 플레이어가 안전한 위치를 찾았다면 두 번째 플레이어를 스폰할 때 체크
    if (foundFirstPlayer) {
        // 두 번째 플레이어가 겹치지 않도록 위치 확인
        for (int i = 0; i < tries; ++i) {
            QPointF p(
                QRandomGenerator::global()->bounded(int(R.left()), int(R.right())),
                QRandomGenerator::global()->bounded(int(R.top()), int(R.bottom()))
                );

            auto* ghost = new QGraphicsEllipseItem(-radius, -radius, 2 * radius, 2 * radius);
            ghost->setPos(p);

            bool bad = false;
            for (auto* r : walls) {
                if (ghost->collidesWithItem(r)) {
                    bad = true;
                    break;
                }
            }

            if (!bad) {
                for (auto* e : waters) {
                    if (ghost->collidesWithItem(e)) {
                        bad = true;
                        break;
                    }
                }
            }

            if (!bad) {
                // 두 번째 플레이어가 첫 번째 플레이어와 너무 가까운지 체크
                if ((firstPlayerPos - p).manhattanLength() < (2 * radius + 50)) {
                    bad = true;
                }
            }

            delete ghost;

            if (!bad) return p;  // 안전한 두 번째 위치를 반환
        }
    }

    // 찾기 실패 시, 랜덤 위치
    const int grid = 20;
    const qreal stepX = R.width() / (grid + 1);
    const qreal stepY = R.height() / (grid + 1);
    for (int gy = 1; gy <= grid; ++gy) {
        for (int gx = 1; gx <= grid; ++gx) {
            QPointF p(R.left() + gx * stepX, R.top() + gy * stepY);

            auto* ghost = new QGraphicsEllipseItem(-radius, -radius, 2 * radius, 2 * radius);
            ghost->setPos(p);

            bool bad = false;
            for (auto* r : walls) {
                if (ghost->collidesWithItem(r)) {
                    bad = true;
                    break;
                }
            }

            if (!bad) {
                for (auto* e : waters) {
                    if (ghost->collidesWithItem(e)) {
                        bad = true;
                        break;
                    }
                }
            }

            delete ghost;

            if (!bad) return p;  // 안전한 위치를 반환
        }
    }

    return R.center();  // 중앙
}


void MainWindow::teleportPlayerTo(const QPointF& p, bool broadcast)
{
    playerPos = p;
    playerItem->setRect(p.x()-playerRadius, p.y()-playerRadius, playerRadius*2, playerRadius*2);
    ui->mainView->centerOn(playerItem);
    updateMiniViewRect();

    if (broadcast && udp && !peerAddr.isNull() && peerPort) {
        QJsonObject o{{"type","pos"},{"x",playerPos.x()},{"y",playerPos.y()}};
        udp->writeDatagram(QJsonDocument(o).toJson(QJsonDocument::Compact), peerAddr, peerPort);
    }
}

void MainWindow::startMyInvulnerability(int ms)
{
    myInvulnUntilMs = QDateTime::currentMSecsSinceEpoch() + ms;

    if (!invulnBlinkTimer) {
        invulnBlinkTimer = new QTimer(this);
        connect(invulnBlinkTimer, &QTimer::timeout, this, [this]{
            const qint64 now = QDateTime::currentMSecsSinceEpoch();
            if (now >= myInvulnUntilMs) {
                invulnBlinkTimer->stop();
                playerItem->setPen(playerDefaultPen);
                return;
            }

            invulnBlinkOn = !invulnBlinkOn;
            if (invulnBlinkOn) {
                QPen p(Qt::yellow);
                p.setWidth(3);
                playerItem->setPen(p);
            } else {
                playerItem->setPen(playerDefaultPen);
            }
        });
    }
    invulnBlinkTimer->start(120);
}
