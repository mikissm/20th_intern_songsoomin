#include "weapon.h"
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QTimer>
#include <QtMath>
#include <QRandomGenerator>

// 생성자
Weapon::Weapon(Kind kind,
               const QString& name,
               qreal bulletSpeed,
               int   magazineSize,
               int   reloadTimeMs,
               int   damage,
               qreal rangePx)
    : kind_(kind), name_(name), bulletSpeed_(bulletSpeed),
    magazineSize_(magazineSize), reloadTimeMs_(reloadTimeMs),
    damage_(damage), rangePx_(rangePx),
    ammo_(magazineSize), isReloading_(false)
{}

//  공통 발사 함수
bool Weapon::fire(QGraphicsScene* scene,
                  const QPointF& playerPos,
                  const QPointF& targetScenePos,
                  QObject* ctx)
{
    if(isReloading_ || ammo_ <= 0) return false;
    ammo_--;

    switch(kind_) {
    case BASIC:
        spawnOneBullet_(scene, playerPos, targetScenePos,
                        bulletSpeed_, rangePx_, 8, ctx);
        break;
    case MULTI: // 따발총: 여러 발 퍼짐
        spawnSpread_(scene, playerPos, targetScenePos,
                     2, 15.0, bulletSpeed_, rangePx_, 6, ctx);
        break;
    case 저격총: // 저격총: 크고 빠른 1발
        spawnOneBullet_(scene, playerPos, targetScenePos,
                        bulletSpeed_*1.5, rangePx_, 14, ctx);
        break;
    case SHOTGUN: // 샷건: 여러 발 랜덤 퍼짐
        spawnSpread_(scene, playerPos, targetScenePos,
                     6, 10.0, bulletSpeed_, rangePx_, 5, ctx);
        break;
    }
    return true;
}

// 장전
void Weapon::reload(QObject* ctx, std::function<void(const QString&)> onText)
{
    if(isReloading_ || ammo_==magazineSize_) return;
    isReloading_ = true;
    if(onText) onText(QString("%1 장전중...").arg(name_));

    QTimer::singleShot(reloadTimeMs_, ctx, [=]() mutable {
        ammo_ = magazineSize_;
        isReloading_ = false;
        if(onText) onText(QString("%1 장전 완료").arg(name_));
    });
}

//  습득
void Weapon::onPicked(std::function<void(const QString&)> onText)
{
    isReloading_ = false;
    if(onText) onText(QString("%1 획득").arg(name_));
}

//  버림
void Weapon::onDropped(std::function<void(const QString&)> onText)
{
    if(onText) onText(QString("%1 버림").arg(name_));
}

//  내부: 총알 1발, 생성용 가상총알
void Weapon::spawnOneBullet_(QGraphicsScene* scene, const QPointF& from, const QPointF& to,
                             qreal speed, qreal rangePx, int size, QObject* ctx)
{
    QLineF dir(from, to);
    dir.setLength(speed);
    QPointF velocity = dir.p2() - dir.p1();

    auto* bullet = scene->addEllipse(-size/2, -size/2, size, size,
                                     QPen(QColor(0, 0, 0, 0)), QBrush(QColor(0, 0, 0, 0)));
    bullet->setPos(from);

    QPointF start = from;
    QTimer* timer = new QTimer(ctx);
    QObject::connect(timer, &QTimer::timeout, [=]() mutable {
        bullet->moveBy(velocity.x(), velocity.y());
        if(QLineF(start, bullet->pos()).length() > rangePx ||
            !scene->sceneRect().intersects(bullet->sceneBoundingRect())) {
            scene->removeItem(bullet);
            delete bullet;
            timer->stop();
            timer->deleteLater();
        }
    });
    timer->start(16);
}

//  내부: 여러 발 퍼뜨리기, 생성용 가상총알
void Weapon::spawnSpread_(QGraphicsScene* scene, const QPointF& from, const QPointF& to,
                          int count, qreal totalSpreadDeg, qreal speed, qreal rangePx, int size, QObject* ctx)
{
    QLineF base(from, to);
    qreal baseAngle = base.angle();

    for(int i=0; i<count; i++) {
        qreal offset = -totalSpreadDeg/2 + i*(totalSpreadDeg/(count-1));
        QLineF dir = base;
        dir.setAngle(baseAngle + offset);
        dir.setLength(speed);
        QPointF velocity = dir.p2() - dir.p1();

        auto* bullet = scene->addEllipse(-size/2, -size/2, size, size,
                                         QPen(QColor(0, 0, 0, 0)), QBrush(QColor(0, 0, 0, 0)));
        bullet->setPos(from);

        QPointF start = from;
        QTimer* timer = new QTimer(ctx);
        QObject::connect(timer, &QTimer::timeout, [=]() mutable {
            bullet->moveBy(velocity.x(), velocity.y());
            if(QLineF(start, bullet->pos()).length() > rangePx ||
                !scene->sceneRect().intersects(bullet->sceneBoundingRect())) {
                scene->removeItem(bullet);
                delete bullet;
                timer->stop();
                timer->deleteLater();
            }
        });
        timer->start(16);
    }
}


// weapon.cpp
void Weapon::setAmmo(int n) {
    ammo_ = n;
}
