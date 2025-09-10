#pragma once
#include <QString>
#include <QPointF>
#include <functional>

class QGraphicsScene;
class QObject;

class Weapon {
public:
    enum Kind { BASIC=0, MULTI=1, 저격총=2, SHOTGUN=3 };

    Weapon(Kind kind,
           const QString& name,
           qreal bulletSpeed,
           int   magazineSize,
           int   reloadTimeMs,
           int   damage,
           qreal rangePx);

    bool fire(QGraphicsScene* scene,
              const QPointF& playerPos,
              const QPointF& targetScenePos,
              QObject* ctx);

    void reload(QObject* ctx, std::function<void(const QString&)> onText);
    void onPicked(std::function<void(const QString&)> onText);
    void onDropped(std::function<void(const QString&)> onText);

    // getters
    Kind kind() const { return kind_; }
    const QString& name() const { return name_; }
    qreal bulletSpeed() const { return bulletSpeed_; }
    int damage() const { return damage_; }
    int ammo() const { return ammo_; }
    int magSize() const { return magazineSize_; }
    bool reloading() const { return isReloading_; }
    qreal rangePx() const { return rangePx_; }
    void setAmmo(int n);

private:
    void spawnOneBullet_(QGraphicsScene* scene,
                         const QPointF& from,
                         const QPointF& to,
                         qreal speed,
                         qreal rangePx,
                         int size,
                         QObject* ctx);

    void spawnSpread_(QGraphicsScene* scene,
                      const QPointF& from,
                      const QPointF& to,
                      int count,
                      qreal totalSpreadDeg,
                      qreal speed,
                      qreal rangePx,
                      int size,
                      QObject* ctx);

    Kind   kind_;
    QString name_;
    qreal  bulletSpeed_;
    int    magazineSize_;
    int    reloadTimeMs_;
    int    damage_;
    qreal  rangePx_;

    int    ammo_;
    bool   isReloading_;
};
