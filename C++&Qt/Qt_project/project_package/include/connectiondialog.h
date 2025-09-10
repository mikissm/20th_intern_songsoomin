#pragma once
#include <QDialog>
#include <QHostAddress>

class QLineEdit;
class QPushButton;
class QLabel;
class QRadioButton;

class ConnectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit ConnectionDialog(QWidget* parent=nullptr);

    // 결과 조회
    bool    isHost()     const { return m_isHost; }
    QString localIp()    const;
    quint16 localPort()  const;
    QString peerIp()     const;
    quint16 peerPort()   const;

private:
    QString detectLocalIPv4() const;
    void    updateRoleUi();

    // UI 위젯
    QRadioButton* rbHost      = nullptr;
    QRadioButton* rbJoin      = nullptr;

    QLineEdit*    editLocalIp   = nullptr;
    QLineEdit*    editLocalPort = nullptr;
    QLineEdit*    editPeerIp    = nullptr;
    QLineEdit*    editPeerPort  = nullptr;
    QPushButton*  okBtn         = nullptr;
    QPushButton*  cancelBtn     = nullptr;

    bool m_isHost = true;
};
