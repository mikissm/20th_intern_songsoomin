#include "connectiondialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QNetworkInterface>
#include <QIntValidator>
#include <QRadioButton>
#include <QMessageBox>

ConnectionDialog::ConnectionDialog(QWidget* parent): QDialog(parent) {
    setWindowTitle(u8"네트워크 설정 (UDP)");
    setModal(true);

    auto* lay = new QVBoxLayout(this);

    // 역할 선택 (호스트/참가)
    rbHost = new QRadioButton(u8"호스트(방 만들기)", this);
    rbJoin = new QRadioButton(u8"참가(접속하기)", this);
    rbHost->setChecked(true);

    auto* roleRow = new QHBoxLayout;
    roleRow->addWidget(rbHost);
    roleRow->addWidget(rbJoin);
    roleRow->addStretch();
    lay->addLayout(roleRow);

    auto row = [&](const QString& label, QWidget* editor){
        auto* h = new QHBoxLayout;
        h->addWidget(new QLabel(label));
        h->addWidget(editor, /*stretch*/1);
        lay->addLayout(h);
    };

    editLocalIp   = new QLineEdit(detectLocalIPv4(), this);
    editLocalIp->setReadOnly(true);

    editLocalPort = new QLineEdit(this);
    editLocalPort->setValidator(new QIntValidator(1, 65535, this));
    editLocalPort->setPlaceholderText(u8"예: 50000");
    editLocalPort->setText("50000");

    editPeerIp    = new QLineEdit(this);
    editPeerIp->setPlaceholderText(u8"상대 IPv4 (예: 192.168.x.y)");

    editPeerPort  = new QLineEdit(this);
    editPeerPort->setValidator(new QIntValidator(1, 65535, this));
    editPeerPort->setPlaceholderText(u8"예: 50001");
    editPeerPort->setText("50001");

    row(u8"내 IP",    editLocalIp);
    row(u8"내 포트",  editLocalPort);
    row(u8"상대 IP",  editPeerIp);
    row(u8"상대 포트", editPeerPort);

    auto* btns = new QHBoxLayout;
    okBtn = new QPushButton(u8"확인");
    cancelBtn = new QPushButton(u8"취소");
    btns->addStretch(); btns->addWidget(okBtn); btns->addWidget(cancelBtn);
    lay->addLayout(btns);

    // 역할 전환 시 UI 활성/비활성
    auto onRoleChanged = [this]{
        m_isHost = rbHost->isChecked();
        updateRoleUi();
    };
    connect(rbHost, &QRadioButton::toggled, this, onRoleChanged);
    connect(rbJoin, &QRadioButton::toggled, this, onRoleChanged);
    updateRoleUi();

    // OK/Cancel
    connect(okBtn, &QPushButton::clicked, this, [this]{
        if (editLocalPort->text().isEmpty()) {
            QMessageBox::warning(this, u8"오류", u8"내 포트를 입력하세요.");
            return;
        }
        if (!m_isHost) {
            if (editPeerIp->text().trimmed().isEmpty() || editPeerPort->text().isEmpty()) {
                QMessageBox::warning(this, u8"오류", u8"참가 모드에서는 상대 IP/포트를 입력해야 합니다.");
                return;
            }
            QHostAddress addr;
            if (!addr.setAddress(editPeerIp->text().trimmed())) {
                QMessageBox::warning(this, u8"오류", u8"상대 IP 형식이 올바르지 않습니다.");
                return;
            }
        }
        accept();
    });
    connect(cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
}

void ConnectionDialog::updateRoleUi() {
    const bool host = rbHost->isChecked();
    // 호스트는 상대 정보를 몰라도 시작 가능 (첫 패킷 수신 시 자동 학습)
    editPeerIp->setEnabled(!host);
    editPeerPort->setEnabled(!host);
}

QString ConnectionDialog::localIp() const { return editLocalIp->text(); }
quint16 ConnectionDialog::localPort() const { return editLocalPort->text().toUShort(); }
QString ConnectionDialog::peerIp() const { return editPeerIp->text().trimmed(); }
quint16 ConnectionDialog::peerPort() const { return editPeerPort->text().toUShort(); }

QString ConnectionDialog::detectLocalIPv4() const {
    for (const QNetworkInterface& nif : QNetworkInterface::allInterfaces()) {
        if (!(nif.flags() & QNetworkInterface::IsUp) || !(nif.flags() & QNetworkInterface::IsRunning))
            continue;
        if (nif.flags() & QNetworkInterface::IsLoopBack)
            continue;
        for (const QNetworkAddressEntry& e : nif.addressEntries()) {
            const QHostAddress ip = e.ip();
            if (ip.protocol() == QAbstractSocket::IPv4Protocol && ip != QHostAddress::LocalHost)
                return ip.toString();
        }
    }
    return QStringLiteral("127.0.0.1");
}
