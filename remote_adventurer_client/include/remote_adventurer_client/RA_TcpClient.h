#ifndef RA_TCPCLIENT_H
#define RA_TCPCLIENT_H

#include "nxt_adventurer/RA_XMLHelper.h"

#include <QtNetwork>

using namespace RemoteAdventurer;

namespace RemoteAdventurerCLient
{

class TcpClient : public QObject
{
Q_OBJECT
public:
    TcpClient(QObject* parent = NULL);
    ~TcpClient();

    void                connect(const QString & sIp, int nPort);
    bool                isConnected() { return (m_Socket.state() == QAbstractSocket::ConnectedState); }

    void                sendStr(const QString &sText);

signals:
    void                connected();
    void                dashUpdated(Dashboard dash);

private slots:
    void                connectionSuccess();
    void                readServerMsg();

private:
    QTcpSocket          m_Socket;
    QString             m_sIp;
    int                 m_nPort;
    Dashboard           m_Dash;
    XMLDashboardHelper  m_XmlDash;
}; // class TcpClient

} // namespace RemoteAdventurer

#endif // RA_TCPCLIENT_H
