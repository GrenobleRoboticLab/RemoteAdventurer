#ifndef RA_TCPCLIENT_H
#define RA_TCPCLIENT_H

#include "nxt_adventurer/RA_XMLHelper.h"

#include <QtNetwork>

using namespace RemoteAdventurer;

namespace RemoteAdventurerClient
{

/**
 * This class allows to communicate with a given server.
 */
class TcpClient : public QObject
{
Q_OBJECT
public:
    TcpClient(QObject* parent = NULL);
    ~TcpClient();

    /**
     * Try to connect to the given remote host.
     */
    void                connect(const QString & sIp, int nPort);
    bool                isConnected() { return (m_Socket.state() == QAbstractSocket::ConnectedState); }

    /**
     * Send the given string to the server.
     */
    void                sendStr(const QString &sText);

public slots:
    /**
     * Compute a XML string from the given order and send it to the server.
     */
    void                sendOrder(const nxt_adventurer::Order & order);

signals:
    /**
     * Send a notification when this object is connected to the remote host.
     */
    void                connected();
    /**
     * Send the last dashboard received from the server.
     */
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
    XMLOrderHelper      m_OrderHelper;
}; // class TcpClient

} // namespace RemoteAdventurer

#endif // RA_TCPCLIENT_H
