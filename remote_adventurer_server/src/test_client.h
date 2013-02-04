#ifndef TEST_CLIENT_H
#define TEST_CLIENT_H

#include <QtNetwork>

class TcpClient : public QObject
{
Q_OBJECT
public:
    TcpClient(QObject* parent = NULL);
    ~TcpClient();

    void        connect(const std::string & sIp, int nPort);
    bool        isConnected() { return (m_Socket.state() == QAbstractSocket::ConnectedState); }

    void        sendStr(const std::string & sText);

private slots:
    void        connectionSuccess();
    void        readServerMsg();

private:
    QTcpSocket  m_Socket;
    std::string m_sIp;
    int         m_nPort;
};

#endif // TEST_CLIENT_H
