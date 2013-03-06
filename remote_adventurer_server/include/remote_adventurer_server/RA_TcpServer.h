#ifndef RA_TCPSERVER_H
#define RA_TCPSERVER_H

#include <nxt_adventurer/RA_Dashboard.h>
#include <nxt_adventurer/RA_Commander.h>
#include <nxt_adventurer/RA_XMLHelper.h>
#include <QtNetwork>
#include <QThread>

#define MAX_CONNECTIONS 1

using namespace RemoteAdventurer;

namespace RemoteAdventurerServer
{
class TcpServer;

class NXTOrder
{
public:
    NXTOrder(ros::NodeHandle & nh) : m_Commander(nh) { ; }
    ~NXTOrder() { ; }

    bool                    treat(const std::string & sXML);

private:
    nxt_adventurer::Order   m_Order;
    NXTLocalCommander       m_Commander;
    XMLOrderHelper          m_XMLOrderHelper;

}; // class NXTOrder

class NXTDashboard : public DashboardListener, public QThread
{
public:
    NXTDashboard() { m_pTcpServer = NULL; }
    NXTDashboard(ros::NodeHandle & nh);
    ~NXTDashboard() { ; }

    void                    setTcpServer(TcpServer* pTcpServer = NULL) { m_pTcpServer = pTcpServer; }

    virtual void            onDashboardUpdate(Dashboard * pDashboard);
    virtual void            run() { ros::spin(); }

private:
    NXTWatcherFactory       m_WatcherFactory;
    Dashboard               m_Dashboard;
    Dashboard               m_OldDashboard;
    TcpServer*              m_pTcpServer;
    XMLDashboardHelper      m_XMLDashboardHelper;
    QTime                   m_Timer;
}; // class NXTDashboard

class TcpServer : public QTcpServer
{
    Q_OBJECT
public:
    TcpServer(QObject* parent = NULL, int nPort = 4000);
    ~TcpServer();

    bool                    ListenSucceed;
    void                    run() { m_NxtDashboard.start(); }

public slots :
    void                    sendStr(const QString & sValue);

private slots :
    void                    attempNewConnection();
    void                    readClientMsg();
    void                    disconnectSocket();

private:
    QTcpSocket*             m_pSocket;
    ros::NodeHandle         m_NodeHandle;

    NXTOrder                m_NxtOrder;
    NXTDashboard            m_NxtDashboard;
}; // class TcpServer


} // namespace RemoteAdventurer

#endif // RA_TCPSERVER_H
