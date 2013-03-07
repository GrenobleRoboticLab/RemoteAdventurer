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

/**
 * This class provides simple way to transmit xml string order to ros topic.
 */
class NXTOrder
{
public:
    NXTOrder(ros::NodeHandle & nh) : m_Commander(nh) { ; }
    ~NXTOrder() { ; }

    /**
     * Transform the given XML string into an order object and send it through a ros topic.
     */
    bool                    treat(const std::string & sXML);

private:
    nxt_adventurer::Order   m_Order;
    NXTLocalCommander       m_Commander;
    XMLOrderHelper          m_XMLOrderHelper;

}; // class NXTOrder

/**
 * This class provides a simple way to listen NXT ros topic and send it through a TcpServer.
 */
class NXTDashboard : public DashboardListener, public QThread
{
public:
    NXTDashboard() { m_pTcpServer = NULL; }
    NXTDashboard(ros::NodeHandle & nh);
    ~NXTDashboard() { ; }

    /**
     * Set the TcpServer use to send Dashboard informations.
     */
    void                    setTcpServer(TcpServer* pTcpServer = NULL) { m_pTcpServer = pTcpServer; }

    /**
     * receive Dashboard's updates notification and send it through the TcpServer.
     */
    virtual void            onDashboardUpdate(Dashboard * pDashboard);
    /**
     * Start a ros::spin to catch information from ros topics.
     */
    virtual void            run() { ros::spin(); }

private:
    NXTWatcherFactory       m_WatcherFactory;
    Dashboard               m_Dashboard;
    Dashboard               m_OldDashboard;
    TcpServer*              m_pTcpServer;
    XMLDashboardHelper      m_XMLDashboardHelper;
    QTime                   m_Timer;
}; // class NXTDashboard

/**
 * This class provides a network interface to send dashboard and receive order over the network.
 * <p>
 * Be aware that only one client can be connected at time.
 */
class TcpServer : public QTcpServer
{
    Q_OBJECT
public:
    TcpServer(QObject* parent = NULL, int nPort = 4000);
    ~TcpServer();

    /**
     * Indicate if this TcpServer is listening.
     */
    bool                    ListenSucceed;
    /**
     * Start the m_NxtDashboard thread.
     * @see NXTDashboard::run()
     */
    void                    run() { m_NxtDashboard.start(); }

public slots :
    /**
     * Send the given string over the network.
     */
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
