#include "remote_adventurer_server/RA_TcpServer.h"

#include <QMetaObject>

using namespace RemoteAdventurerServer;

/* ----------------------------- NXTOrder ----------------------------- */

bool NXTOrder::treat(const std::string & sXML)
{
    bool bRet = false;

    if(ISOK(m_XMLOrderHelper.load(sXML, m_Order)))
    {
        m_Commander.sendOrder(m_Order);
        bRet = true;
    }
    return bRet;
}

/* ----------------------------- NXTDashboard ----------------------------- */

NXTDashboard::NXTDashboard(ros::NodeHandle &nh)
{
    m_pTcpServer = NULL;
    m_WatcherFactory.connect(nh, &m_Dashboard);
    m_Dashboard.addListener(this);
    m_Timer.start();
}

void NXTDashboard::onDashboardUpdate(Dashboard * pDashboard)
{
    std::string sXML;

    if (pDashboard
    && (m_OldDashboard != *pDashboard)
    && (m_Timer.elapsed() >= 500))
    {
        m_XMLDashboardHelper.genXMLString(*pDashboard, sXML);
        QString sqXML(sXML.c_str());

        if (m_pTcpServer)
            QMetaObject::invokeMethod(m_pTcpServer, "sendStr", Qt::QueuedConnection, Q_ARG(QString, sqXML));

        m_Timer.start();
        m_OldDashboard = *pDashboard;
    }
}

/* ----------------------------- TcpServer ----------------------------- */

TcpServer::TcpServer(QObject *parent, int nPort)
    : QTcpServer(parent), m_NxtOrder(m_NodeHandle), m_NxtDashboard(m_NodeHandle)
{
    m_pSocket       = NULL;
    ListenSucceed   = listen(QHostAddress::Any,nPort);

    m_NxtDashboard.setTcpServer(this);
    setMaxPendingConnections(MAX_CONNECTIONS);
    QObject::connect(this, SIGNAL(newConnection()), this, SLOT(attempNewConnection()));
}

TcpServer::~TcpServer()
{
    if(m_pSocket)
        m_pSocket->close();
}

void TcpServer::sendStr(const QString &sValue)
{
    if (m_pSocket)
    {
        QTextStream stream(m_pSocket);
        stream << sValue << endl;
    }
}

void TcpServer::attempNewConnection()
{
    if(!m_pSocket)
    {
        std::cout << "New socket connected." << std::endl;
        m_pSocket = nextPendingConnection();
        QObject::connect(m_pSocket, SIGNAL(disconnected()), this, SLOT(disconnectSocket()));
        QObject::connect(m_pSocket, SIGNAL(readyRead()), this, SLOT(readClientMsg()));
        sendStr("You're now able to send me order !");
    }
    else
        nextPendingConnection()->close();
}

void TcpServer::disconnectSocket()
{
    std::cout << "Socket disconnected." << std::endl;
    if (m_pSocket)
        m_pSocket->close();
    m_pSocket = NULL;
}


void TcpServer::readClientMsg()
{
    std::string sReceive;
    char        cBuffer[4000];
    int         nLinesRead = 0;

    while (m_pSocket->canReadLine())
    {
        nLinesRead = m_pSocket->readLine(cBuffer, 1280);
        if (nLinesRead != -1)
            sReceive += cBuffer;
    }

    if (m_NxtOrder.treat(sReceive))
        sendStr("Your order as been trasmitted.");
    else
        sendStr("Did you send me an order ?");
 }
