#include <nxt_adventurer/RA_XMLHelper.h>
#include <iostream>
#include "remote_adventurer_client/RA_TcpClient.h"

using namespace RemoteAdventurerClient;

/* ----------------------------- TcpServer ----------------------------- */

TcpClient::TcpClient(QObject * parent)
    : QObject(parent)
{
    QObject::connect(&m_Socket, SIGNAL(connected()), this, SLOT(connectionSuccess()));
    QObject::connect(&m_Socket, SIGNAL(readyRead()), this, SLOT(readServerMsg()));
}

TcpClient::~TcpClient()
{
    std::cout << "Destroying TcpClient" << std::endl;
    if (isConnected())
        m_Socket.close();
}

void TcpClient::connect(const QString &sIp, int nPort)
{
    m_sIp   =   sIp;
    m_nPort =   nPort;
    std::cout << "Connecting to remote Host..." << std::endl;
    m_Socket.connectToHost(QHostAddress(sIp), m_nPort);
}

void TcpClient::sendStr(const QString &sText)
{
    std::cout << "Sending datas over socket" << std::endl;
    QTextStream stream(&m_Socket);
    stream << sText;
    m_Socket.waitForBytesWritten(-1);
}

void TcpClient::connectionSuccess()
{
    emit connected();
}

void TcpClient::readServerMsg()
{
    std::string sReceive;
    char        cBuffer[4000];
    int         nLinesRead = 0;

    while (m_Socket.canReadLine())
    {
        nLinesRead = m_Socket.readLine(cBuffer, 1280);
        if (nLinesRead != -1)
            sReceive.append(cBuffer, nLinesRead);
    }

    if(ISOK(m_XmlDash.load(sReceive, m_Dash)))
    {
        emit dashUpdated(m_Dash);
    }
    else
        std::cout << "Unable to load received datas." << std::endl;
}
