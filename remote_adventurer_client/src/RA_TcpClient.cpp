#include "nxt_adventurer/RA_TcpClient.h"

#include <iostream>

using namespace RemoteAdventurer;

/* ----------------------------- TcpServer ----------------------------- */

TcpClient::TcpClient(QObject * parent)
    : QObject(parent)
{
    QObject::connect(&m_Socket, SIGNAL(connected()), this, SLOT(connectionSuccess()));
    QObject::connect(&m_Socket, SIGNAL(readyRead()), this, SLOT(readServerMsg()));
}

TcpClient::~TcpClient()
{
    std::cout << "client killed" << std::endl;
}

void TcpClient::connect(const std::string &sIp, int nPort)
{
    m_sIp   =   sIp;
    m_nPort =   nPort;
    m_Socket.connectToHost(QHostAddress("127.0.0.1"), m_nPort);
}

void TcpClient::sendStr(const std::string &sText)
{
    QTextStream stream(&m_Socket);
    stream << sText.c_str();
}

void TcpClient::connectionSuccess()
{
    std::cout << "CONNECTED !" << std::endl; // todo:
    sendStr("CONNECTED");
}

void TcpClient::readServerMsg()
{
    char    sBuffer[1280];
    int     nLinesRead = 0;
    while (nLinesRead >= 0)
    {
        nLinesRead = m_Socket.readLine(sBuffer, 1280);
        if (nLinesRead != -1)
            std::cout << std::string(sBuffer, nLinesRead) << std::endl;
    }
}
