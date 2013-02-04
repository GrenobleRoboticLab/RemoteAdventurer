#include <QtCore>
#include <iostream>

#include "test_client.h"

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
    stream << sText.c_str() << endl;
}

void TcpClient::connectionSuccess()
{
    sendStr("<?xml version=\"1.0\" ?><order><order value=\"0\" /><effort value=\"0.0\" /><direction value=\"true\" /></order>");
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
            sReceive += cBuffer;
    }

    if(sReceive.size() > 0)
        std::cout << sReceive << std::endl;
}

int main (int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    TcpClient t;
    t.connect("127.0.0.1", 4000);

    return app.exec();
}
