#include "remote_adventurer_client/RA_RemoteClientApp.h"

using namespace RemoteAdventurerClient;

RemoteClientApp::RemoteClientApp(int argc, char **argv) : QApplication(argc, argv)
{
    QObject::connect(&m_ConnectDialog, SIGNAL(attemptConnection(QString, QString)), this, SLOT(attemptConnection(QString, QString)), Qt::AutoConnection);
    QObject::connect(&m_ConnectDialog, SIGNAL(attemptQuit()), this, SLOT(attemptQuit()), Qt::AutoConnection);
    QObject::connect(&m_TcpClient, SIGNAL(connected()), this, SLOT(connected()), Qt::AutoConnection);
    QObject::connect(&m_TcpClient, SIGNAL(dashUpdated(Dashboard)), &m_MainWindow, SLOT(updateDash(Dashboard)), Qt::AutoConnection);
}

RemoteClientApp::~RemoteClientApp() { std::cout << "Destroying RemoteClientApp" << std::endl; }

int RemoteClientApp::process()
{
    m_ConnectDialog.exec();

    if (m_TcpClient.isConnected())
        return exec();
    std::cout << "No connection found... Quitting" << std::endl;

    return 0;
}

void RemoteClientApp::attemptConnection(QString sIp, QString sInt)
{
    std::cout << "Ask for new connection at : " << sIp.toStdString() << " on port : " << sInt.toStdString() << std::endl;
    m_TcpClient.connect(sIp, sInt.toInt());
}

void RemoteClientApp::attemptQuit()
{
    std::cout << "Killing connection Dialog" << std::endl;
    m_ConnectDialog.done(0);
}

void RemoteClientApp::connected()
{
    std::cout << "Connection success... Killing connection Dialog" << std::endl;
    m_ConnectDialog.done(0);
    std::cout << "Showing MainWindow" << std::endl;
    m_MainWindow.show();
}
