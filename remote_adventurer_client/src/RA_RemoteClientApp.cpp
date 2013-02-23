#include "remote_adventurer_client/RA_RemoteClientApp.h"

using namespace RemoteAdventurerCLient;

RemoteClientApp::RemoteClientApp(int argc, char **argv) : QApplication(argc, argv)
{
    QObject::connect(&m_ConnectDialog, SIGNAL(attemptConnection(QString, QString)), this, SLOT(attemptConnection(QString, QString)), Qt::AutoConnection);
    QObject::connect(&m_ConnectDialog, SIGNAL(attemptQuit()), this, SLOT(attemptQuit()), Qt::AutoConnection);
    QObject::connect(&m_TcpClient, SIGNAL(connected()), this, SLOT(connected()), Qt::AutoConnection);
    QObject::connect(&m_TcpClient, SIGNAL(dashUpdated(Dashboard)), &m_MainWindow, SLOT(updateDash(Dashboard)), Qt::AutoConnection);
}

int RemoteClientApp::process()
{
    m_ConnectDialog.exec();
    m_MainWindow.show();

    return exec();
}

void RemoteClientApp::connected()
{
    std::cout << "Force Dialog destruction" << std::endl;
    m_ConnectDialog.done(1);
}
