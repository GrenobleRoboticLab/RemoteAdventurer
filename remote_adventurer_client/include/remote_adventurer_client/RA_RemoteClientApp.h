#ifndef RA_REMOTECLIENTAPP_H
#define RA_REMOTECLIENTAPP_H

#include <remote_adventurer_client/RA_MainWindow.h>
#include <remote_adventurer_client/RA_ConnectDialog.h>
#include <remote_adventurer_client/RA_TcpClient.h>

namespace RemoteAdventurerCLient {

class RemoteClientApp : public QApplication
{
    Q_OBJECT
public:
    RemoteClientApp(int argc, char** argv);
    ~RemoteClientApp() { ; }

    int             process();

private slots:
    void            attemptConnection(QString sIp, QString sInt) { m_TcpClient.connect(sIp, sInt.toInt()); }
    void            attemptQuit() { exit(0); }
    void            connected();

private:
    MainWindow      m_MainWindow;
    TcpClient       m_TcpClient;
    ConnectDialog   m_ConnectDialog;

};

}

#endif // RA_REMOTECLIENTAPP_H