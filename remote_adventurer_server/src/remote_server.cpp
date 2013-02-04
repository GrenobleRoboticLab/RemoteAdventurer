#include <ros/ros.h>
#include <remote_adventurer_server/RA_TcpServer.h>
#include <QtCore>

using namespace RemoteAdventurerServer;

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    ros::init(argc, argv, "remote_server");

    TcpServer t;
    t.run();

    return app.exec();
}
