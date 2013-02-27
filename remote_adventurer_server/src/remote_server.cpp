#include <ros/ros.h>
#include <remote_adventurer_server/RA_TcpServer.h>
#include <QtCore>

using namespace RemoteAdventurerServer;

bool myEventFilter(void *message, long *result)
{
    std::cout << "event" << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    ros::init(argc, argv, "remote_server");
    app.setEventFilter( myEventFilter );
    TcpServer t;
    t.run();

    return app.exec();
}
