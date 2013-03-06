#include <remote_adventurer_client/RA_RemoteClientApp.h>
#include <QApplication>

using namespace RemoteAdventurerClient;


int main(int argc, char** argv)
{
    RemoteClientApp    app(argc, argv);
    return app.process();
}
