#include <nxt_adventurer/RA_Commander.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define ACCELERATE 'a'
#define DECELERATE 'e'
#define GO_AHEAD   'z'
#define TURN_LEFT  'q'
#define GO_BACK    's'
#define TURN_RIGHT 'd'
#define STOP       'x'

using namespace RemoteAdventurer;

int kfd = 0;
static struct termios old_term, new_term;

void quit(int sig)
{
        tcsetattr(kfd, TCSANOW, &old_term);
        ros::shutdown();
        exit(0);
}

void initTermios(int echo)
{
    tcgetattr(kfd, &old_term);
    memcpy(&new_term, &old_term, sizeof(struct termios));
    new_term.c_lflag &= ~(ICANON | ECHO);
    new_term.c_cc[VEOL] = 1;
    new_term.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &new_term);
}

void resetTermios()
{
    tcsetattr(0, TCSANOW, &old_term);
}

class Teleop
{
public:
    Teleop() : m_Commander(m_NodeHandle) { initTermios(0); }
    ~Teleop() { resetTermios(); }

    void                    process();

private:
    ros::NodeHandle         m_NodeHandle;
    NXTLocalCommander       m_Commander;
    nxt_adventurer::Order   m_Order;

    void                    treat(char c);
}; // class Teleop

void Teleop::process()
{
    char c;

    puts("Keyboard input");
    puts("--------------");
    puts("[z q s d | x ]");

    while (true)
    {
        // lie la console
        if (read(kfd, &c, 1) < 0)
            exit(-1);
        treat(c);
        m_Commander.sendOrder(m_Order);
    }
}

void Teleop::treat(char c)
{
    switch(c)
    {
    case GO_AHEAD:
        m_Order.order       = NXTLocalCommander::NOT_MOVE;
        m_Order.direction   = NXTLocalCommander::NMD_AHEAD;
        break;
    case TURN_LEFT:
        m_Order.order       = NXTLocalCommander::NOT_TURN;
        m_Order.direction   = NXTLocalCommander::NTD_LEFT;
        break;
    case GO_BACK:
        m_Order.order       = NXTLocalCommander::NOT_MOVE;
        m_Order.direction   = NXTLocalCommander::NMD_BACK;
        break;
    case TURN_RIGHT:
        m_Order.order       = NXTLocalCommander::NOT_TURN;
        m_Order.direction   = NXTLocalCommander::NTD_RIGHT;
        break;
    case ACCELERATE:
        m_Order.effort += 0.1f;
        break;
    case DECELERATE:
        if (m_Order.effort >= 0.1f)
            m_Order.effort -= 0.1f;
        break;
    case STOP:
        m_Order.effort = 0.0f;
        break;
    }
}

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "teleop_node");
    Teleop t;
    signal(SIGINT, quit);

    t.process();

    ros::spin();
    return 0;
}
