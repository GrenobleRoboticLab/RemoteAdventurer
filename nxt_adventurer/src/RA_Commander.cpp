#include "nxt_adventurer/RA_Commander.h"

using namespace RemoteAdventurer;

/* ------------------------ NXTLocalCommander ------------------------ */

NXTLocalCommander::NXTLocalCommander(ros::NodeHandle & nh, const std::string & sName)
{
    m_Publisher = nh.advertise<nxt_adventurer::Order>(sName, 1);
}

void NXTLocalCommander::sendOrder(const nxt_adventurer::Order & order)
{
    m_Order = order;
    m_Publisher.publish(m_Order);
}

void NXTLocalCommander::sendOrder(unsigned long wOrder, float fEffort, unsigned long wDirection)
{
    nxt_adventurer::Order order;

    order.order     = wOrder;
    order.effort    = fEffort;
    order.direction = wDirection;

    sendOrder(order);
}
