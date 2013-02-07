#ifndef RA_COMMANDER_H
#define RA_COMMANDER_H

#include <ros/ros.h>
#include <nxt_adventurer/Order.h>

namespace RemoteAdventurer
{

/**
 * The NXTLocalCommander class provide simple way to publish on NXTAdventurer topic.
 */
class NXTLocalCommander
{
public:
    /**
     * The NXTOrderType enum provide simple nomenclature to assign order parameter of a nxt_adventurer::Order message.
     */
    typedef enum
    {
        NOT_MOVE    = 0,
        NOT_TURN    = 1
    } NXTOrderType;

    /**
     * The NXTOrderType enum provide simple nomenclature to assign direction parameter of a nxt_adventurer::Order message for a move order type.
     */
    typedef enum
    {
        NMD_BACK    = 0,
        NMD_AHEAD   = 1
    } NXTMoveDirection;

    /**
     * The NXTOrderType enum provide simple nomenclature to assign direction parameter of a nxt_adventurer::Order message for a turn order type.
     */
    typedef enum
    {
        NTD_LEFT     = 0,
        NTD_RIGHT    = 1

    } NXTTurnDirection;

    NXTLocalCommander(ros::NodeHandle & nh, const std::string & sName = "adventurer_order");
    virtual ~NXTLocalCommander() { ; }

    /**
     * Send a nxt_adventurer::Order over the ros topic instanciate with this NXTLocalCommander object.
     * @param order the order message to be threw over the topic
     */
    void                    sendOrder(const nxt_adventurer::Order & order);
    /**
     * Construct and send a nxt_adventurer::Order over the ros topic instanciate with this NXTLocalCommander object.
     * @param wOrder can be either a custom parameter or a NXTOrderType (for more compatibility, prefere to use NXTOrderType)
     * @param fEffort is the effort transmit to steering motors.
     * @param wDirection can be either 0, 1, a NXTMoveDirection or a NXTTurnDirection, depending on the wOrder given parameter
     */
    void                    sendOrder(unsigned long wOrder, float fEffort, unsigned long wDirection);

    nxt_adventurer::Order   getLasOrder() { return m_Order; }

private:
    ros::Publisher          m_Publisher;
    nxt_adventurer::Order   m_Order;
}; // class NXTLocalCommander

} // namespace RemoteAdventurer

#endif // RA_COMMANDER_H
