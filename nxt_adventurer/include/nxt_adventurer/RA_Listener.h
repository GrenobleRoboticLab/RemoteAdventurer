#ifndef RA_LISTENER_H
#define RA_LISTENER_H

#include "ros/ros.h"
#include <set>
#include "sensor_msgs/JointState.h"
#include "nxt_msgs/Range.h"
#include "nxt_msgs/Contact.h"
#include "nxt_msgs/Color.h"

namespace RemoteAdventurer
{

class NXTLocalWatcher;

/**
 * The NXTWatcherFactory class provides methods to bind NXTLocalWatcher's callbacks with the ros topics created by nxt_ros node.
 */
class NXTWatcherFactory
{
private:
    struct SubFarm
    {
        int             m_nKey;
        ros::Subscriber m_vMotorSub;
        ros::Subscriber m_vUltrasonicSub;
        ros::Subscriber m_vColorSub;
        ros::Subscriber m_vLeftContactSub;
        ros::Subscriber m_vRightContactSub;
    };

    struct SupFarmComp
    {
        bool operator() (const SubFarm & sf1, const SubFarm & sf2) { return sf1.m_nKey < sf2.m_nKey; }
    };

public:
    NXTWatcherFactory (const std::string & sRightWheelName   = "motor_r",
                       const std::string & sLeftWheelName    = "motor_l",
                       const std::string & sAuxWheelName     = "motor_aux",
                       const std::string & sUltrasonicName   = "ultrasonic_sensor",
                       const std::string & sColorName        = "color_frame",
                       const std::string & sRightContactName = "touch_r",
                       const std::string & sLeftContactName  = "touch_l");

    virtual ~NXTWatcherFactory();

    /**
     * Bind a NXTLocalWatcher's callbacks with ros topics created by nxt_ros node.
     * @param nh        the ros NodeHandle used to subscribe to topics
     * @param pWatcher  the Watcher to be connected.
     * @return the index of the connection.
     */
    int                             connect(ros::NodeHandle & nh, NXTLocalWatcher * pWatcher);
    /**
     * Disconnect the binded identified by the given index.
     * @param nSubId the index of the connection to be disconnect.
     */
    void                            disconnect(int nSubId);

private:
    int                             m_nCount;
    std::string                     m_sRightWheelName;
    std::string                     m_sLeftWheelName;
    std::string                     m_sAuxWheelName;
    std::string                     m_sUltrasonicName;
    std::string                     m_sColorName;
    std::string                     m_sRightContactName;
    std::string                     m_sLeftContactName;

    std::set<SubFarm, SupFarmComp>  m_SubFarms;
}; // class NXTWatcherFactory

class NXTLocalWatcher
{
    friend int NXTWatcherFactory::connect(ros::NodeHandle & nh, NXTLocalWatcher * pWatcher);
public:
    NXTLocalWatcher() { m_pFactory = NULL; }
    virtual ~NXTLocalWatcher() { disconnect(); }

    virtual void        setRightWheelName(const std::string & sName) = 0;
    virtual void        setLeftWheelName(const std::string & sName) = 0;
    virtual void        setAuxWheelName(const std::string & sName) = 0;

protected:
    /**
     * Virtual pure Motor callback.
     */
    virtual void        updateWheels(const sensor_msgs::JointState::ConstPtr& jointState) = 0;
    /**
     * Virtual pure Ultrasonic callback.
     */
    virtual void        updateUltrasonic(const nxt_msgs::Range::ConstPtr& range) = 0;
    /**
     * Virtual pure Color callback.
     */
    virtual void        updateColor(const nxt_msgs::Color::ConstPtr& color) = 0;
    /**
     * Virtual pure Contact callback.
     */
    virtual void        updateRightContact(const nxt_msgs::Contact::ConstPtr& contact) = 0;
    /**
     * Virtual pure Contact callback.
     */
    virtual void        updateLeftContact(const nxt_msgs::Contact::ConstPtr& contact) = 0;

private:
    int                 m_nSubFarmId;
    NXTWatcherFactory*  m_pFactory;

    void                connect(int nSubId, NXTWatcherFactory* pFactory);
    void                disconnect();
}; // class NXTLocalWatcher

} // namespace RemoteAdventurer

#endif // RA_LISTENER_H
