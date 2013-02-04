#ifndef RA_DASHBOARD_H
#define RA_DASHBOARD_H

#include <vector>
#include <string>
#include <boost/signal.hpp>
#include <boost/signals/connection.hpp>
#include "RA_Listener.h"

namespace RemoteAdventurer
{

class DashboardListener;

/**
 * The Wheel class provides informations on a NXT Motor Sensor.
 */
class Wheel
{
public:
    Wheel();
    /**
     * Construct a Wheel object and set his name.
     * @param sName represent the name use to retrieve informmations provides by the nxt_ros node.
     */
    Wheel(const std::string & sName);
    Wheel(const Wheel & wheel);
    virtual ~Wheel();

    const Wheel&        operator=(const Wheel & wheel);
    bool                operator==(const Wheel & wheel);
    bool                operator!=(const Wheel & wheel);

    /**
     * Copy information from a given Wheel.
     * @param wheel the Wheel to be copy.
     */
    void                initFrom(const Wheel & wheel);
    /**
     * Compare all members of this Wheel object with the given Wheel parameter.
     * This method return true if the given wheel parameter is equals to this Wheel object otherwise the return value is false.
     * @param wheel the Wheel object to be compare.
     */
    bool                compare(const Wheel & wheel);
    /**
     * Update this Wheel object with the given joinState information.
     * @param jointState contains informations provides by the "joint_state" topic created by nxt_ros node.
     */
    bool                update(const sensor_msgs::JointState::ConstPtr& jointState);
    /**
     * Reset all fields to empty values.
     */
    void                reset();

    const std::string&  getName()     const { return m_sName;     }
    float               getEffort()   const { return m_fEffort;   }
    float               getPosition() const { return m_fPosition; }
    float               getVelocity() const { return m_fVelocity; }

    void                setName(const std::string & sName);
    void                setEffort(float fEffort)        { m_fEffort     = fEffort;   }
    void                setPosition(float fPosition)    { m_fPosition   = fPosition; }
    void                setVelocity(float fVelocity)    { m_fVelocity   = fVelocity; }

private:
    std::string         m_sName;
    float               m_fEffort;
    float               m_fPosition;
    float               m_fVelocity;
}; // class Wheel

/**
 * The Ultrasonic class provides informations on a NXT Ultrasonic Sensor.
 */
class Ultrasonic
{
public:
    Ultrasonic();
    Ultrasonic(const Ultrasonic & ultrasonic);
    virtual ~Ultrasonic();

    const Ultrasonic&   operator=(const Ultrasonic & ultrasonic);
    bool                operator==(const Ultrasonic & ultrasonic);
    bool                operator!=(const Ultrasonic & ultrasonic);

    /**
     * Copy informations from a given Ultrasonic object.
     * @param ultrasonic the Ultrasonic object to be copy.
     */
    void                initFrom(const Ultrasonic & ultrasonic);
    /**
     * Compare all members of this Ultrasonic object with the given Ultrasonic parameter.
     * This method return true if the given Ultrasonic parameter is equals to this Ultrasonic object otherwise the return value is false.
     * @param ultrasonic the Ultrasonic object to be compare.
     */
    bool                compare(const Ultrasonic & ultrasonic);
    /**
     * Update this Ultrasonic object with the given Range informations.
     * @param range contains informations provides by the "~<ultrasonic_sensor_name>" topic created by nxt_ros node.
     */
    bool                update(const nxt_msgs::Range::ConstPtr & range);
    /**
     * Reset all fields to empty values.
     */
    void                reset();

    float               getRange()       const { return m_fRange;       }
    float               getRangeMin()    const { return m_fRangeMin;    }
    float               getRangeMax()    const { return m_fRangeMax;    }
    float               getSpreadAngle() const { return m_fSpreadAngle; }

    void                setRange(float fValue)          { m_fRange          = fValue; }
    void                setRangeMin(float fValue)       { m_fRangeMin       = fValue; }
    void                setRangeMax(float fValue)       { m_fRangeMax       = fValue; }
    void                setSpreadAngle(float fValue)    { m_fSpreadAngle    = fValue; }

private:
    float               m_fRange;
    float               m_fRangeMin;
    float               m_fRangeMax;
    float               m_fSpreadAngle;
}; // class Ultrassonic

/**
 * The Contact class provide informations on a NXT Touch sensor.
 */
class Contact
{
public:
    Contact();
    Contact(const Contact & contact);
    virtual ~Contact();

    const Contact&      operator=(const Contact & contact);
    bool                operator==(const Contact & contact);
    bool                operator!=(const Contact & contact);

    /**
     * Copy informations from a given Contact object.
     * @param contact the Contact object to be copy.
     */
    void                initFrom(const Contact & contact);
    /**
     * Compare all members of this Contact object with the given Contact parameter.
     * This method return true if the given Contact parameter is equals to this Contact object othrewise the return value is false.
     * @param contact the Contact object to be copy.
     */
    bool                compare(const Contact & contact);
    /**
     * Update this Contact object with the given contact informations.
     * @param contact contains informations provides by the "~<touch_sensor_name>" topic created by nxt_ros node.
     */
    bool                update(const nxt_msgs::Contact::ConstPtr & contact);
    /**
     * Reset all fields to empty values.
     */
    void                reset();

    bool                getContact() const { return m_bContact; }
    void                setContact(bool bValue) { m_bContact = bValue; }

private:
    bool                m_bContact;
}; // class Contact

/**
 * The Color class provide informations on a NXT Color sensor.
 */
class Color
{
public:
    Color();
    Color(const Color & color);
    virtual ~Color();

    const Color&        operator=(const Color & color);
    bool                operator==(const Color & color);
    bool                operator!=(const Color & color);

    /**
     * Copy informations from a given Color object.
     * @param color the Color object to be copy.
     */
    void                initFrom(const Color & color);
    /**
     * Compare all members of this Color object with the given Color parameter.
     * This method return true if the given Color parameter is equals to this Color object othrewise the return value is false.
     * @param color the Color object to be compare.
     */
    bool                compare(const Color & color);
    /**
     * Update this Color object with the given color informations.
     * @param color contains informations provides by the "~<color_sensor_name>" topic created by nxt_ros node.
     */
    bool                update(const nxt_msgs::Color::ConstPtr & color);
    /**
     * Reset all fields to empty values.
     */
    void                reset();

    float               getIntensity() const { return m_fIntensity; }
    float               getRed()       const { return m_fRed;       }
    float               getGreen()     const { return m_fGreen;     }
    float               getBlue()      const { return m_fBlue;      }

    void                setIntensity(float fValue)  { m_fIntensity  = fValue; }
    void                setRed(float fValue)        { m_fRed        = fValue; }
    void                setGreen(float fValue)      { m_fGreen      = fValue; }
    void                setBlue(float fValue)       { m_fBlue       = fValue; }

private:
    float               m_fIntensity;
    float               m_fRed;
    float               m_fGreen;
    float               m_fBlue;
}; // class Color

/**
 * The Dashboard class provide informations on all NXT sensors.
 */
class Dashboard : public NXTLocalWatcher
{
public:
    Dashboard();
    Dashboard(const Dashboard & dash);
    virtual ~Dashboard();

    const Dashboard&                    operator=(const Dashboard & dash);
    bool                                operator==(const Dashboard & dash);
    bool                                operator!=(const Dashboard & dash);

    /**
     * Copy informations from a given Dashboard object.
     * @param dash the Dashboard object to be copy.
     */
    void                                initFrom(const Dashboard & dash);
    /**
     * Compare all members of this Dashboard object with the given Dashboard parameter.
     * This method return true if the given Dashboard parameter is equals to this Dashboard object othrewise the return value is false.
     * @param dash the Dashboard object to be compare.
     */
    bool                                compare(const Dashboard & dash);

    /**
     * Connects the given DashboardListener object and his onDashboardUpdate method to the m_Signal boost::signal member.
     * The onDashboardUpdate will be called on each sensor update.
     * <p>
     * Be carefull, you have to disconnect your DashboardListener (using removeListener method) before destroy it.
     */
    void                                addListener(DashboardListener * pDashboardListener);
    /**
     * Disconnect the given DashboardListener from the m_Signal member.
     * After this method, your DashboardListener will not receive informations anymore.
     */
    void                                removeListener(DashboardListener * pDashboardListener);

    const Wheel&                        getRightWheel()     const { return m_RightWheel;    }
    const Wheel&                        getLeftWheel()      const { return m_LeftWheel;     }
    const Wheel&                        getAuxWheel()       const { return m_AuxWheel;      }
    const Ultrasonic&                   getUltrasonic()     const { return m_Ultrasonic;    }
    const Color&                        getColor()          const { return m_Color;         }
    const Contact&                      getRightContact()   const { return m_RightContact;  }
    const Contact&                      getLeftContact()    const { return m_LeftContact;   }

    virtual void                        setRightWheelName(const std::string & sName)    { m_RightWheel = Wheel(sName); }
    virtual void                        setLeftWheelName (const std::string & sName)    { m_LeftWheel  = Wheel(sName); }
    virtual void                        setAuxWheelName  (const std::string &sName)     { m_AuxWheel   = Wheel(sName); }

    virtual void                        setRightWheel(const Wheel & wheel)              { m_RightWheel      = wheel;      }
    virtual void                        setLeftWheel(const Wheel & wheel)               { m_LeftWheel       = wheel;      }
    virtual void                        setAuxWheel(const Wheel & wheel)                { m_AuxWheel        = wheel;      }
    virtual void                        setUltrasonic(const Ultrasonic & ultrasonic)    { m_Ultrasonic      = ultrasonic; }
    virtual void                        setColor(const Color & color)                   { m_Color           = color;      }
    virtual void                        setRightContact(const Contact & contact)        { m_RightContact    = contact;    }
    virtual void                        setLeftContact(const Contact & contact)         { m_LeftContact     = contact;    }

private:
    Wheel                               m_RightWheel;
    Wheel                               m_LeftWheel;
    Wheel                               m_AuxWheel;
    Ultrasonic                          m_Ultrasonic;
    Color                               m_Color;
    Contact                             m_RightContact;
    Contact                             m_LeftContact;

    boost::signal<void (Dashboard*)>    m_Signal;

    virtual void                        updateWheels(const sensor_msgs::JointState::ConstPtr& jointState);
    virtual void                        updateUltrasonic(const nxt_msgs::Range::ConstPtr& range);
    virtual void                        updateColor(const nxt_msgs::Color::ConstPtr& color);
    virtual void                        updateRightContact(const nxt_msgs::Contact::ConstPtr& contact);
    virtual void                        updateLeftContact(const nxt_msgs::Contact::ConstPtr& contact);
}; // class Dashboard


/**
 * The DashboardListener class provides abstract method for
 * listening Dashboard update.
 * <p>
 * @see Dashboard::addListener
 */
class DashboardListener
{
    friend void Dashboard::addListener(DashboardListener *pDashboardListener);
    friend void Dashboard::removeListener(DashboardListener *pDashboardListener);
public:
    DashboardListener() { ; }
    virtual ~DashboardListener() { ; }

    /**
     * This method is called when a Dashboard field is update.
     * @param pDashboard a pointer to the Dashboard just updated.
     * @see   Dashboard
     */
    virtual void onDashboardUpdate(Dashboard * pDashboard) = 0;

private:
    boost::signals::connection m_Connection;

    void setConnection(const boost::signals::connection & c);
    void disconnect();
};

} // namespace RemoteAdventurer

#endif // RA_DASHBOARD_H
