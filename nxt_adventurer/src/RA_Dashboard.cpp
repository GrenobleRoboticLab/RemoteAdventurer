#include "nxt_adventurer/RA_Dashboard.h"
#include <boost/bind.hpp>

using namespace RemoteAdventurer;

/* ------------------------ DashboardListener ------------------------ */

void DashboardListener::setConnection(const boost::signals::connection & c)
{
    m_Connection = c;
}

void DashboardListener::disconnect()
{
    if (m_Connection.connected())
        m_Connection.disconnect();
}

/* ------------------------------ Wheel ------------------------------ */

Wheel::Wheel() { reset(); }

Wheel::Wheel(const std::string & sName)
{
    reset();
    m_sName = sName;
}

Wheel::Wheel(const Wheel & wheel) { initFrom(wheel); }

Wheel::~Wheel() { reset(); }

const Wheel& Wheel::operator=(const Wheel & wheel)
{
    initFrom(wheel);
    return *this;
}

bool Wheel::operator==(const Wheel & wheel) { return compare(wheel);  }
bool Wheel::operator!=(const Wheel & wheel) { return !compare(wheel); }

void Wheel::initFrom(const Wheel & wheel)
{
    m_sName     = wheel.m_sName;
    m_fEffort   = wheel.m_fEffort;
    m_fPosition = wheel.m_fPosition;
    m_fVelocity = wheel.m_fVelocity;
}

bool Wheel::compare(const Wheel & wheel)
{
    bool bRet = false;

    if ((m_sName     == wheel.m_sName)
     && (m_fEffort   == wheel.m_fEffort)
     && (m_fPosition == wheel.m_fPosition)
     && (m_fVelocity == wheel.m_fVelocity))
    {
        bRet = true;
    }

    return bRet;
}

bool Wheel::update(const sensor_msgs::JointState::ConstPtr & jointState)
{
    bool bRet = false;

    if (jointState->name.back() == m_sName)
    {
        m_fEffort   = jointState->effort.back();
        m_fPosition = jointState->position.back();
        m_fVelocity = jointState->velocity.back();
        bRet = true;
    }

    return bRet;
}

void Wheel::reset()
{
    m_sName     = "";
    m_fEffort   = 0.0f;
    m_fPosition = 0.0f;
    m_fVelocity = 0.0f;
}

void Wheel::setName(const std::string & sName)
{
    reset();
    m_sName = sName;
}

/* ---------------------------- Ultrasonic ---------------------------- */

Ultrasonic::Ultrasonic() { reset(); }
Ultrasonic::Ultrasonic(const Ultrasonic & ultrasonic) { initFrom(ultrasonic); }

Ultrasonic::~Ultrasonic() { reset(); }

const Ultrasonic& Ultrasonic::operator=(const Ultrasonic & ultrasonic)
{
    initFrom(ultrasonic);
    return *this;
}

bool Ultrasonic::operator==(const Ultrasonic & ultrasonic) { return compare(ultrasonic);  }
bool Ultrasonic::operator!=(const Ultrasonic & ultrasonic) { return !compare(ultrasonic); }

void Ultrasonic::initFrom(const Ultrasonic & ultrasonic)
{
    m_fRange       = ultrasonic.m_fRange;
    m_fRangeMin    = ultrasonic.m_fRangeMin;
    m_fRangeMax    = ultrasonic.m_fRangeMax;
    m_fSpreadAngle = ultrasonic.m_fSpreadAngle;
}

bool Ultrasonic::compare(const Ultrasonic & ultrasonic)
{
    bool bRet = false;

    if ((m_fRange       == ultrasonic.m_fRange)
     && (m_fRangeMin    == ultrasonic.m_fRangeMin)
     && (m_fRangeMax    == ultrasonic.m_fRangeMax)
     && (m_fSpreadAngle == ultrasonic.m_fSpreadAngle))
    {
        bRet = true;
    }

    return bRet;
}

bool Ultrasonic::update(const nxt_msgs::Range::ConstPtr & range)
{
    m_fRange       = range->range;
    m_fRangeMin    = range->range_min;
    m_fRangeMax    = range->range_max;
    m_fSpreadAngle = range->spread_angle;

    return true;
}

void Ultrasonic::reset()
{
    m_fRange       = 0.0f;
    m_fRangeMin    = 0.0f;
    m_fRangeMax    = 0.0f;
    m_fSpreadAngle = 0.0f;
}

/* ----------------------------- Contact ----------------------------- */

Contact::Contact() { reset(); }
Contact::Contact(const Contact & contact) { initFrom(contact); }

Contact::~Contact() { reset(); }

const Contact& Contact::operator=(const Contact & contact)
{
    initFrom(contact);
    return *this;
}

bool Contact::operator==(const Contact & contact) { return compare(contact);  }
bool Contact::operator!=(const Contact & contact) { return !compare(contact); }

void Contact::initFrom(const Contact & contact)   { m_bContact = contact.m_bContact;           }
bool Contact::compare(const Contact & contact)    { return (m_bContact == contact.m_bContact); }

bool Contact::update(const nxt_msgs::Contact::ConstPtr & contact) 
{ 
    m_bContact = contact->contact; 
    return true;
}

void Contact::reset() { m_bContact = false; }

/* ------------------------------ Color ------------------------------ */

Color::Color() { reset(); }
Color::Color(const Color & color) { initFrom(color); }

Color::~Color() { reset(); }

const Color& Color::operator=(const Color & color)
{
    initFrom(color);
    return *this;
}

bool Color::operator==(const Color & color) { return compare(color);  }
bool Color::operator!=(const Color & color) { return !compare(color); }

void Color::initFrom(const Color & color)
{
    m_fIntensity = color.m_fIntensity;
    m_fRed       = color.m_fRed;
    m_fGreen     = color.m_fGreen;
    m_fBlue      = color.m_fBlue;
}

bool Color::compare(const Color & color)
{
    bool bRet = false;
    
    if ((m_fIntensity == color.m_fIntensity)
     && (m_fRed       == color.m_fRed)
     && (m_fGreen     == color.m_fGreen)
     && (m_fBlue      == color.m_fBlue))
    {
        bRet = true;
    }

    return bRet;
}

bool Color::update(const nxt_msgs::Color::ConstPtr& color)
{
    m_fIntensity = color->intensity;
    m_fRed       = color->r;
    m_fGreen     = color->g;
    m_fBlue      = color->b;

    return true;
}

void Color::reset()
{
    m_fIntensity = 0.0f;
    m_fRed       = 0.0f;
    m_fGreen     = 0.0f;
    m_fBlue      = 0.0f;
}

/* ---------------------------- Dashboard ---------------------------- */

Dashboard::Dashboard() { ; }

Dashboard::Dashboard(const Dashboard & dashboard) : NXTLocalWatcher(dashboard) { initFrom(dashboard); }

Dashboard::~Dashboard() { ; }

const Dashboard& Dashboard::operator=(const Dashboard & dashboard)
{
    initFrom(dashboard);
    return *this;
}

bool Dashboard::operator==(const Dashboard & dashboard) { return compare(dashboard);  }
bool Dashboard::operator!=(const Dashboard & dashboard) { return !compare(dashboard); }

void Dashboard::initFrom(const Dashboard & dashboard)
{
    m_RightWheel    = dashboard.m_RightWheel;
    m_LeftWheel     = dashboard.m_LeftWheel;
    m_AuxWheel      = dashboard.m_AuxWheel;
    m_Ultrasonic    = dashboard.m_Ultrasonic;
    m_Color         = dashboard.m_Color;
    m_RightContact  = dashboard.m_RightContact;
    m_LeftContact   = dashboard.m_LeftContact;
}

bool Dashboard::compare(const Dashboard & dashboard)
{
    bool bRet = false;

    if ((m_RightWheel   == dashboard.m_RightWheel)
     && (m_LeftWheel    == dashboard.m_LeftWheel)
     && (m_AuxWheel     == dashboard.m_AuxWheel)
     && (m_Ultrasonic   == dashboard.m_Ultrasonic)
     && (m_Color        == dashboard.m_Color)
     && (m_RightContact == dashboard.m_RightContact)
     && (m_LeftContact  == dashboard.m_LeftContact))
    {
        bRet = true;
    }

    return bRet;
}

void Dashboard::addListener(DashboardListener *pDashboardListener)
{
    if (pDashboardListener)
        pDashboardListener->setConnection(m_Signal.connect(boost::bind(&DashboardListener::onDashboardUpdate, pDashboardListener, _1)));
}

void Dashboard::removeListener(DashboardListener *pDashboardListener)
{
    if (pDashboardListener)
        pDashboardListener->disconnect();
}

void Dashboard::updateWheels(const sensor_msgs::JointState::ConstPtr& jointState)
{
    bool bUpdated = false;

    if (jointState->name.back() == m_RightWheel.getName())
    {
        m_RightWheel.update(jointState);
        bUpdated = true;
    }
    else if (jointState->name.back() == m_LeftWheel.getName())
    {
        m_LeftWheel.update(jointState);
        bUpdated = true;
    }
    else if (jointState->name.back() == m_AuxWheel.getName())
    {
        m_AuxWheel.update(jointState);
        bUpdated = true;
    }

    if (bUpdated)
    {
        m_Signal(this);
        m_LastUpdated = ST_MOTOR;
    }
}

void Dashboard::updateUltrasonic(const nxt_msgs::Range::ConstPtr& range)
{
    m_Ultrasonic.update(range);
    m_Signal(this);
    m_LastUpdated = ST_ULTRASONIC;
}

void Dashboard::updateColor(const nxt_msgs::Color::ConstPtr& color)
{
    m_Color.update(color);
    m_Signal(this);
    m_LastUpdated = ST_COLOR;
}

void Dashboard::updateRightContact(const nxt_msgs::Contact::ConstPtr& contact)
{
    m_RightContact.update(contact);
    m_Signal(this);
    m_LastUpdated = ST_CONTACT;
}

void Dashboard::updateLeftContact(const nxt_msgs::Contact::ConstPtr& contact)
{
    m_LeftContact.update(contact);
    m_Signal(this);
    m_LastUpdated = ST_CONTACT;
}
