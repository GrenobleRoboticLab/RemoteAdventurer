#ifndef RA_XMLHELPER_H
#define RA_XMLHELPER_H

#include <cstdlib>

#include <pugixml/pugixml.hpp>
#include "RA_Dashboard.h"
#include <nxt_adventurer/Order.h>

namespace RemoteAdventurer
{

/**
 * The RA_ERROR enum provides some error code.
 * You can use it as return type to specify error type
 */
enum RA_ERROR
{
    RA_SUCCESS,
    RA_FAIL,
    RA_OVERFLOW,
    RA_UNDERFLOW,
    RA_INCONVERTIBLE
};

/**
 * ISOK inline method return true if the error code is equals to RA_SUCCESS.
 */
inline bool ISOK(RA_ERROR error) { return error == RA_SUCCESS; }

/**
 * The CSTRTOD inline method allow you to convert c string to float value.
 * @param sValue the string to be parse
 * @param fValue the float witch receive parsed value.
 * @return if convertion failed the return value is RA_INCONVERTIBLE otherwise the return value is RA_SUCCESS.
 */
inline RA_ERROR CSTRTOD(char const * sValue, float & fValue)
{
    RA_ERROR wRet = RA_FAIL;
    char*    end;
    float    f;

    f = strtof(sValue, &end);

    if (!*end)
        wRet = RA_INCONVERTIBLE;
    else wRet = RA_SUCCESS;

    if(ISOK(wRet))
        fValue = f;

    return wRet;
}

/**
 * The XMLStringWriter provide way to save pugi::xml_document into string variable.
 */
struct XMLStringWriter: pugi::xml_writer
{
    std::string XMLText;

    virtual void write(const void* data, size_t size)
    {
        XMLText += std::string(static_cast<const char*>(data), size);
    }
};

/**
 * The XMLWheelHelper class provide tools for generating and parsing pugi::xml_node from/to a Wheel object.
 */
class XMLWheelHelper
{
private:
    // non copyable object
    XMLWheelHelper(const XMLWheelHelper &);
    const XMLWheelHelper & operator =(const XMLWheelHelper &);

public:
    XMLWheelHelper() { ; }
    virtual ~XMLWheelHelper() { ; }

    /**
     * This method build a pugi::xml_node from a wheel object.
     */
    bool                    buildNode(const Wheel & wheel, pugi::xml_node & node);
    /**
     * This method build a Wheel from a pugi::xml_node object.
     */
    RA_ERROR                buildWheel(const pugi::xml_node & node, Wheel & wheel);

private:
    pugi::xml_node          m_NameNode;
    pugi::xml_node          m_EffortNode;
    pugi::xml_node          m_PositionNode;
    pugi::xml_node          m_VelocityNode;
}; // class XMLWheelHelper

/**
 * The XMLUltrasonicHelper class provide tools for generating and parsing pugi::xml_node from/to an Ultrasonic object.
 */
class XMLUltrasonicHelper
{
private:
    // non copyable object
    XMLUltrasonicHelper(const XMLUltrasonicHelper&);
    const XMLUltrasonicHelper & operator=(const XMLUltrasonicHelper &);

public:
    XMLUltrasonicHelper() { ; }
    virtual ~XMLUltrasonicHelper() { ; }

    /**
     * This method build a pugi::xml_node from an Ultrasonic object.
     */
    bool                    buildNode(const Ultrasonic & ultrasonic, pugi::xml_node & node);
    /**
     * This method build an Ultrasonic from a pugi::xml_node object.
     */
    RA_ERROR                buildUltrasonic(const pugi::xml_node & node, Ultrasonic & ultrasonic);

private:
    pugi::xml_node          m_RangeNode;
    pugi::xml_node          m_RangeMinNode;
    pugi::xml_node          m_RangeMaxNode;
    pugi::xml_node          m_SpreadAngleNode;
}; // class XMLUltrasonicHelper

/**
 * The XMLColorHelper class provide tools for generating and parsing pugi::xml_node from/to a Color object.
 */
class XMLColorHelper
{
private:
    // non copyable object
    XMLColorHelper(const XMLColorHelper &);
    const XMLColorHelper & operator=(const XMLColorHelper &);

public:
    XMLColorHelper() { ; }
    virtual ~XMLColorHelper() { ; }

    /**
     * This method build a pugi::xml_node from a Color object.
     */
    bool                    buildNode(const Color & color, pugi::xml_node & node);
    /**
     * This method build a Color from a pugi::xml_node object.
     */
    RA_ERROR                buildColor(const pugi::xml_node & node, Color & color);

private:
    pugi::xml_node          m_IntensityNode;
    pugi::xml_node          m_RedNode;
    pugi::xml_node          m_GreenNode;
    pugi::xml_node          m_BlueNode;

}; // class XMLColorHelper

/**
 * The XMLColorHelper class provide tools for generating and parsing pugi::xml_node from/to a Contact object.
 */
class XMLContactHelper
{
private:
    // non copyable object
    XMLContactHelper(const XMLContactHelper &);
    const XMLContactHelper & operator=(const XMLContactHelper &);

public:
    XMLContactHelper() { ; }
    virtual ~XMLContactHelper() { ; }

    /**
     * This method build a pugi::xml_node from a Contact object.
     */
    bool                    buildNode(const Contact & contact, pugi::xml_node & node);
    /**
     * This method build a Contact from a pugi::xml_node object.
     */
    RA_ERROR                buildContact(const pugi::xml_node & node, Contact & contact);

}; // class XMLContactHelper

/**
 * The XMLDashboardHelper class provides tools for generate and parse XML from a Dashboard.
 */
class XMLDashboardHelper
{
private:
    // non copyable object
    XMLDashboardHelper(const XMLDashboardHelper &);
    const XMLDashboardHelper & operator =(const XMLDashboardHelper &);

public:
    XMLDashboardHelper() { ; }
    virtual ~XMLDashboardHelper() { ; }

    /**
     * This method buid a Dashboard from a XML string.
     * @param sXML the input XML string.
     * @param dashboard the output DashBoard.
     * @return if the method succeed, the return value is RA_SUCCESS.
     */
    RA_ERROR                load(const std::string & sXML, Dashboard & dashboard);
    /**
     * This method build a XML tree from Dashboard.
     * @param dashboard the input Dashboard
     * @param sXML the output XML string
     * @return if the method succeed the return value is true.
     */
    bool                    genXMLString(const Dashboard & dashboard, std::string & sXML);

private:
    XMLWheelHelper          m_Wheel;
    XMLUltrasonicHelper     m_Ultrasonic;
    XMLColorHelper          m_Color;
    XMLContactHelper        m_Contact;

    pugi::xml_document      m_Document;
    XMLStringWriter         m_StringWriter;
}; // class XMLDashboardHelper

/**
 * The XMLOrderHelper class provides tools for generate and parse XML from a nxt_adventurer::Order.
 */
class XMLOrderHelper
{
private:
    // non copyable object
    XMLOrderHelper(const XMLOrderHelper&);
    const XMLOrderHelper& operator=(const XMLOrderHelper&);

public:
    XMLOrderHelper() { ; }
    virtual ~XMLOrderHelper() { ; }

    /**
     * This method buid a nxt_adventurer::Order from a XML string.
     * @param sXML the input XML string.
     * @param order the output nxt_adventurer::Order.
     * @return if the method succeed, the return value is RA_SUCCESS.
     */
    RA_ERROR            load(const std::string & sXML, nxt_adventurer::Order & order);
    /**
     * This method build a XML tree from nxt_adventurer::Order.
     * @param order the input nxt_adventurer::Order
     * @param sXML the output XML string
     * @return if the method succeed the return value is true.
     */
    bool                genXMLString(const nxt_adventurer::Order & order, std::string & sXML);

private:
    pugi::xml_document  m_Document;
    XMLStringWriter     m_StringWriter;

}; // class XMLOrderHelper

} // namespace RemoteAdventurer

#endif // RA_XMLHELPER_H
