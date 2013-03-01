#include "nxt_adventurer/RA_XMLHelper.h"

#define COEF_PERCENT 100.0f

using namespace RemoteAdventurer;

/* ------------------------ XMLWheelHelper ------------------------ */

bool XMLWheelHelper::buildNode(const Wheel & wheel, pugi::xml_node & node)
{
    m_NameNode = node.append_child("name");
    m_NameNode.append_attribute("value") = wheel.getName().c_str();

    m_EffortNode = node.append_child("effort");
    m_EffortNode.append_attribute("value") = wheel.getEffort() * COEF_PERCENT;

    m_PositionNode = node.append_child("position");
    m_PositionNode.append_attribute("value") = wheel.getPosition() * COEF_PERCENT;

    m_VelocityNode = node.append_child("velocity");
    m_VelocityNode.append_attribute("value") = wheel.getVelocity() * COEF_PERCENT;

    return true;
}

RA_ERROR XMLWheelHelper::buildWheel(const pugi::xml_node &node, Wheel &wheel)
{
    wheel.setName(node.child("name").attribute("value").value());
    wheel.setEffort(node.child("effort").attribute("value").as_int() / COEF_PERCENT);
    wheel.setPosition(node.child("position").attribute("value").as_int() / COEF_PERCENT);
    wheel.setVelocity(node.child("velocity").attribute("value").as_int() / COEF_PERCENT);

    return RA_SUCCESS;
}

/* ------------------------ XMLUltrasonicHelper ------------------------ */

bool XMLUltrasonicHelper::buildNode(const Ultrasonic &ultrasonic, pugi::xml_node &node)
{
    m_RangeNode = node.append_child("range");
    m_RangeNode.append_attribute("value") = ultrasonic.getRange() * COEF_PERCENT;

    m_RangeNode = node.append_child("rangeMin");
    m_RangeNode.append_attribute("value") = ultrasonic.getRangeMin() * COEF_PERCENT;

    m_RangeNode = node.append_child("rangeMax");
    m_RangeNode.append_attribute("value") = ultrasonic.getRangeMax() * COEF_PERCENT;

    m_RangeNode = node.append_child("spreadAngle");
    m_RangeNode.append_attribute("value") = ultrasonic.getSpreadAngle() * COEF_PERCENT;

    return true;
}

RA_ERROR XMLUltrasonicHelper::buildUltrasonic(const pugi::xml_node &node, Ultrasonic &ultrasonic)
{
    ultrasonic.setRange(node.child("range").attribute("value").as_int() / COEF_PERCENT);
    ultrasonic.setRangeMin(node.child("rangeMin").attribute("value").as_int() / COEF_PERCENT);
    ultrasonic.setRangeMax(node.child("rangeMax").attribute("value").as_int() / COEF_PERCENT);
    ultrasonic.setSpreadAngle(node.child("spreadAngle").attribute("value").as_int() / COEF_PERCENT);

    return RA_SUCCESS;
}

/* ------------------------ XMLColorHelper ------------------------ */

bool XMLColorHelper::buildNode(const Color &color, pugi::xml_node &node)
{
    m_IntensityNode = node.append_child("intensity");
    m_IntensityNode.append_attribute("value") = color.getIntensity() * COEF_PERCENT;

    m_RedNode = node.append_child("red");
    m_RedNode.append_attribute("value") = color.getRed() * COEF_PERCENT;

    m_GreenNode = node.append_child("green");
    m_GreenNode.append_attribute("value") = color.getGreen() * COEF_PERCENT;

    m_BlueNode = node.append_child("blue");
    m_BlueNode.append_attribute("value") = color.getBlue() * COEF_PERCENT;

    return true;
}

RA_ERROR XMLColorHelper::buildColor(const pugi::xml_node &node, Color &color)
{
    color.setIntensity(node.child("intensity").attribute("value").as_int() / COEF_PERCENT);
    color.setRed(node.child("red").attribute("value").as_int() / COEF_PERCENT);
    color.setGreen(node.child("green").attribute("value").as_int() / COEF_PERCENT);
    color.setBlue(node.child("blue").attribute("value").as_int() / COEF_PERCENT);

    return RA_SUCCESS;
}

/* ------------------------ XMLContactHelper ------------------------ */

bool XMLContactHelper::buildNode(const Contact &contact, pugi::xml_node &node)
{
    if (contact.getContact())
        node.append_attribute("value") = "true";
    else
        node.append_attribute("value") = "false";

    return true;
}

RA_ERROR XMLContactHelper::buildContact(const pugi::xml_node &node, Contact &contact)
{
    contact.setContact(!strcmp(node.attribute("value").value(), "true"));
    return RA_SUCCESS;
}

/* ------------------------ XMLDashboardHelper ------------------------ */

RA_ERROR XMLDashboardHelper::load(const std::string & sXML, Dashboard & dashboard)
{
    RA_ERROR                wRet = RA_FAIL;
    pugi::xml_parse_result  result = m_Document.load(sXML.c_str());

    if (result)
    {
        wRet = RA_SUCCESS;
        pugi::xml_node  mainNode    = m_Document.first_child();
        pugi::xml_node  tempNode;
        Wheel           tempWheel;
        Ultrasonic      tempUltrasonic;
        Color           tempColor;
        Contact         tempContact;

        // getting Wheels informations
        tempNode    = mainNode.child("wheels");

        if (ISOK(wRet) && ISOK(wRet = m_Wheel.buildWheel(tempNode.find_child_by_attribute("wheel", "type", "right"), tempWheel)))
            dashboard.setRightWheel(tempWheel);
        if (ISOK(wRet) && ISOK(wRet = m_Wheel.buildWheel(tempNode.find_child_by_attribute("wheel", "type", "left"), tempWheel)))
            dashboard.setLeftWheel(tempWheel);
        if (ISOK(wRet) && ISOK(wRet = m_Wheel.buildWheel(tempNode.find_child_by_attribute("wheel", "type", "aux"), tempWheel)))
            dashboard.setAuxWheel(tempWheel);

        // getting ultrasonic informations
        tempNode = mainNode.child("ultrasonic");

        if (ISOK(wRet) && ISOK(wRet = m_Ultrasonic.buildUltrasonic(tempNode, tempUltrasonic)))
            dashboard.setUltrasonic(tempUltrasonic);

        // getting color informations
        tempNode = mainNode.child("color");

        if (ISOK(wRet) && ISOK(wRet = m_Color.buildColor(tempNode, tempColor)))
            dashboard.setColor(tempColor);

        // getting contacts information
        tempNode = mainNode.child("contacts");

        if (ISOK(wRet) && ISOK(wRet = m_Contact.buildContact(tempNode.find_child_by_attribute("contact", "type", "right"), tempContact)))
            dashboard.setRightContact(tempContact);

        if (ISOK(wRet) && ISOK(wRet = m_Contact.buildContact(tempNode.find_child_by_attribute("contact", "type", "left"), tempContact)))
            dashboard.setLeftContact(tempContact);
    }
    else
    {
        std::cout << "XML [" << sXML << "] parsed with errors, attr value: [" << m_Document.child("node").attribute("attr").value() << "]\n";
        std::cout << "Error description: " << result.description() << "\n";
        std::cout << "Error offset: " << result.offset << " (error at [...]\n\n";
    }

    return wRet;
}

bool XMLDashboardHelper::genXMLString(const Dashboard &dashboard, std::string &sXML)
{
    m_Document.reset();
    m_StringWriter.XMLText.clear();

    pugi::xml_node  mainNode            = m_Document.append_child("dashboard");

    pugi::xml_node  wheelsNode          = mainNode.append_child("wheels");
    pugi::xml_node  rightWheelNode      = wheelsNode.append_child("wheel");
    pugi::xml_node  leftWheelNode       = wheelsNode.append_child("wheel");
    pugi::xml_node  auxWheelNode        = wheelsNode.append_child("wheel");

    pugi::xml_node  ultrasonicNode      = mainNode.append_child("ultrasonic");
    pugi::xml_node  colorNode           = mainNode.append_child("color");

    pugi::xml_node  contactsNode        = mainNode.append_child("contacts");
    pugi::xml_node  rightContactNode    = contactsNode.append_child("contact");
    pugi::xml_node  leftContactNode     = contactsNode.append_child("contact");

    // building wheels informations
    rightWheelNode.append_attribute("type") = "right";
    m_Wheel.buildNode(dashboard.getRightWheel(), rightWheelNode);

    leftWheelNode.append_attribute("type") = "left";
    m_Wheel.buildNode(dashboard.getLeftWheel(), leftWheelNode);

    auxWheelNode.append_attribute("type") = "aux";
    m_Wheel.buildNode(dashboard.getAuxWheel(), auxWheelNode);

    // building ultrasonic informations
    m_Ultrasonic.buildNode(dashboard.getUltrasonic(), ultrasonicNode);
    // building color informations
    m_Color.buildNode(dashboard.getColor(), colorNode);

    // building contacts informations
    rightContactNode.append_attribute("type") = "right";
    m_Contact.buildNode(dashboard.getRightContact(), rightContactNode);

    leftContactNode.append_attribute("type") = "left";
    m_Contact.buildNode(dashboard.getLeftContact(), leftContactNode);

    m_Document.save(m_StringWriter);
    sXML = m_StringWriter.XMLText;

    return true;
}

/* ------------------------ XMLOrderHelper ------------------------ */

RA_ERROR XMLOrderHelper::load(const std::string &sXML, nxt_adventurer::Order &order)
{
    RA_ERROR    wRet = RA_SUCCESS;
    m_Document.load(sXML.c_str());


    pugi::xml_node  mainNode = m_Document.first_child();

    order.order = mainNode.child("order").attribute("value").as_int();
    order.effort = mainNode.child("effort").attribute("value").as_double() / COEF_PERCENT;

    if (ISOK(wRet))
    {
        order.direction = !strcmp(mainNode.child("direction").attribute("value").value(), "true");
    }


    return wRet;
}

bool XMLOrderHelper::genXMLString(const nxt_adventurer::Order &order, std::string &sXML)
{
    m_Document.load("<?xml version=\"1.0\" ?>");
    pugi::xml_node mainNode = m_Document.append_child("order");

    mainNode.append_child("order").append_attribute("value") = order.order;
    mainNode.append_child("effort").append_attribute("value") = order.effort;

    if (order.direction)
        mainNode.append_child("direction").append_attribute("value") = "true";
    else
        mainNode.append_child("direction").append_attribute("value") = "false";

    m_Document.save(m_StringWriter);
    sXML = m_StringWriter.XMLText;
    m_StringWriter.XMLText.clear();

    return true;
}
