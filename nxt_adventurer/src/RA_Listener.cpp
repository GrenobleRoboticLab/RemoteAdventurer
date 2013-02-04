#include "nxt_adventurer/RA_Listener.h"

using namespace RemoteAdventurer;

/* ----------------------------- NXTWatcherFactory ----------------------------- */

NXTWatcherFactory::NXTWatcherFactory(const std::string &sRightWheelName, const std::string &sLeftWheelName, const std::string &sAuxMotorName, const std::string &sUltrasonicName, const std::string &sColorName, const std::string &sRightContactName, const std::string &sLeftContactName)
    : m_sRightWheelName(sRightWheelName), m_sLeftWheelName(sLeftWheelName), m_sAuxWheelName(sAuxMotorName), m_sUltrasonicName(sUltrasonicName), m_sColorName(sColorName), m_sRightContactName(sRightContactName), m_sLeftContactName(sLeftContactName)
{
    m_nCount = 0;
}

NXTWatcherFactory::~NXTWatcherFactory() { m_SubFarms.clear(); }

int NXTWatcherFactory::connect(ros::NodeHandle& nh, NXTLocalWatcher *pWatcher)
{
    if (pWatcher)
    {
        m_nCount++;
        pWatcher->connect(m_nCount, this);
        pWatcher->setRightWheelName(m_sRightWheelName);
        pWatcher->setLeftWheelName(m_sLeftWheelName);
        pWatcher->setAuxWheelName(m_sAuxWheelName);

        SubFarm sb;
        sb.m_nKey = m_nCount;
        sb.m_vMotorSub          = nh.subscribe("joint_state", 5, &NXTLocalWatcher::updateWheels, pWatcher);
        sb.m_vUltrasonicSub     = nh.subscribe(m_sUltrasonicName, 5, &NXTLocalWatcher::updateUltrasonic, pWatcher);
        sb.m_vColorSub          = nh.subscribe(m_sColorName, 5, &NXTLocalWatcher::updateColor, pWatcher);
        sb.m_vLeftContactSub    = nh.subscribe(m_sLeftContactName, 5, &NXTLocalWatcher::updateLeftContact, pWatcher);
        sb.m_vRightContactSub   = nh.subscribe(m_sRightContactName, 5, &NXTLocalWatcher::updateRightContact, pWatcher);

        m_SubFarms.insert(sb);
    }

    return m_nCount;
}

void NXTWatcherFactory::disconnect(int nSubId)
{
    SubFarm search;
    std::set<SubFarm, SupFarmComp>::iterator it;

    search.m_nKey = nSubId;
    it            = m_SubFarms.find(search);

    if (it != m_SubFarms.end())
        m_SubFarms.erase(it);
}

/* ------------------------------ NXTLocalWatcher ------------------------------ */

void NXTLocalWatcher::connect(int nSubId, NXTWatcherFactory *pFactory)
{
    if (pFactory)
    {
        m_nSubFarmId    = nSubId;
        m_pFactory      = pFactory;
    }
}

void NXTLocalWatcher::disconnect()
{
    if (m_pFactory)
        m_pFactory->disconnect(m_nSubFarmId);
}
