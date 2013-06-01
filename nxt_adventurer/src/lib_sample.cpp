#include <nxt_adventurer/RA_Dashboard.h>
#include <nxt_adventurer/RA_XMLHelper.h>

using namespace RemoteAdventurer;

// first we create a class wich inherit from DashboardListener alowing her to subscribe to a Dashboard.
class DashboardViewer : public DashboardListener
{
 public:
    DashboardViewer() { ; }
    virtual ~DashboardViewer() { ; }

    // we override the onDashboardUpdate to catch Dashboard update
    virtual void onDashboardUpdate(Dashboard * pDashboard)
    {
        // the compare method incate if differences exist between the updated dashboard and the last receive dashboard
        if(pDashboard && !m_LastDash.compare(*pDashboard))
        {
            // for this example we just print dashboard informations on console
            printWheel(pDashboard->getRightWheel());
            printWheel(pDashboard->getLeftWheel());
            printWheel(pDashboard->getAuxWheel());
            printUltrasonic(pDashboard->getUltrasonic());
            printColor(pDashboard->getColor());

            std::cout << "Right contact :" << std::endl << "\t";
            printContact(pDashboard->getRightContact());

            std::cout << "Left contact :" << std::endl << "\t";
            printContact(pDashboard->getLeftContact());

            // we store the new dashboard
            m_LastDash = *pDashboard;
        }
    }

private:
    // the last dashboard received
    Dashboard   m_LastDash;

    // print motor informations
    void printWheel(const Wheel & wheel)
    {
        std::cout << "Wheel : " << wheel.getName() << std::endl;
        std::cout << "Effort : " << wheel.getEffort() << " ";
        std::cout << " Position : " << wheel.getPosition() << " ";
        std::cout << " Velocity : " << wheel.getVelocity() << " " << std::endl;
    }

    // print ultrasonic sensor informations
    void printUltrasonic(const Ultrasonic & ultrasonic)
    {
        std::cout << "Ultrasonic - SpreadAngle = " << ultrasonic.getSpreadAngle() << std::endl;
        std::cout << "Min : " << ultrasonic.getRangeMin() << " < Range : " << ultrasonic.getRange() << " < Max : " << ultrasonic.getRangeMax();
    }

    // print color sensor informations
    void printColor(const Color & color)
    {
        std::cout << "Color : " << "R : " << color.getRed() << " G : " << color.getGreen() << " B : " << color.getBlue() << std::endl;
    }

    // print touch sensor informations
    void printContact(const Contact & contact)
    {
        if (contact.getContact())
            std::cout << "There is contact" << std::endl;
        else
            std::cout << "There is no contact" << std::endl;
    }

};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "lib_sample");
    ros::NodeHandle nh;

    // we create a factory
    // this factory is able to connect dashboard to topics created by nxt_ros node
    NXTWatcherFactory   watcherFactory;

    // then we create a dashboard
    Dashboard           dashboard;
    // finaly a DashboardViewer
    DashboardViewer     dashboardViewer;

    // we connect dashboard to topics using connect method
    watcherFactory.connect(nh, &dashboard);

    // and we add our dashboardViewer to dashboard listeners
    dashboard.addListener(&dashboardViewer);

    // we let ros manage thread for us
    ros::spin();
    return 0;
}
