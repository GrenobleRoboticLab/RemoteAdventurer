#include "remote_adventurer_client/RA_MainWindow.h"
#include <QtAlgorithms>

using namespace RemoteAdventurerClient;

ColorViewer::ColorViewer(QWidget * parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    m_Pal.setColor(QPalette::Background, QColor(255, 255, 255));
    setPalette(m_Pal);
    setMinimumSize(100, 50);
    setMaximumHeight(200);
}

void ColorViewer::update(const Color &color)
{
    m_Pal.setColor(QPalette::Background, QColor(255 * color.getRed(), 255 * color.getGreen(), 255 * color.getBlue()));
    setPalette(m_Pal);
}

ContactViewer::ContactViewer(QWidget * parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    m_Pal.setColor(QPalette::Background, QColor(6, 141, 41));
    setPalette(m_Pal);
    setMinimumSize(100, 100);
    setMaximumHeight(200);
}

void ContactViewer::update(const Contact &contact)
{
    if (contact.getContact())
        m_Pal.setColor(QPalette::Background, QColor(141, 13, 5));
    else
        m_Pal.setColor(QPalette::Background, QColor(6, 141, 41));
    setPalette(m_Pal);
}

RobotViewer::RobotViewer(QWidget *parent) : QGridLayout(parent)
{
    m_pUltrasonic   = new UltrasonicViewer;
    m_pLeftWheel    = new WheelMeter;
    m_pRightWheel   = new WheelMeter;
    m_pAuxWheel     = new WheelMeter;

    m_pAuxWheel->setStartAngle(150);
    m_pAuxWheel->setSpanAngle(240);
    m_pRightWheel->setStartAngle(150);
    m_pRightWheel->setSpanAngle(240);
    m_pLeftWheel->setStartAngle(150);
    m_pLeftWheel->setSpanAngle(240);

    m_pColor        = new ColorViewer;
    m_pRightContact = new ContactViewer;
    m_pLeftContact  = new ContactViewer;

    m_pController   = new ControllerView;

    if (m_pUltrasonic)
        addWidget(m_pUltrasonic, 0, 0, 2, 6);

    if(m_pLeftContact)
        addWidget(m_pLeftContact, 2, 0, 1, 1);

    if(m_pColor)
        addWidget(m_pColor, 2, 1, 1, 4);

    if(m_pRightContact)
        addWidget(m_pRightContact, 2, 5, 1, 1);

    if (m_pLeftWheel)
        addWidget(m_pLeftWheel, 3, 0, 1, 2);

    if (m_pAuxWheel)
        addWidget(m_pAuxWheel, 3, 2, 1, 2);

    if (m_pRightWheel)
        addWidget(m_pRightWheel, 3, 4, 1, 2);

    if(m_pController)
        addWidget(m_pController, 0, 7, 4, 3);
}

void RobotViewer::update(const Dashboard &dashboard)
{
    if (m_pRightWheel)
        m_pRightWheel->update(dashboard.getRightWheel());
    if (m_pAuxWheel)
        m_pAuxWheel->update(dashboard.getAuxWheel());
    if (m_pLeftWheel)
        m_pLeftWheel->update(dashboard.getLeftWheel());

    if(m_pUltrasonic)
        m_pUltrasonic->update(dashboard.getUltrasonic());

    if(m_pRightContact)
        m_pRightContact->update(dashboard.getRightContact());
    if(m_pLeftContact)
        m_pLeftContact->update(dashboard.getLeftContact());
    if(m_pColor)
        m_pColor->update(dashboard.getColor());
}

void RobotViewer::release()
{
    releaseRightWheel();
    releaseLeftWheel();
    releaseAuxWheel();
    releaseColor();
    releaseRightContact();
    releaseLeftContact();
    releaseUltrasonic();
    releaseController();
}

void RobotViewer::releaseRightWheel()
{
    if(m_pRightWheel)
        delete m_pRightWheel;
    m_pRightWheel = NULL;
}

void RobotViewer::releaseLeftWheel()
{
    if(m_pLeftWheel)
        delete m_pLeftWheel;
    m_pLeftWheel = NULL;
}

void RobotViewer::releaseAuxWheel()
{
    if(m_pAuxWheel)
        delete m_pAuxWheel;
    m_pAuxWheel = NULL;
}

void RobotViewer::releaseColor()
{
    if(m_pColor)
        delete m_pColor;
    m_pColor = NULL;
}

void RobotViewer::releaseRightContact()
{
    if(m_pRightContact)
        delete m_pRightContact;
    m_pRightContact = NULL;
}

void RobotViewer::releaseLeftContact()
{
    if(m_pLeftContact)
        delete m_pLeftContact;
    m_pLeftContact = NULL;
}

void RobotViewer::releaseUltrasonic()
{
    if (m_pUltrasonic)
        delete m_pUltrasonic;
    m_pUltrasonic = NULL;
}

void RobotViewer::releaseController()
{
    if (m_pController)
        delete m_pController;
    m_pController = NULL;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    m_pRobotViewer  = new RobotViewer;

    if(m_pRobotViewer)
    {
        m_Widget.setLayout(m_pRobotViewer);
        setCentralWidget(&m_Widget);
    }

    setMinimumSize(800, 600);

    QPalette pal;
    pal.setColor(QPalette::Background, QColor(73, 73, 73));
    m_Widget.setPalette(pal);
    m_Widget.setAutoFillBackground(true);
}

MainWindow::~MainWindow()
{
    std::cout << "Destroying MainWindow" << std::endl;
    release();
}

void MainWindow::release()
{
    releaseRobotViewer();
}

void MainWindow::releaseRobotViewer()
{
    if (m_pRobotViewer)
        delete m_pRobotViewer;
    m_pRobotViewer = NULL;
}
