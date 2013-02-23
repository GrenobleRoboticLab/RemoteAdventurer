#include "remote_adventurer_client/RA_MainWindow.h"

using namespace RemoteAdventurerCLient;

WheelViewer::WheelViewer(QWidget *parent) : QVBoxLayout(parent)
{
    m_pWidget           = new QWidget;
    m_pName             = new QLabel("Name");
    m_pEffortValue      = new QLabel("Effort");
    m_pVelocityValue    = new QLabel("Velocity");
    m_pPositionValue    = new QLabel("Position");

    setAlignment(Qt::AlignCenter);
    m_pWidget->setLayout(this);

    addWidget(m_pName);
    addWidget(m_pEffortValue);
    addWidget(m_pVelocityValue);
    addWidget(m_pPositionValue);
}

void WheelViewer::update(const Wheel &wheel)
{
    m_pName->setText(QString(wheel.getName().c_str()));
    m_pEffortValue->setText(QString::number(wheel.getEffort()));
    m_pPositionValue->setText(QString::number(wheel.getPosition()));
    m_pVelocityValue->setText(QString::number(wheel.getVelocity()));
}

void WheelViewer::release()
{
    releasePosition();
    releaseVelocity();
    releaseEffort();
    releaseName();
    releaseWidget();
}

void WheelViewer::releasePosition()
{
    if(m_pPositionValue)
        delete m_pPositionValue;
    m_pPositionValue = NULL;
}

void WheelViewer::releaseVelocity()
{
    if(m_pVelocityValue)
        delete m_pVelocityValue;
    m_pVelocityValue = NULL;
}

void WheelViewer::releaseEffort()
{
    if(m_pEffortValue)
        delete m_pEffortValue;
    m_pEffortValue = NULL;
}

void WheelViewer::releaseName()
{
    if(m_pName)
        delete m_pName;
    m_pName = NULL;
}

void WheelViewer::releaseWidget()
{
    if(m_pWidget)
        delete m_pWidget;
    m_pWidget = NULL;
}

ColorViewer::ColorViewer(QWidget * parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    m_pal.setColor(QPalette::Background, QColor(255, 255, 255));
    setPalette(m_pal);
    setMinimumSize(100, 50);
    setMaximumHeight(200);
}

void ColorViewer::update(const Color &color)
{
    m_pal.setColor(QPalette::Background, QColor(255 * color.getRed(), 255 * color.getGreen(), 255 * color.getBlue()));
    setPalette(m_pal);
}

ContactViewer::ContactViewer(QWidget * parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    m_pal.setColor(QPalette::Background, QColor(0, 255, 0));
    setPalette(m_pal);
    setMinimumSize(50, 50);
    setMaximumHeight(100);
}

void ContactViewer::update(const Contact &contact)
{
    if (contact.getContact())
        m_pal.setColor(QPalette::Background, QColor(255, 0, 0));
    else
        m_pal.setColor(QPalette::Background, QColor(0, 255, 0));
    setPalette(m_pal);
}

UltrasonicViewer::UltrasonicViewer(QWidget *parent) : QVBoxLayout(parent)
{

}

RobotViewer::RobotViewer(QWidget *parent) : QGridLayout(parent)
{
    m_pUltrasonic   = NULL;
    m_pWidget       = new QWidget;
    m_pLeftWheel    = new WheelViewer;
    m_pRightWheel   = new WheelViewer;
    m_pAuxWheel     = new WheelViewer;
    m_pColor        = new ColorViewer;
    m_pRightContact = new ContactViewer;
    m_pLeftContact  = new ContactViewer;

    m_pWidget->setLayout(this);

    if(m_pRightContact)
        addWidget(m_pRightContact, 0, 4);
    if(m_pLeftContact)
        addWidget(m_pLeftContact, 0, 0);
    if(m_pColor)
        addWidget(m_pColor, 1, 1, 1, 2);

    if (m_pLeftWheel)
        addWidget(m_pLeftWheel->getWidget(), 1, 0, 2, 1);
    if (m_pRightWheel)
        addWidget(m_pRightWheel->getWidget(), 1, 4, 2, 1);
    if (m_pAuxWheel)
        addWidget(m_pAuxWheel->getWidget(), 2, 1, 1, 2);
}

void RobotViewer::update(const Dashboard &dashboard)
{
    if (m_pRightWheel)
        m_pRightWheel->update(dashboard.getRightWheel());
    if (m_pAuxWheel)
        m_pAuxWheel->update(dashboard.getAuxWheel());
    if (m_pLeftWheel)
        m_pLeftWheel->update(dashboard.getLeftWheel());

    if(m_pRightContact)
        m_pRightContact->update(dashboard.getRightContact());
    if(m_pLeftContact)
        m_pLeftContact->update(dashboard.getLeftContact());
    if(m_pColor)
        m_pColor->update(dashboard.getColor());

    m_pWidget->repaint();
}

void RobotViewer::Release()
{
    ReleaseRightWheel();
    ReleaseLeftWheel();
    ReleaseAuxWheel();
    ReleaseColor();
    ReleaseRightContact();
    ReleaseLeftContact();
}

void RobotViewer::ReleaseRightWheel()
{
    if(m_pRightWheel)
        delete m_pRightWheel;
    m_pRightWheel = NULL;
}

void RobotViewer::ReleaseLeftWheel()
{
    if(m_pLeftWheel)
        delete m_pLeftWheel;
    m_pLeftWheel = NULL;
}

void RobotViewer::ReleaseAuxWheel()
{
    if(m_pAuxWheel)
        delete m_pAuxWheel;
    m_pAuxWheel = NULL;
}

void RobotViewer::ReleaseColor()
{
    if(m_pColor)
        delete m_pColor;
    m_pColor = NULL;
}

void RobotViewer::ReleaseRightContact()
{
    if(m_pRightContact)
        delete m_pRightContact;
    m_pRightContact = NULL;
}

void RobotViewer::ReleaseLeftContact()
{
    if(m_pLeftContact)
        delete m_pLeftContact;
    m_pLeftContact = NULL;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    m_pRobotViewer  = new RobotViewer;

    if(m_pRobotViewer)
        setCentralWidget(m_pRobotViewer->getWidget());

    setMinimumSize(500, 450);
}

void MainWindow::Release()
{
    ReleaseRobotViewer();
    ReleaseWidget();
}

void MainWindow::ReleaseRobotViewer()
{
    if (m_pRobotViewer)
        delete m_pRobotViewer;
    m_pRobotViewer = NULL;
}

void MainWindow::ReleaseWidget()
{
    if(m_pWidget)
        delete m_pWidget;
    m_pWidget = NULL;
}
