#include "remote_adventurer_client/RA_MainWindow.h"
#include <QtAlgorithms>

using namespace RemoteAdventurerCLient;

WheelViewer::WheelViewer(QWidget *parent) : QVBoxLayout(parent)
{
    m_pName             = new QLabel("Name");
    m_pEffortValue      = new QLabel("Effort");
    m_pVelocityValue    = new QLabel("Velocity");
    m_pPositionValue    = new QLabel("Position");

    QPalette pal;
    pal.setColor(QPalette::Foreground, QColor(255, 255, 255));

    setAlignment(Qt::AlignCenter);

    if (m_pName)
    {
        m_pName->setPalette(pal);
        addWidget(m_pName);
    }

    if (m_pEffortValue)
    {
        m_pEffortValue->setPalette(pal);
        addWidget(m_pEffortValue);
    }

    if (m_pVelocityValue)
    {
        m_pVelocityValue->setPalette(pal);
        addWidget(m_pVelocityValue);
    }

    if (m_pPositionValue)
    {
        m_pPositionValue->setPalette(pal);
        addWidget(m_pPositionValue);
    }
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

UltrasonicViewer::UltrasonicViewer(QWidget *parent) : QGraphicsView(parent)
{
    m_pBoundedRange = NULL;
    m_pPolyRange    = NULL;

    m_pTextMinrange = NULL;
    m_pTextMaxrange = NULL;
    m_pTextRange    = NULL;

    m_pScene        = new QGraphicsScene;

    setBackgroundBrush(Qt::black);
    setScene(m_pScene);

    m_pTextMaxrange = m_pScene->addText("Max range : ");
    m_pTextMinrange = m_pScene->addText("Min range : ");
    m_pTextRange = m_pScene->addText("Range : ");

    m_pTextMaxrange->setDefaultTextColor(Qt::white);
    m_pTextRange->setDefaultTextColor(Qt::white);
    m_pTextMinrange->setDefaultTextColor(Qt::white);

    QPolygonF poly;
    m_LinearGradient.setColorAt(0.0, QColor(53, 246, 160));
    m_LinearGradient.setColorAt(1.0, QColor(6, 141, 81));
    m_pBoundedRange = m_pScene->addPolygon(poly, QPen(QColor(73, 73, 73), 5), QBrush(QColor(73, 73, 73)));
    m_pPolyRange = m_pScene->addPolygon(poly, QPen(), QBrush(m_LinearGradient));
}

void UltrasonicViewer::update(const Ultrasonic &ultrasonic)
{
    if (!m_LastUltrasonic.compare(ultrasonic))
        m_LastUltrasonic = ultrasonic;

    double xMid = this->size().width() / 2.0;
    double dAd = computeAd(m_LastUltrasonic.getRange(), m_LastUltrasonic.getRangeMax());

    QPolygonF boundedPoly;
    boundedPoly << QPointF(xMid, this->size().height())
                << QPointF(xMid - computeOp(m_LastUltrasonic.getSpreadAngle(), this->size().height()), 0)
                << QPointF(xMid + computeOp(m_LastUltrasonic.getSpreadAngle(), this->size().height()), 0);
    m_pBoundedRange->setPolygon(boundedPoly);

    QPolygonF rangePoly;
    rangePoly   << QPointF(xMid, this->size().height())
                << QPointF(xMid - computeOp(m_LastUltrasonic.getSpreadAngle(), dAd), this->size().height() - dAd)
                << QPointF(xMid + computeOp(m_LastUltrasonic.getSpreadAngle(), dAd), this->size().height() - dAd);
    m_pPolyRange->setPolygon(rangePoly);
    m_LinearGradient.setStart(xMid, this->size().height());
    m_LinearGradient.setFinalStop(xMid, 0);
    m_pPolyRange->setBrush(QBrush(m_LinearGradient));


    m_pTextRange->setPos(xMid + computeOp(m_LastUltrasonic.getSpreadAngle(), dAd), this->size().height() - dAd);
    m_pTextRange->setPlainText(QString("Range : ") + QString::number(m_LastUltrasonic.getRange()) + QString("m"));
    m_pTextMaxrange->setPlainText(QString("Max range : ") + QString::number(m_LastUltrasonic.getRangeMax()) + QString("m"));
    m_pTextMinrange->setPlainText(QString("Min range : ") + QString::number(m_LastUltrasonic.getRangeMin()) + QString("m"));
}

void UltrasonicViewer::resizeEvent(QResizeEvent *event)
{
    QSize size = event->size();
    m_pScene->setSceneRect(0, 0, size.width(), size.height());
    m_pTextMinrange->setPos(0, size.height()-25);
    m_pTextRange->setPos(size.width() / 2, size.height() / 2);
    update(m_LastUltrasonic);
}

double UltrasonicViewer::computeAd(double dValue, double dMaxValue)
{
    if (dMaxValue > 0)
        return (dValue * this->size().height()) / dMaxValue;
    return 0.0;
}

double UltrasonicViewer::computeOp(double dAngle, double dAd)
{
    return (dAd * tan(dAngle));
}

void UltrasonicViewer::release()
{
    setScene(NULL);
    releaseScene();
}

void UltrasonicViewer::releaseScene()
{
    if (m_pScene)
        delete m_pScene;
    m_pScene        = NULL;
    m_pBoundedRange = NULL;
    m_pPolyRange    = NULL;
    m_pTextMinrange = NULL;
    m_pTextMaxrange = NULL;
    m_pTextRange    = NULL;
}

RobotViewer::RobotViewer(QWidget *parent) : QGridLayout(parent)
{
    m_pUltrasonic   = new UltrasonicViewer;
    m_pLeftWheel    = new WheelViewer;
    m_pRightWheel   = new WheelViewer;
    m_pAuxWheel     = new WheelViewer;
    m_pColor        = new ColorViewer;
    m_pRightContact = new ContactViewer;
    m_pLeftContact  = new ContactViewer;

    m_pRWheelMeter   = new WheelView;
    m_pLWheelMeter   = new WheelView;
    m_pAWheelMeter   = new WheelView;

    if (m_pUltrasonic)
        addWidget(m_pUltrasonic, 0, 0, 2, 6);

    if(m_pLeftContact)
        addWidget(m_pLeftContact, 2, 0, 1, 1);

    if(m_pColor)
        addWidget(m_pColor, 2, 1, 1, 4);

    if(m_pRightContact)
        addWidget(m_pRightContact, 2, 5, 1, 1);

    if (m_pLeftWheel)
    {
        m_LWWidget.setLayout(m_pLeftWheel);
        addWidget(&m_LWWidget, 3, 0, 1, 2);
    }

    if (m_pAuxWheel)
    {
        m_AWWidget.setLayout(m_pAuxWheel);
        addWidget(&m_AWWidget, 3, 2, 1, 2);
    }

    if (m_pRightWheel)
    {
        m_RWWidget.setLayout(m_pRightWheel);
        addWidget(&m_RWWidget, 3, 4, 1, 2);
    }

    if(m_pRWheelMeter)
        addWidget(m_pRWheelMeter, 4, 4, 1, 2);

    if(m_pLWheelMeter)
        addWidget(m_pLWheelMeter, 4, 0, 1, 2);

    if(m_pAWheelMeter)
        addWidget(m_pAWheelMeter, 4, 2, 1, 2);
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

void RobotViewer::Release()
{
    ReleaseRightWheel();
    ReleaseLeftWheel();
    ReleaseAuxWheel();
    ReleaseColor();
    ReleaseRightContact();
    ReleaseLeftContact();
    ReleaseUltrasonic();
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

void RobotViewer::ReleaseUltrasonic()
{
    if (m_pUltrasonic)
        delete m_pUltrasonic;
    m_pUltrasonic = NULL;
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

    setMinimumSize(300, 300);

    QPalette pal;
    pal.setColor(QPalette::Background, QColor(73, 73, 73));
    m_Widget.setPalette(pal);
    m_Widget.setAutoFillBackground(true);
}

MainWindow::~MainWindow()
{
    std::cout << "Destroying MainWindow" << std::endl;
    Release();
}

void MainWindow::Release()
{
    ReleaseRobotViewer();
}

void MainWindow::ReleaseRobotViewer()
{
    if (m_pRobotViewer)
        delete m_pRobotViewer;
    m_pRobotViewer = NULL;
}
