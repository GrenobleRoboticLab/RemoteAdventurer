#include "remote_adventurer_client/RA_UltrasonicViewer.h"

using namespace RemoteAdventurerClient;

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

UltrasonicViewer::~UltrasonicViewer()
{
    std::cout << "Destroying UltrasonicViewer" << std::endl;
    release();
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
