#include "remote_adventurer_client/RA_WheelMeter.h"

using namespace RemoteAdventurerCLient;

static const QPoint effortHand[3] = {
    QPoint(1, 2),
    QPoint(-1, 2),
    QPoint(0, -EFFORT_WIDTH)
};

static const QPoint speedHand[3] = {
    QPoint(1, 2),
    QPoint(-1, 2),
    QPoint(0, -SPEED_WIDTH)
};

static double test = SPEED_MAX / 4;

GraphicsWheelMeterItem::GraphicsWheelMeterItem(qreal x, qreal y, qreal width, qreal height, QGraphicsItem *parent) : QGraphicsEllipseItem(x, y, width, height, parent)
{
    m_MaxSpeed          = SPEED_MAX;
    m_MaxEffort         = EFFORT_MAX;
    m_SpeedBrush        = QBrush(QColor(141, 5, 38));
    m_SpeedPen          = QPen(QColor(141, 5, 38), 3);
    m_EffortBrush       = QBrush(QColor(141, 41, 5));
    m_EffortPen         = QPen(QColor(141, 41, 5), 2);
    m_PosDefaultColor   = Qt::white;

    m_Wheel.setEffort(0);
    m_Wheel.setVelocity(0);
}

void GraphicsWheelMeterItem::setArc(qreal x, qreal y, qreal width, qreal height)
{
    setRect(x, y, width, height);
}

void GraphicsWheelMeterItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    int side = qMin(rect().width(), rect().height());

    painter->setRenderHint(QPainter::Antialiasing);
    painter->translate(rect().width() / 2, rect().height() / 2);
    painter->scale(side / 200.0, side / 200.0);

    drawSpeedMeter(painter);
    drawEffortMeter(painter);
}

void GraphicsWheelMeterItem::drawEffortMeter(QPainter *painter)
{
    double dBaseRotate = spanAngle() / EFFORT_MAX;

    painter->save();

    painter->setPen(m_EffortPen);
    painter->setBrush(m_EffortBrush);

    // Drawing hand
    painter->save();
    painter->rotate((dBaseRotate * abs(m_Wheel.getEffort())) + 90 + startAngle());
    painter->drawConvexPolygon(effortHand, 3);
    painter->restore();

    // Draw arc
    painter->drawArc(-EFFORT_WIDTH, -EFFORT_WIDTH, 2 * EFFORT_WIDTH, 2 * EFFORT_WIDTH, (startAngle() + 60.0) * 16, - spanAngle() * 16);

    // Draw edges
    painter->rotate(startAngle());
    painter->drawLine(EFFORT_WIDTH, 0, EFFORT_WIDTH - 5, 0);
    for (int j = 0; j < EFFORT_MAX; ++j) {
        painter->rotate(spanAngle() / EFFORT_MAX);
        painter->drawLine(EFFORT_WIDTH, 0, EFFORT_WIDTH - 5, 0);
    }
    painter->restore();
}

void GraphicsWheelMeterItem::drawSpeedMeter(QPainter *painter)
{
    double dBaseRotate = spanAngle() / SPEED_MAX;

    painter->save();

    painter->setPen(m_SpeedPen);
    painter->setBrush(m_SpeedBrush);

    // Drawing hand
    painter->save();
    painter->rotate((dBaseRotate * abs(m_Wheel.getVelocity())) + 90 + startAngle());
    painter->drawConvexPolygon(speedHand, 3);
    painter->restore();

    // Draw arc
    painter->drawArc(-SPEED_WIDTH, -SPEED_WIDTH, 2 * SPEED_WIDTH, 2 * SPEED_WIDTH, (startAngle() + 60.0) * 16, - spanAngle() * 16);

    // Draw edges
    painter->rotate(startAngle());
    painter->drawLine(SPEED_WIDTH, 0, SPEED_WIDTH - 5, 0);
    for (int j = 0; j < SPEED_MAX; ++j) {
        painter->rotate(spanAngle() / SPEED_MAX);
        painter->drawLine(SPEED_WIDTH, 0, SPEED_WIDTH - 5, 0);
    }
    painter->restore();
}

void GraphicsWheelMeterItem::drawPosition(QPainter *painter)
{
    painter->save();




    painter->restore();
}

WheelView::WheelView(QWidget *parent) : QGraphicsView(parent)
{
    m_pScene = new QGraphicsScene;

    setBackgroundBrush(QColor(73, 73, 73));
    setScene(m_pScene);

    m_pWheelMeter = new GraphicsWheelMeterItem;
    m_pWheelMeter->setBrush(QColor(255, 255, 102));
    m_pWheelMeter->setPen(QPen(QColor(255, 255, 102), 5));
    m_pWheelMeter->setStartAngle(150);
    m_pWheelMeter->setSpanAngle(240);
    m_pScene->addItem(m_pWheelMeter);
}

void WheelView::resizeEvent(QResizeEvent *event)
{
    QSize size = event->size();
    m_pScene->setSceneRect(0, 0, size.width(), size.height());
    double radius = 0.75 * (size.width() + size.height()) / 2.0;

    m_pWheelMeter->setArc((size.width() / 2.0) - (radius / 2.0), (size.height() / 2.0) - (radius / 2.0), size.width(), size.height());
}

void WheelView::release()
{
    releaseScene();
}

void WheelView::releaseScene()
{
    if (m_pScene)
        delete m_pScene;
    m_pScene        = NULL;
    m_pWheelMeter   = NULL;
}
