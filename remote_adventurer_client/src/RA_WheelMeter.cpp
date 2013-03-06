#include "remote_adventurer_client/RA_WheelMeter.h"

using namespace RemoteAdventurerClient;

static const QPoint effortHand[3] = {
                                        QPoint(1, 2),
                                        QPoint(-1, 2),
                                        QPoint(0, -EFFORT_WIDTH)
                                    };

static const QPoint speedHand[3] =  {
                                        QPoint(1, 2),
                                        QPoint(-1, 2),
                                        QPoint(0, -SPEED_WIDTH)
                                    };

WheelMeter::WheelMeter(QWidget *parent) : QWidget(parent)
{
    m_MaxSpeed          = SPEED_MAX;
    m_MaxEffort         = EFFORT_MAX;
    m_SpeedBrush        = QBrush(QColor(141, 5, 38));
    m_SpeedPen          = QPen(QColor(141, 5, 38), 3);
    m_EffortBrush       = QBrush(QColor(141, 41, 5));
    m_EffortPen         = QPen(QColor(141, 41, 5), 2);
    m_PosDefaultColor   = Qt::white;
}

WheelMeter::~WheelMeter()
{
    std::cout << "Destroying WheelMeter" << std::endl;
}

void WheelMeter::paintEvent(QPaintEvent *event)
{
    int side = qMin(width(), height());

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(width() / 2, height() / 2);
    painter.scale(side / 200.0, side / 200.0);

    if (m_Wheel.getEffort() < 0  || m_Wheel.getVelocity() < 0)
        drawReverseWarn(&painter);

    drawSpeedMeter(&painter);
    drawEffortMeter(&painter);
    drawPosition(&painter);
}

void WheelMeter::drawEffortMeter(QPainter *painter)
{
    double dBaseRotate = spanAngle() / EFFORT_MAX;
    double dEffort = (m_Wheel.getEffort() >= 0) ? m_Wheel.getEffort() : -m_Wheel.getEffort();

    painter->save();

    painter->setPen(m_EffortPen);
    painter->setBrush(m_EffortBrush);

    // Drawing hand
    painter->save();
    painter->rotate((dBaseRotate * dEffort) + 90 + startAngle());
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

void WheelMeter::drawSpeedMeter(QPainter *painter)
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

void WheelMeter::drawPosition(QPainter *painter)
{
    QString sPosText    = QString("Position : ") + QString::number(m_Wheel.getPosition());
    QString sNameText   = QString::fromStdString(m_Wheel.getName());
    QFontMetrics fm     = painter->fontMetrics();
    QPointF posPoint    = QPointF(-fm.width(sPosText)/2, 60);
    QPointF namePoint   = QPointF(-fm.width(sNameText)/2, 80);

    painter->save();

    painter->setPen(m_PosDefaultColor);
    painter->drawText(posPoint, sPosText);
    painter->drawText(namePoint, sNameText);


    painter->restore();
}

void WheelMeter::drawReverseWarn(QPainter *painter)
{
    QString sText("/!\\Reverse !");
    QFontMetrics fm = painter->fontMetrics();
    painter->setPen(Qt::red);
    painter->drawText(-fm.width(sText) / 2, 0, sText);
}

void WheelMeter::update(const Wheel &wheel)
{
    if (!m_Wheel.compare(wheel))
    {
        m_Wheel = wheel;
        repaint();
    }
}
