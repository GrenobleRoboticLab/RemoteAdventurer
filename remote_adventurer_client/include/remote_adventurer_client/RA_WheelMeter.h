#ifndef RA_WHEELMETER_H
#define RA_WHEELMETER_H

#include <nxt_adventurer/RA_Dashboard.h>

#include <QtGui>

#define SPEED_MAX 15.0
#define SPEED_WIDTH 90.0
#define EFFORT_MAX 2.0
#define EFFORT_WIDTH 50.0

using namespace RemoteAdventurer;

namespace RemoteAdventurerCLient {

class GraphicsWheelMeterItem : public QGraphicsEllipseItem {
public:
    GraphicsWheelMeterItem (qreal x = 0, qreal y = 0, qreal width = 0, qreal height = 0, QGraphicsItem * parent = NULL);
    virtual ~GraphicsWheelMeterItem() { ; }

    void setArc(qreal x = 0, qreal y = 0, qreal width = 0, qreal height = 0);

    int     maxSpeed()          const { return m_MaxSpeed;          }
    int     maxEffort()         const { return m_MaxEffort;         }
    QBrush  speedBrush()        const { return m_SpeedBrush;        }
    QPen    speedPen()          const { return m_SpeedPen;          }
    QBrush  effortBrush()       const { return m_EffortBrush;       }
    QPen    effortPen()         const { return m_EffortPen;         }
    QColor  posDefaultColor()   const { return m_PosDefaultColor;   }
    Wheel   wheel()             const { return m_Wheel;             }

    void    setMaxSpeed(int maxSpeed)                   { m_MaxSpeed        = maxSpeed;     }
    void    setMaxEffort(int maxEffort)                 { m_MaxEffort       = maxEffort;    }
    void    setSpeedBrush(const QBrush & brush)         { m_SpeedBrush      = brush;        }
    void    setSpeedPen(const QPen & pen)               { m_SpeedPen        = pen;          }
    void    setEffortBrush(const QBrush & brush)        { m_EffortBrush     = brush;        }
    void    setEffortPen(const QPen & pen)              { m_EffortPen       = pen;          }
    void    setPosDefaultColor(const QColor & color)    { m_PosDefaultColor = color;        }
    void    setWheel(const Wheel & wheel)               { m_Wheel           = wheel;        }

private:
    int     m_MaxSpeed;
    int     m_MaxEffort;

    QBrush  m_SpeedBrush;
    QPen    m_SpeedPen;

    QBrush  m_EffortBrush;
    QPen    m_EffortPen;

    QColor  m_PosDefaultColor;

    Wheel   m_Wheel;

    void    paint ( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

    void    drawSpeedMeter(QPainter* painter);
    void    drawEffortMeter(QPainter* painter);
    void    drawPosition(QPainter* painter);
};

class WheelView : public QGraphicsView
{
    Q_OBJECT
public:
    WheelView(QWidget *parent = 0);
    virtual ~WheelView() { release(); }
    
signals:
    
public slots:

protected:
    virtual void            resizeEvent(QResizeEvent * event);

private:
    QGraphicsScene*         m_pScene;
    GraphicsWheelMeterItem* m_pWheelMeter;

    void                    release();
    void                    releaseScene();
};

}

#endif // RA_WHEELMETER_H
