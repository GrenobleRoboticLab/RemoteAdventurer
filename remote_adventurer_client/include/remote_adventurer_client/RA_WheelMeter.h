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

class WheelMeter : public QWidget {
public:
    WheelMeter (QWidget * parent = NULL);
    virtual ~WheelMeter() { ; }

    int     maxSpeed()          const { return m_MaxSpeed;          }
    int     maxEffort()         const { return m_MaxEffort;         }
    QBrush  speedBrush()        const { return m_SpeedBrush;        }
    QPen    speedPen()          const { return m_SpeedPen;          }
    QBrush  effortBrush()       const { return m_EffortBrush;       }
    QPen    effortPen()         const { return m_EffortPen;         }
    QColor  posDefaultColor()   const { return m_PosDefaultColor;   }
    Wheel   wheel()             const { return m_Wheel;             }
    QRectF  rect()              const { return m_Rect;              }
    double  startAngle()        const { return m_dStartAngle;       }
    double  spanAngle()         const { return m_dSpanAngle;        }

    void    setMaxSpeed(int maxSpeed)                   { m_MaxSpeed        = maxSpeed;     }
    void    setMaxEffort(int maxEffort)                 { m_MaxEffort       = maxEffort;    }
    void    setSpeedBrush(const QBrush & brush)         { m_SpeedBrush      = brush;        }
    void    setSpeedPen(const QPen & pen)               { m_SpeedPen        = pen;          }
    void    setEffortBrush(const QBrush & brush)        { m_EffortBrush     = brush;        }
    void    setEffortPen(const QPen & pen)              { m_EffortPen       = pen;          }
    void    setPosDefaultColor(const QColor & color)    { m_PosDefaultColor = color;        }
    void    setRect(const QRectF & rect)                { m_Rect            = rect;         }
    void    setStartAngle(double dAngle)                { m_dStartAngle     = dAngle;       }
    void    setSpanAngle(double dAngle)                 { m_dSpanAngle      = dAngle;       }
    void    setRect(double x, double y, double width, double height) { setRect(QRect(x, y, width, height)); }

    void    update(const Wheel & wheel);

protected:
    void paintEvent(QPaintEvent *event);

private:
    int     m_MaxSpeed;
    int     m_MaxEffort;
    double  m_dStartAngle;
    double  m_dSpanAngle;


    QBrush  m_SpeedBrush;
    QPen    m_SpeedPen;

    QBrush  m_EffortBrush;
    QPen    m_EffortPen;

    QColor  m_PosDefaultColor;
    Wheel   m_Wheel;

    QRectF  m_Rect;

    void    drawSpeedMeter(QPainter* painter);
    void    drawEffortMeter(QPainter* painter);
    void    drawPosition(QPainter* painter);
    void    drawReverseWarn(QPainter* painter);
};

}

#endif // RA_WHEELMETER_H
