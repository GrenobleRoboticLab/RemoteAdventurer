#ifndef RA_ULTRASONICVIEWER_H
#define RA_ULTRASONICVIEWER_H

#include <nxt_adventurer/RA_Dashboard.h>
#include <QtGui>

using namespace RemoteAdventurer;

namespace RemoteAdventurerClient
{

class UltrasonicViewer : public QGraphicsView
{
    Q_OBJECT
public:
    UltrasonicViewer(QWidget * parent = NULL);
    ~UltrasonicViewer();

    void                    update(const Ultrasonic & ultrasonic);

protected:
    virtual void            resizeEvent(QResizeEvent * event);

private:
    QPalette                m_Pal;
    QGraphicsScene*         m_pScene;

    QGraphicsPolygonItem*   m_pBoundedRange;
    QGraphicsPolygonItem*   m_pPolyRange;

    QGraphicsTextItem*      m_pTextMinrange;
    QGraphicsTextItem*      m_pTextMaxrange;
    QGraphicsTextItem*      m_pTextRange;

    QLinearGradient         m_LinearGradient;
    Ultrasonic              m_LastUltrasonic;

    double                  computeOp(double dAngle, double dAd);
    double                  computeAd(double dValue, double dMaxValue);

    void                    release();
    void                    releaseScene();
};

}

#endif // RA_ULTRASONICVIEWER_H
