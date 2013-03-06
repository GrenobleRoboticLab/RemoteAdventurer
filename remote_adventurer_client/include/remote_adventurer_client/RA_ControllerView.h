#ifndef RA_CONTROLLERVIEW_H
#define RA_CONTROLLERVIEW_H

#include <nxt_adventurer/RA_XMLHelper.h>
#include <QtGui>

#define SCENE_DIA 50.0
#define DIR_RAT 2 / 3
#define MOV_ELL_WIDTH 4.0
#define TIMER_MAX_LOOP 10

using namespace RemoteAdventurer;

namespace RemoteAdventurerClient {

class DirGearItem : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
public:
    DirGearItem(QGraphicsItem * parent = 0 );
    ~DirGearItem();

    void            setOrgPos(QPointF p) { m_OrgPos = p; }

public slots:
    void            timeout();

protected:
    virtual void    mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void    mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    virtual void    mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    bool            m_bIsAnimated;

    QPointF         m_OrgPos;
    QRectF          m_PendingRect;

    QTimer          m_Timer;
    int             m_nCount;
    double          m_dTimerXOff;
    double          m_dTimerYOff;
};

class ControllerView : public QGraphicsView {
    Q_OBJECT
public:
    ControllerView(QWidget* parent = NULL);
    ~ControllerView();

protected:
    virtual void                resizeEvent(QResizeEvent * event);

private:
    QPalette                    m_Pal;
    QGraphicsScene*             m_pScene;

    QGraphicsEllipseItem*       m_pEllipse;
    QGraphicsLineItem*          m_pHDirLine;
    QGraphicsLineItem*          m_pVDirLine;
    DirGearItem*                m_pDirEllipse;

    QGraphicsLineItem*          m_pEffLine;
    DirGearItem*                m_pEffEllipse;

    nxt_adventurer::Order       m_PendingOrder;
    nxt_adventurer::Order       m_CurrentOrder;

    void                        drawDir();

    void                        release();
};

} // namespace RemoteAdventurerClient

#endif // RA_CONTROLLERVIEW_H
