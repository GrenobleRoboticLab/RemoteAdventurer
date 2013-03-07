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

/**
 * This class provides a gear for sending Order's direction informations.
 */
class DirGearItem : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
public:
    DirGearItem(QGraphicsItem * parent = 0 );
    ~DirGearItem();

    /**
     * Set the original position of this QGraphicsEllipseItem.
     */
    void            setOrgPos(QPointF p) { m_OrgPos = p; }

signals:
    /**
     * Send new Order's direction informations.
     */
    void            newOrder(int nOrder, bool bDirection);

protected:
    /**
     * Store the current rect use to draw this QGraphicsEllipseItem object.
     */
    virtual void    mousePressEvent(QGraphicsSceneMouseEvent *event);
    /**
     * Set the rect use to draw this QGraphicsEllipseItem object to the mouse position.
     */
    virtual void    mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    /**
     * Compute the previous mouse movement to emit the new order's direction informations.
     */
    virtual void    mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private slots:
    void            timeout();

private:
    bool            m_bIsAnimated;

    QPointF         m_OrgPos;
    QRectF          m_PendingRect;

    QTimer          m_Timer;
    int             m_nCount;
    double          m_dTimerXOff;
    double          m_dTimerYOff;
}; //class DirGearItem


/**
 * This class provide QGraphicsEllipseItem to set the desired effort.
 */
class TrackBarItem : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
public:
    TrackBarItem(QGraphicsItem * parent = NULL);
    ~TrackBarItem();

    /**
     * Set the origin position of this QGraphicsEllipseItem object.
     */
    void            setOrgPos(QPointF p)            { m_OrgPos = p; }
    /**
     * Set the delta max from the origin position to compute the order's desired effort.
     */
    void            setMaxDelta(double dMaxDelta)   { m_dMaxDelta = dMaxDelta; }

signals:
    /**
     * Send the new desired effort.
     */
    void            newEffort(double dEffort);

protected:
    /**
     * Set the rect use to draw this QGraphicsEllipseItem object to the mouse position.
     */
    virtual void    mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    /**
     * Compute the previous mouse movement to emit the new order's effort.
     */
    virtual void    mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    double          m_dMaxDelta;

    QPointF         m_OrgPos;
    QRectF          m_PendingRect;

};//class TrackBarItem

/**
 * This class provide a QGraphicsView to send order to the server.
 */
class ControllerView : public QGraphicsView {
    Q_OBJECT
public:
    ControllerView(QWidget* parent = NULL);
    ~ControllerView();

signals:
    /**
     * Send the new order.
     */
    void                        sendOrder(const nxt_adventurer::Order & order);

protected:
    /**
     * Adjust the transformation matrix of this QGraphicsView with the new size.
     */
    virtual void                resizeEvent(QResizeEvent * event);

private slots:
    void                        newOrder(int nOrder, bool bDirection);
    void                        newEffort(double dEffort);

private:
    QPalette                    m_Pal;
    QGraphicsScene*             m_pScene;

    QGraphicsEllipseItem*       m_pEllipse;
    QGraphicsLineItem*          m_pHDirLine;
    QGraphicsLineItem*          m_pVDirLine;
    DirGearItem*                m_pDirEllipse;

    QGraphicsLineItem*          m_pEffLine;
    TrackBarItem*               m_pEffEllipse;

    nxt_adventurer::Order       m_PendingOrder;
    nxt_adventurer::Order       m_CurrentOrder;

    void                        drawDir();
    void                        drawEff();

    void                        release();
};//class ControllerView

} // namespace RemoteAdventurerClient

#endif // RA_CONTROLLERVIEW_H
