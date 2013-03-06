#include "remote_adventurer_client/RA_ControllerView.h"
#include <QPropertyAnimation>

using namespace RemoteAdventurerClient;

DirGearItem::DirGearItem(QGraphicsItem * parent) : QGraphicsEllipseItem(parent)
{
    m_bIsAnimated = false;
    setFlag(ItemIsMovable);
    QObject::connect(&m_Timer, SIGNAL(timeout()), this, SLOT(timeout()));
}

DirGearItem::~DirGearItem()
{
    ;
}

void DirGearItem::timeout()
{
    if (m_nCount < TIMER_MAX_LOOP)
    {
        setRect(rect().x() + m_dTimerXOff, rect().y() + m_dTimerYOff, rect().width(), rect().height());
        m_nCount++;
    }
    else {
        setRect(m_PendingRect);
        m_Timer.stop();
    }
}

void DirGearItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    m_PendingRect = rect();
}

void DirGearItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    double dX = event->lastScenePos().x() - event->scenePos().x();
    double dY = event->lastScenePos().y() - event->scenePos().y();

    setRect(rect().x() - dX, rect().y() - dY, rect().width(), rect().height());
}

void DirGearItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    double          deltaX = m_OrgPos.x() - rect().x() - (MOV_ELL_WIDTH / 2),
                    deltaY = m_OrgPos.y() - rect().y() - (MOV_ELL_WIDTH / 2);
    const double    dDirSize = (SCENE_DIA * DIR_RAT) / 2;
    QPointF         p;

    if (abs(deltaX) > abs(deltaY))
    {
        if (deltaX > 0)
            p = mapToParent(-dDirSize, 0);
        else
            p = mapToParent(dDirSize, 0);
    }
    else
    {
        if (deltaY > 0)
            p = mapToParent(0, -dDirSize);
        else
            p = mapToParent(0, dDirSize);
    }

    m_PendingRect = QRectF(m_OrgPos.x() + p.x(), m_OrgPos.y() + p.y(), MOV_ELL_WIDTH, MOV_ELL_WIDTH);

    m_nCount = 0;
    m_dTimerXOff = (m_PendingRect.x() - rect().x()) / TIMER_MAX_LOOP;
    m_dTimerYOff = (m_PendingRect.y() - rect().y()) / TIMER_MAX_LOOP;
    m_bIsAnimated = true;
    m_Timer.start(100);
}

ControllerView::ControllerView(QWidget* parent) : QGraphicsView(parent)
{
    m_pEllipse      = NULL;
    m_pHDirLine     = NULL;
    m_pVDirLine     = NULL;
    m_pDirEllipse   = NULL;
    m_pEffLine      = NULL;
    m_pEffEllipse   = NULL;

    m_pScene        = new QGraphicsScene;

    setRenderHint(QPainter::Antialiasing);
    setBackgroundBrush(Qt::black);
    setScene(m_pScene);
    m_pScene->setSceneRect(0, 0, SCENE_DIA, SCENE_DIA);
    drawDir();
}

ControllerView::~ControllerView()
{
    ;
}

void ControllerView::resizeEvent(QResizeEvent *event)
{
    double  nSide    = qMin(event->size().width(), event->size().height()),
            nOldSide = qMin(event->oldSize().width(), event->oldSize().height()),
            dScaleFactor = 0.0;

    if (nOldSide < 1)
        dScaleFactor = (nSide / nOldSide) / SCENE_DIA;
    else
        dScaleFactor = (nSide / nOldSide);

    scale(dScaleFactor, dScaleFactor);
}

void ControllerView::drawDir()
{
    const double    dDirSize = SCENE_DIA * DIR_RAT;
    double          dX = (dDirSize / 4),
                    dY = SCENE_DIA - dDirSize;
    QPointF         center(dX + dDirSize / 2, dY + dDirSize / 2);

    m_pHDirLine     = m_pScene->addLine(dX, center.y(), dX + dDirSize, center.y(), QPen(QColor(141, 5, 38), 1.5));
    m_pVDirLine     = m_pScene->addLine(center.x(), dY, center.x(), dY + dDirSize, QPen(QColor(141, 5, 38), 1.5));
    m_pEllipse      = m_pScene->addEllipse(dX, dY, dDirSize, dDirSize, QPen(QColor(141, 5, 38), 1.5));

    m_pDirEllipse   = new DirGearItem;
    m_pDirEllipse->setRect(center.x() - (MOV_ELL_WIDTH / 2), center.y() - (MOV_ELL_WIDTH / 2), MOV_ELL_WIDTH, MOV_ELL_WIDTH);
    m_pDirEllipse->setOrgPos(QPointF(m_pDirEllipse->rect().x(), m_pDirEllipse->rect().y()));

    m_pDirEllipse->setBrush(QColor(6, 141, 41));
    m_pScene->addItem(m_pDirEllipse);
}

void ControllerView::release()
{
    ;
}
