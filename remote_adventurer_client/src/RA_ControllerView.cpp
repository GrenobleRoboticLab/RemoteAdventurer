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

    bool            bDirection;
    int             nOrder;

    if (abs(deltaX) > abs(deltaY))
    {
        nOrder = 1;
        if (deltaX > 0)
        {
            p = mapToParent(-dDirSize, 0);
            bDirection = true;
        }
        else
        {
            p = mapToParent(dDirSize, 0);
            bDirection = false;
        }
    }
    else
    {
        nOrder = 0;
        if (deltaY > 0)
        {
            p = mapToParent(0, -dDirSize);
            bDirection = false;
        }
        else
        {
            p = mapToParent(0, dDirSize);
            bDirection = true;
        }
    }

    m_PendingRect = QRectF(m_OrgPos.x() + p.x(), m_OrgPos.y() + p.y(), MOV_ELL_WIDTH, MOV_ELL_WIDTH);

    m_nCount = 0;
    m_dTimerXOff = (m_PendingRect.x() - rect().x()) / TIMER_MAX_LOOP;
    m_dTimerYOff = (m_PendingRect.y() - rect().y()) / TIMER_MAX_LOOP;
    m_bIsAnimated = true;
    m_Timer.start(100);

    emit newOrder(nOrder, bDirection);
}

TrackBarItem::TrackBarItem(QGraphicsItem *parent) : QGraphicsEllipseItem(parent)
{
    setFlag(ItemIsMovable);
}

TrackBarItem::~TrackBarItem() { ; }

void TrackBarItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    double dNewX = rect().x() - (event->lastScenePos().x() - event->scenePos().x());
    if (dNewX < m_OrgPos.x() && abs(dNewX - m_OrgPos.x()) < m_dMaxDelta)
        setRect(dNewX, rect().y(), rect().width(), rect().height());
}

void TrackBarItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    double dDelta = m_OrgPos.x() - rect().x();
    emit newEffort(dDelta * 3 / m_dMaxDelta);
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
    drawEff();

    if(m_pDirEllipse)
        QObject::connect(m_pDirEllipse, SIGNAL(newOrder(int,bool)), this, SLOT(newOrder(int,bool)));
    if(m_pEffEllipse)
        QObject::connect(m_pEffEllipse, SIGNAL(newEffort(double)), this, SLOT(newEffort(double)));
}

ControllerView::~ControllerView()
{
    std::cout << "Destroying ControllerView" << std::endl;
    release();
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

void ControllerView::newOrder(int nOrder, bool bDirection)
{
    m_PendingOrder.order        = nOrder;
    m_PendingOrder.direction    = bDirection;

    emit sendOrder(m_PendingOrder);
}
void ControllerView::newEffort(double dEffort)
{
    m_PendingOrder.effort = dEffort;

    emit sendOrder(m_PendingOrder);
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

void ControllerView::drawEff()
{
    const double    dEffSize = SCENE_DIA * DIR_RAT;
    double          dQuarterSize = (dEffSize / 4);

    QPointF         center(SCENE_DIA - dQuarterSize, dQuarterSize);

    m_pEffLine      = m_pScene->addLine(dQuarterSize, center.y(), dQuarterSize + dEffSize, center.y(), QPen(QColor(141, 5, 38), 1.5));

    m_pEffEllipse   = new TrackBarItem;

    m_pEffEllipse->setRect(center.x() - (MOV_ELL_WIDTH / 2), center.y() - (MOV_ELL_WIDTH / 2), MOV_ELL_WIDTH, MOV_ELL_WIDTH);
    m_pEffEllipse->setBrush(QColor(6, 141, 41));

    m_pEffEllipse->setOrgPos(QPointF(m_pEffEllipse->rect().x(), m_pEffEllipse->rect().y()));

    m_pEffEllipse->setMaxDelta(m_pEffEllipse->mapToParent(SCENE_DIA - (dQuarterSize * 2), 0).x());

    m_pScene->addItem(m_pEffEllipse);
}

void ControllerView::release()
{
    if(m_pScene)
        delete m_pScene;
    m_pScene        = NULL;
    m_pEllipse      = NULL;
    m_pHDirLine     = NULL;
    m_pVDirLine     = NULL;
    m_pDirEllipse   = NULL;
    m_pEffLine      = NULL;
    m_pEffEllipse   = NULL;
}
