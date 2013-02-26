#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <nxt_adventurer/RA_Dashboard.h>
#include <iostream>

#include <QtGui>

using namespace RemoteAdventurer;

namespace RemoteAdventurerCLient
{

class WheelViewer : public QVBoxLayout
{
    Q_OBJECT
public:
    WheelViewer(QWidget * parent = NULL);
    ~WheelViewer() { release(); }

    void                update(const Wheel & wheel);

private:
    QLabel*             m_pName;
    QLabel*             m_pEffortValue;
    QLabel*             m_pVelocityValue;
    QLabel*             m_pPositionValue;

    void                release();
    void                releasePosition();
    void                releaseVelocity();
    void                releaseEffort();
    void                releaseName();
    void                releaseWidget();
};

class ColorViewer : public QWidget
{
    Q_OBJECT
public:
    ColorViewer(QWidget * parent = NULL);

    void                update(const Color & color);

private:
    QPalette            m_Pal;
};

class ContactViewer : public QWidget
{
    Q_OBJECT
public:
    ContactViewer(QWidget * parent = NULL);

    void                update(const Contact & contact);

private:
    QPalette            m_Pal;
};

class UltrasonicViewer : public QGraphicsView
{
    Q_OBJECT
public:
    UltrasonicViewer(QWidget * parent = NULL);
    ~UltrasonicViewer() { release(); }

    void             update(const Ultrasonic & ultrasonic);

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

class RobotViewer : public QGridLayout
{
    Q_OBJECT
public:
    RobotViewer(QWidget * parent = NULL);
    ~RobotViewer() { Release(); }

    void                update(const Dashboard & dashboard);

private:
    QWidget             m_RWWidget;
    QWidget             m_LWWidget;
    QWidget             m_AWWidget;

    WheelViewer*        m_pRightWheel;
    WheelViewer*        m_pLeftWheel;
    WheelViewer*        m_pAuxWheel;

    ColorViewer*        m_pColor;

    ContactViewer*      m_pRightContact;
    ContactViewer*      m_pLeftContact;

    UltrasonicViewer*   m_pUltrasonic;

    void Release();
    void ReleaseRightWheel();
    void ReleaseLeftWheel();
    void ReleaseAuxWheel();
    void ReleaseColor();
    void ReleaseRightContact();
    void ReleaseLeftContact();
    void ReleaseUltrasonic();
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = NULL);
    ~MainWindow();

signals:
    
public slots:
    void                updateDash(Dashboard dashboard) { m_pRobotViewer->update(dashboard); }
    void                quit() { quit(); }

protected:

private:
    QWidget             m_Widget;
    RobotViewer*        m_pRobotViewer;

    void Release();
    void ReleaseRobotViewer();
};

}

#endif // MAINWINDOW_H
