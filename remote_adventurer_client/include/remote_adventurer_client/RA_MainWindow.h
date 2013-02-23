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

    QWidget*            getWidget() { return m_pWidget; }

    void                update(const Wheel & wheel);

private:
    QWidget*            m_pWidget;

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
    QPalette            m_pal;
};

class ContactViewer : public QWidget
{
    Q_OBJECT
public:
    ContactViewer(QWidget * parent = NULL);

    void                update(const Contact & contact);

private:
    QPalette            m_pal;
};

class UltrasonicViewer : public QVBoxLayout
{
    Q_OBJECT
public:
    UltrasonicViewer(QWidget * parent = NULL);
    ~UltrasonicViewer() { release(); }

    QWidget*            getWidget() { return m_pWidget; }

private:
    QWidget*            m_pWidget;

    void                release() { ; }
};

class RobotViewer : public QGridLayout
{
    Q_OBJECT
public:
    RobotViewer(QWidget * parent = NULL);
    ~RobotViewer() { Release(); }

    QWidget*            getWidget() { return m_pWidget; }

    void                update(const Dashboard & dashboard);

private:
    QWidget*            m_pWidget;

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
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = NULL);
    ~MainWindow() { Release(); }

    QWidget*            getWidget() { return m_pWidget; }

signals:
    
public slots:
    void                updateDash(Dashboard dashboard) { m_pRobotViewer->update(dashboard); }
    void                quit() { quit(); }

protected:

private:
    QWidget*            m_pWidget;
    RobotViewer*        m_pRobotViewer;

    void Release();
    void ReleaseWidget();
    void ReleaseRobotViewer();
};

}

#endif // MAINWINDOW_H
