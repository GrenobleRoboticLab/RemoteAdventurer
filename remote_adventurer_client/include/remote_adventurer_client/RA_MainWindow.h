#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <nxt_adventurer/RA_Dashboard.h>
#include "RA_WheelMeter.h"
#include "RA_UltrasonicViewer.h"

#include <QtGui>

using namespace RemoteAdventurer;

namespace RemoteAdventurerCLient
{
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

class RobotViewer : public QGridLayout
{
    Q_OBJECT
public:
    RobotViewer(QWidget * parent = NULL);
    ~RobotViewer() { Release(); }

    void                update(const Dashboard & dashboard);

private:
    WheelMeter*         m_pRightWheel;
    WheelMeter*         m_pLeftWheel;
    WheelMeter*         m_pAuxWheel;

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
