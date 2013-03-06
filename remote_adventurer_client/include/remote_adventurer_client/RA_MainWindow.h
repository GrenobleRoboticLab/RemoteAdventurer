#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <nxt_adventurer/RA_Dashboard.h>
#include "RA_WheelMeter.h"
#include "RA_UltrasonicViewer.h"
#include "RA_ControllerView.h"

#include <QtGui>

using namespace RemoteAdventurer;

namespace RemoteAdventurerClient
{
class ColorViewer : public QWidget
{
    Q_OBJECT
public:
    ColorViewer(QWidget * parent = NULL);
    ~ColorViewer() { std::cout << "Destorying ColorViewer" << std::endl; }

    void                update(const Color & color);

private:
    QPalette            m_Pal;
};

class ContactViewer : public QWidget
{
    Q_OBJECT
public:
    ContactViewer(QWidget * parent = NULL);
    ~ContactViewer() { std::cout << "Destroying ContactViewer" << std::endl; }

    void                update(const Contact & contact);

private:
    QPalette            m_Pal;
};

class RobotViewer : public QGridLayout
{
    Q_OBJECT
public:
    RobotViewer(QWidget * parent = NULL);
    ~RobotViewer() { release(); }

    void                update(const Dashboard & dashboard);

private:
    WheelMeter*         m_pRightWheel;
    WheelMeter*         m_pLeftWheel;
    WheelMeter*         m_pAuxWheel;

    ColorViewer*        m_pColor;
    ContactViewer*      m_pRightContact;
    ContactViewer*      m_pLeftContact;
    UltrasonicViewer*   m_pUltrasonic;

    ControllerView*     m_pController;

    void                release();
    void                releaseRightWheel();
    void                releaseLeftWheel();
    void                releaseAuxWheel();
    void                releaseColor();
    void                releaseRightContact();
    void                releaseLeftContact();
    void                releaseUltrasonic();
    void                releaseController();
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

    void                release();
    void                releaseRobotViewer();
};

}

#endif // MAINWINDOW_H
