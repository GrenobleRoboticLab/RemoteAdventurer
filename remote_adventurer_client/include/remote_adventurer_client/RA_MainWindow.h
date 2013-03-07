#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <nxt_adventurer/RA_Dashboard.h>
#include "RA_WheelMeter.h"
#include "RA_UltrasonicViewer.h"
#include "RA_ControllerView.h"
#include "RA_TcpClient.h"

#include <QtGui>

using namespace RemoteAdventurer;

namespace RemoteAdventurerClient
{
/**
 * This class show color information get from the server.
 */
class ColorViewer : public QWidget
{
    Q_OBJECT
public:
    ColorViewer(QWidget * parent = NULL);
    ~ColorViewer() { std::cout << "Destorying ColorViewer" << std::endl; }

    /**
     * Update the color of this QWidget object.
     */
    void                update(const Color & color);

private:
    QPalette            m_Pal;
};

/**
 * This class show contact information get from the server.
 */
class ContactViewer : public QWidget
{
    Q_OBJECT
public:
    ContactViewer(QWidget * parent = NULL);
    ~ContactViewer() { std::cout << "Destroying ContactViewer" << std::endl; }

    /**
     * Update the color of this QWidget object.
     */
    void                update(const Contact & contact);

private:
    QPalette            m_Pal;
};

/**
 * This class provide a layout to show dashboard's informations get from the server.
 */
class RobotViewer : public QGridLayout
{
    Q_OBJECT
public:
    RobotViewer(QWidget * parent = NULL);
    ~RobotViewer() { release(); }

    /**
     * Update all dashboard informations.
     */
    void                update(const Dashboard & dashboard);

    /**
     * Connect the given TcpClient to the ControllerView
     */
    void                connect(TcpClient * pTcp);
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

/**
 * This class provides a MainWindow to show dashboard informations and get movement wanted by the user.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = NULL);
    ~MainWindow();

    /**
     * @see RobotViewer::connect(TcpClient * pTcp)
     */
    void                connect(TcpClient * pTcp);
    
public slots:
    /**
     * Update dashboard's informations.
     */
    void                updateDash(Dashboard dashboard) { m_pRobotViewer->update(dashboard); }

private:
    QWidget             m_Widget;
    RobotViewer*        m_pRobotViewer;

    void                release();
    void                releaseRobotViewer();
};

}

#endif // MAINWINDOW_H
