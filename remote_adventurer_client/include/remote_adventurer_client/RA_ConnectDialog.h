#ifndef RA_CONNECTDIALOG_H
#define RA_CONNECTDIALOG_H

#include <QtGui>

namespace RemoteAdventurerClient {

/**
 * This class provide dialog for Ip and Port inputs
 */
class ConnectDialog : public QDialog
{
    Q_OBJECT
public:
    ConnectDialog(QWidget * parent = NULL);
    ~ConnectDialog();

    /**
     * Show this QDialog object.
     */
    void            launch() { exec(); }
    /**
     * Release all members pointer.
     */
    void            release();

signals:
    /**
     * Send ip and port signal.
     */
    void            attemptConnection(QString sIp, QString sInt);
    /**
     * Send quit signal.
     */
    void            attemptQuit();

private slots:
    void            okClicked();
    void            quitClicked();

private:
    QLabel*         m_pIpLabel;
    QLineEdit*      m_pIpLine;

    QLabel*         m_pPortLabel;
    QLineEdit*      m_pPortLine;

    QPushButton*    m_pOkButton;
    QPushButton*    m_pQuitButton;

    QGridLayout*    m_pGrid;

    void            releaseIpLabel();
    void            releaseIpLine();
    void            releasePortLabel();
    void            releasePortLine();
    void            releaseOkButton();
    void            releaseQuitButton();
    void            releaseGrid();
};


} // RemoteAdventurerCLient

#endif // RA_CONNECTDIALOG_H
