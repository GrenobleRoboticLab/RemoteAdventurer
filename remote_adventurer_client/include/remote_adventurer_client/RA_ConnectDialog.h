#ifndef RA_CONNECTDIALOG_H
#define RA_CONNECTDIALOG_H

#include <QtGui>

namespace RemoteAdventurerCLient {

class ConnectDialog : public QDialog
{
    Q_OBJECT
public:
    ConnectDialog(QWidget * parent = NULL);
    ~ConnectDialog() { Release(); }

    void launch() { exec(); }
signals:
    void attemptConnection(QString sIp, QString sInt);
    void attemptQuit();

private slots:
    void okClicked();
    void quitClicked();

private:
    QLabel*         m_pIpLabel;
    QLineEdit*      m_pIpLine;

    QLabel*         m_pPortLabel;
    QLineEdit*      m_pPortLine;

    QPushButton*    m_pOkButton;
    QPushButton*    m_pQuitButton;

    QGridLayout*    m_pGrid;

    void Release();
    void ReleaseIpLabel();
    void ReleaseIpLine();
    void ReleasePortLabel();
    void ReleasePortLine();
    void ReleaseOkButton();
    void ReleaseQuitButton();
    void ReleaseGrid();
};


} // RemoteAdventurerCLient

#endif // RA_CONNECTDIALOG_H
