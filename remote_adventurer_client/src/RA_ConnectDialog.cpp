#include "remote_adventurer_client/RA_ConnectDialog.h"
#include <iostream>

using namespace RemoteAdventurerCLient;

ConnectDialog::ConnectDialog(QWidget *parent) : QDialog(parent)
{
    m_pIpLabel      = new QLabel("IP : ");
    m_pIpLine       = new QLineEdit("");

    m_pPortLabel    = new QLabel("PORT : ");
    m_pPortLine     = new QLineEdit("");

    m_pOkButton     = new QPushButton("OK");
    m_pQuitButton   = new QPushButton("QUIT");

    m_pGrid         = new QGridLayout;

    QObject::connect(m_pOkButton, SIGNAL(clicked()), this, SLOT(okClicked()));
    QObject::connect(m_pQuitButton, SIGNAL(clicked()), this, SLOT(quitClicked()));

    m_pGrid->addWidget(m_pIpLabel);
    m_pGrid->addWidget(m_pIpLine, 0, 1);

    m_pGrid->addWidget(m_pPortLabel, 1, 0);
    m_pGrid->addWidget(m_pPortLine, 1, 1);

    m_pGrid->addWidget(m_pQuitButton, 2, 0);
    m_pGrid->addWidget(m_pOkButton, 2, 1);

    setLayout(m_pGrid);
    setWindowTitle("Connect");
}

ConnectDialog::~ConnectDialog()
{
    std::cout << "Destroying ConnectDialog" << std::endl;
    release();
}

void ConnectDialog::quitClicked()
{
    emit attemptQuit();
}

void ConnectDialog::okClicked()
{
    emit attemptConnection(m_pIpLine->text(), m_pPortLine->text());
}

void ConnectDialog::release()
{
    releaseIpLabel();
    releaseIpLine();
    releasePortLabel();
    releasePortLine();
    releaseOkButton();
    releaseQuitButton();
    releaseGrid();
}

void ConnectDialog::releaseIpLabel()
{
    if (m_pIpLabel)
        delete m_pIpLabel;
    m_pIpLabel = NULL;
}

void ConnectDialog::releaseIpLine()
{
    if (m_pIpLine)
        delete m_pIpLine;
    m_pIpLine = NULL;
}

void ConnectDialog::releasePortLabel()
{
    if (m_pPortLabel)
        delete m_pPortLabel;
    m_pPortLabel = NULL;
}

void ConnectDialog::releasePortLine()
{
    if (m_pPortLine)
        delete m_pPortLine;
    m_pPortLine = NULL;
}

void ConnectDialog::releaseOkButton()
{
    if (m_pOkButton)
        delete m_pOkButton;
    m_pOkButton = NULL;
}

void ConnectDialog::releaseQuitButton()
{
    if (m_pQuitButton)
        delete m_pQuitButton;
    m_pQuitButton = NULL;
}

void ConnectDialog::releaseGrid()
{
    if (m_pGrid)
        delete m_pGrid;
    m_pGrid = NULL;
}
