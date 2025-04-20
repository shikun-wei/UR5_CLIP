
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include "robot_arm_tools/PauseButton.h"

PauseButton::PauseButton(QWidget *parent)
  : QMainWindow(parent)
{
    // Create the button, make "this" the parent
    m_button = new QPushButton("Pause Button", this);
    m_button->setMinimumWidth(80);
    m_button->setMinimumHeight(80);
    
    // Connect button signal to appropriate slot
    connect(m_button, &QPushButton::toggled, this, &PauseButton::handleButton);

    m_button->setCheckable(true);
    m_button->setChecked(false);
    m_button->setText("PAUSE");
    m_button->setAutoFillBackground(true);
    m_button->setStyleSheet("background-color: rgb(255,0,0); color: black; border: none; font-size: 12px;");

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(m_button, 0, Qt::AlignCenter);
    QWidget *mainWidget = new QWidget;
    mainWidget->setLayout(layout);
    setCentralWidget(mainWidget);

    ros::NodeHandle nh;
    m_pausePublisher = nh.advertise<std_msgs::Bool>("/pause", 1, true);
}
 
void PauseButton::handleButton(bool checked)
{
    if(checked)
    {
        m_button->setText("PLAY");
        m_button->setStyleSheet("background-color: rgb(0,255,0); color: black; border: none; font-size: 12px;");
    }

    else
    {
        m_button->setText("PAUSE");
        m_button->setStyleSheet("background-color: rgb(255,0,0); color: black; border: none; font-size: 12px;");
    }
    m_msg.data = checked;
    m_pausePublisher.publish(m_msg);
}

PauseButton::~PauseButton()
{
    m_pausePublisher.shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pause_button");
    
    QApplication app(argc, argv);
    PauseButton button;
    button.show();

    while(ros::ok())
    {
        QCoreApplication::processEvents();
        ros::spinOnce();
    }
    
    return 0; 
}

