#pragma once

#include <QMainWindow>
#include <QPushButton>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#pragma once

namespace Ui {
  class MainWindow;
}
 
class PauseButton : public QMainWindow
{
    public:
        PauseButton(QWidget *parent = nullptr);
        virtual ~PauseButton();

    private slots:
        void handleButton(bool checked);

    private:
        std_msgs::Bool m_msg;
        ros::Publisher m_pausePublisher;
        QPushButton *m_button;
};