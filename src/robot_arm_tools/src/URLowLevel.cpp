#include <pluginlib/class_list_macros.h>
#include "robot_arm_tools/URLowLevel.h"

#include <actionlib/client/simple_action_client.h>
#include <ur_dashboard_msgs/SetModeAction.h>
#include <std_srvs/Trigger.h>

void URLowLevel::triggerStartup() 
{
    std_srvs::Trigger trigger;
    
    ros::ServiceClient stopClient = m_nodeHandle.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/stop");
    ros::ServiceClient playClient = m_nodeHandle.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
    ros::Time begin = ros::Time::now();
    bool flag = false;

    while(ros::Time::now().toSec() - begin.toSec() < 5.0)
    {
        if (!stopClient.call(trigger) || !trigger.response.success)
        {
            continue;
        }
        else
        {
            if (!playClient.call(trigger) || !trigger.response.success)
            {
                continue;
            }
            else
            {
                ROS_INFO("ROS.urp started successfully !");
                flag = true;
                break;
            }
        }
    }

    if(!flag)
    {
        ROS_ERROR("%s",trigger.response.message.c_str());
        ROS_ERROR("Unable to start robot !");
        throw std::runtime_error("ROBOT STARTUP ERROR");
    }
}

void URLowLevel::triggerShutdown() 
{
    std_srvs::Trigger trigger;
    
    ros::ServiceClient stopClient = m_nodeHandle.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/stop");
    ros::Time begin = ros::Time::now();
    bool flag = false;

    while(ros::Time::now().toSec() - begin.toSec() < 5.0)
    {
        if (!stopClient.call(trigger) || !trigger.response.success)
        {
            continue;
        }
        else
        {
            ROS_INFO("ROS.urp stopped successfully !");
            flag = true;
            break;
        }
    }

    if(!flag)
    {
        ROS_ERROR("%s",trigger.response.message.c_str());
        ROS_ERROR("Unable to stop robot !");
        throw std::runtime_error("ROBOT STARTUP ERROR");
    }
}

void URLowLevel::recovery() 
{
    actionlib::SimpleActionClient<ur_dashboard_msgs::SetModeAction> recoveryAction("/ur_hardware_interface/set_mode", true);
    recoveryAction.waitForServer();

    ur_dashboard_msgs::SetModeGoal recoveryActionGoal;
    ur_dashboard_msgs::RobotMode runningMode;
    runningMode.mode = 7;
    recoveryActionGoal.target_robot_mode = runningMode;
    recoveryActionGoal.stop_program = false;
    recoveryActionGoal.play_program = false;

    recoveryAction.sendGoal(recoveryActionGoal);

    if(recoveryAction.waitForResult(ros::Duration(30.0)) && recoveryAction.getState().state_ ==  actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED && recoveryAction.getResult()->success)
    {
        ROS_INFO("Robot recovery procedure succeeded !");
    }
    else
    {
        ROS_ERROR("Unable to recover robot !");
        throw std::runtime_error("ROBOT RECOVERY ERROR");
    }
}

PLUGINLIB_EXPORT_CLASS(URLowLevel, RobotLowLevel)
