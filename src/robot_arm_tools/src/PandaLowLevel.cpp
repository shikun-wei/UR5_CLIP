#include <pluginlib/class_list_macros.h>
#include "robot_arm_tools/PandaLowLevel.h"

#include <actionlib/client/simple_action_client.h>
#include <franka_msgs/ErrorRecoveryAction.h>

void PandaLowLevel::recovery() 
{
    actionlib::SimpleActionClient<franka_msgs::ErrorRecoveryAction> recoveryAction("/franka_control/error_recovery", true);
    recoveryAction.waitForServer();

    franka_msgs::ErrorRecoveryGoal recoveryActionGoal;
    recoveryAction.sendGoal(recoveryActionGoal);

    if(recoveryAction.waitForResult(ros::Duration(30.0)) && recoveryAction.getState().state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
        ROS_INFO("Robot recovery procedure succeeded !");
    }
    else
    {
        ROS_ERROR("Unable to recover robot !");
        throw std::runtime_error("ROBOT RECOVERY ERROR");
    }
}


PLUGINLIB_EXPORT_CLASS(PandaLowLevel, RobotLowLevel)

