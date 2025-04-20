#include "robot_arm_tools/Robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_arm_tools_actions");

    Robot robot;
    robot.startActions();
    
    ros::spin();
    return 0;
}
