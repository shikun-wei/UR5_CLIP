#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/PandaGripper.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_tools_example_gripper_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;

    //Gripper initialisation
    PandaGripper gripper(robot);

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    robot.init();
    gripper.init();

    //Define a collision object - A sphere for instance
    geometry_msgs::Pose spherePose;
    spherePose.position.x = 0.5;
    spherePose.position.y = 0.0;
    spherePose.position.z = 0.2;
    double sphereRadius = 0.02;

    visualTools.addSphere("sphere", spherePose, sphereRadius, false);

    //Define a target pose above the collision object
    geometry_msgs::Pose abovePose = sphericPose(spherePose, sphereRadius + 0.2, 0, 0);
    robot.goToTarget(abovePose);

    //Open and close the gripper
    gripper.open();
    ros::WallDuration(1.0).sleep();
    gripper.close();

    //stop robot
    robot.init();
    visualTools.deleteObject("sphere");

    ros::shutdown();
    return 0;
}