#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <std_srvs/Empty.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_tools_scan_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    //robot.init();
    
    //Define and add collisions objects
    geometry_msgs::Pose centerPose;
    centerPose.position.x = 0.4;
    centerPose.position.y = 0.0;
    centerPose.position.z = 0.5;

    double radiusObject = 0.05;
    double radiusTrajectory = radiusObject + 0.05;
    
    if(radiusObject != 0)
    {
        visualTools.addSphere("collisionSphere", centerPose, radiusObject, false);
    }

    //Create measurement waypoints poses
    int N=20;   //Waypoints number
    std::vector<geometry_msgs::Pose> waypoints;

    sphericInclinationTrajectory(centerPose, radiusTrajectory, M_PI/2, 0, 2*M_PI, N, waypoints);

    //Main loop
    ros::NodeHandle nh;
    bool constrainedYaw = nh.param<bool>("constrainedYaw", false);
    bool visibilityConstraint = nh.param<bool>("visibilityConstraint", false);

    robot.runMeasurementRoutine(waypoints,false,true,constrainedYaw ? 0.0 : -1.0,visibilityConstraint,false);

    //stop robot
    //robot.init();

    if(radiusObject != 0)
    {
        visualTools.deleteObject("collisionSphere");
    }

    ros::shutdown();
    return 0;
}

