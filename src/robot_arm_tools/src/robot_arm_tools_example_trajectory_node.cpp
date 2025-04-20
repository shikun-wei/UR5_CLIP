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
    ros::init(argc, argv, "robot_arm_tools_trajectory_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;
    
    //Retreive trajectory from file => either as poses ("poses") or joint values ("jointValues")
    ros::NodeHandle nh("~");
    std::string filePath;
    if(!nh.getParam("trajectoryFilePath",filePath))
    {
        ROS_ERROR("No trajectory file path provided !");
        throw std::runtime_error("NO TRAJECTORY FILE PATH");
    }
    bool visibilityConstraint = nh.param<bool>("visibilityConstraint",true);
    bool constrainedYaw = nh.param<bool>("constrainedYaw",false);

    try
    {
        std::vector<std::vector<double>> waypoints;
        trajectoryFromFile(waypoints,filePath);

        robot.runMeasurementRoutine(waypoints,false,true,false);
    }
    catch(const std::exception& e)
    {
        try
        {
            std::vector<geometry_msgs::Pose> waypoints;
            trajectoryFromFile(waypoints,filePath);

            robot.runMeasurementRoutine(waypoints,false,true,constrainedYaw ? 0.0 : -1.0,visibilityConstraint,false);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("Unable to load trajectory from file !");
            throw std::runtime_error("INVALID TRAJECTORY FILE");
        }
    }

    ros::shutdown();
    return 0;
}
