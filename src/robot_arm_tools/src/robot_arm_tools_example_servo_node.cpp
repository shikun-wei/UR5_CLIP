#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_manager.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>

#include <interactive_markers/interactive_marker_server.h>

#include <math.h>

#include <tf/transform_listener.h>  //For twist listener

#define LINEAR_MIN 0.001
#define LINEAR_MAX 1.0
#define ANGULAR_MIN 0.0001
#define ANGULAR_MAX 0.1

#define sign(x)  (x < 0) ? -1 : (x > 0)

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world"; //TODO Change world to robot base link name
    transformStamped.child_frame_id = "interactive_marker";

    transformStamped.transform.translation.x = feedback->pose.position.x;
    transformStamped.transform.translation.y = feedback->pose.position.y;
    transformStamped.transform.translation.z = feedback->pose.position.z;
    transformStamped.transform.rotation = feedback->pose.orientation;

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_tools_example_servo_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;
    std::string endEffectorName = robot.getEndEffectorName();

    //Robot visual tools initialisation
    RobotVisualTools visualTools;
    std::string baseLinkName = visualTools.getBaseFrame();

    ros::NodeHandle nh;   

    //Perform trajectory
    do 
    {
        ROS_INFO("Switch the robot to manual control mode, and move to a non-singluar pose - Press enter to continue");
    } while (std::cin.get() != '\n');

    //Switch to position/velocity controller instead of trajectory controller
    std::string servoMode = nh.param<std::string>("servoMode", "position");
    if(servoMode == "velocity")
    {
        robot.switchController({"joint_group_vel_controller"},{"scaled_pos_joint_traj_controller", "pos_joint_traj_controller","position_joint_trajectory_controller"});
    }
    else
    {
        robot.switchController({"joint_group_pos_controller"},{"scaled_pos_joint_traj_controller","pos_joint_traj_controller","position_joint_trajectory_controller"});
    }
    //TODO get all joint trajectory controllers names from controller manager ?

    interactive_markers::InteractiveMarkerServer server("interactive_marker", "", true);

    visualization_msgs::InteractiveMarker interactiveMarker;

    interactiveMarker.header.frame_id = "world";
    interactiveMarker.pose = robot.getCurrentPose();
    
    interactiveMarker.scale = 1;
    interactiveMarker.name = "interactive_marker";

    visualization_msgs::InteractiveMarkerControl controlA,controlB;
    controlA.always_visible = true;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    controlA.markers.push_back(marker);
    interactiveMarker.controls.push_back(controlA);
    interactiveMarker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    controlB.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

    controlB.orientation.w = 1;
    controlB.orientation.x = 1;
    controlB.orientation.y = 0;
    controlB.orientation.z = 0;
    controlB.name = "rotate_x";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(controlB);
    controlB.name = "move_x";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(controlB);

    controlB.orientation.w = 1;
    controlB.orientation.x = 0;
    controlB.orientation.y = 1;
    controlB.orientation.z = 0;
    controlB.name = "rotate_z";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(controlB);
    controlB.name = "move_z";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(controlB);

    controlB.orientation.w = 1;
    controlB.orientation.x = 0;
    controlB.orientation.y = 0;
    controlB.orientation.z = 1;
    controlB.name = "rotate_y";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(controlB);
    controlB.name = "move_y";
    controlB.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(controlB);

    server.insert(interactiveMarker);
    server.setCallback(interactiveMarker.name,&processFeedback);
    server.applyChanges();

    geometry_msgs::Twist markerTwist;
    tf::StampedTransform markerTransform;
    tf::StampedTransform robotTransform;
    tf::TransformListener listener;

    ros::Publisher twistCommandPublisher = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);
    geometry_msgs::TwistStamped twistCommand;

    ros::Rate rate(1000);
    while (nh.ok())
    {
        if(servoMode == "velocity")
        {
            try
            {
                listener.lookupTwist("interactive_marker", baseLinkName, "interactive_marker", tf::Point(), "interactive_marker", ros::Time(0), ros::Duration(0.01), markerTwist);
            }
            catch(tf::TransformException ex)
            {
                continue;
            }

            twistCommand.header.stamp = ros::Time::now();

            twistCommand.twist.linear.x = 0.1*markerTwist.linear.x;
            twistCommand.twist.linear.y = 0.1*markerTwist.linear.y;
            twistCommand.twist.linear.z = 0.1*markerTwist.linear.z;
            twistCommand.twist.angular.x = 0.1*markerTwist.angular.x;
            twistCommand.twist.angular.y = 0.1*markerTwist.angular.y;
            twistCommand.twist.angular.z = 0.1*markerTwist.angular.z;
        }

        else
        {
            try
            {
                listener.lookupTransform(baseLinkName, "interactive_marker", ros::Time(0), markerTransform);
                listener.lookupTransform(baseLinkName, endEffectorName, ros::Time(0), robotTransform);
            }
            catch(tf::TransformException ex)
            {
                continue;
            }

            twistCommand.header.stamp = ros::Time::now();
            twistCommand.twist.linear.x = - robotTransform.getOrigin().getX() + markerTransform.getOrigin().getX();
            twistCommand.twist.linear.y = - robotTransform.getOrigin().getY() + markerTransform.getOrigin().getY();
            twistCommand.twist.linear.z = - robotTransform.getOrigin().getZ() + markerTransform.getOrigin().getZ();
            twistCommand.twist.angular.x = 0.0;
            twistCommand.twist.angular.y = 0.0;
            twistCommand.twist.angular.z = 0.0;
        }
        
        if(abs(twistCommand.twist.linear.x) > LINEAR_MAX)
        {
            twistCommand.twist.linear.x = sign(twistCommand.twist.linear.x)*LINEAR_MAX;
        }
        else if(abs(twistCommand.twist.linear.x) < LINEAR_MIN)
        {
            twistCommand.twist.linear.x = 0;
        }
        if(abs(twistCommand.twist.linear.y) > LINEAR_MAX)
        {
            twistCommand.twist.linear.y = sign(twistCommand.twist.linear.y)*LINEAR_MAX;
        }
        else if(abs(twistCommand.twist.linear.y) < LINEAR_MIN)
        {
            twistCommand.twist.linear.y = 0;
        }
        if(abs(twistCommand.twist.linear.z) > LINEAR_MAX)
        {
            twistCommand.twist.linear.z = sign(twistCommand.twist.linear.z)*LINEAR_MAX;
        }
        else if(abs(twistCommand.twist.linear.z) < LINEAR_MIN)
        {
            twistCommand.twist.linear.z = 0;
        }

        if(abs(twistCommand.twist.angular.x) > ANGULAR_MAX)
        {
            twistCommand.twist.angular.x = sign(twistCommand.twist.angular.x)*ANGULAR_MAX;
        }
        else if(abs(twistCommand.twist.angular.x) < ANGULAR_MIN)
        {
            twistCommand.twist.angular.x = 0;
        }
        if(abs(twistCommand.twist.angular.y) > ANGULAR_MAX)
        {
            twistCommand.twist.angular.y = sign(twistCommand.twist.angular.y)*ANGULAR_MAX;
        }
        else if(abs(twistCommand.twist.angular.y) < ANGULAR_MIN)
        {
            twistCommand.twist.angular.y = 0;
        }
        if(abs(twistCommand.twist.angular.z) > ANGULAR_MAX)
        {
            twistCommand.twist.angular.z = sign(twistCommand.twist.angular.z)*ANGULAR_MAX;
        }
        else if(abs(twistCommand.twist.angular.z) < ANGULAR_MIN)
        {
            twistCommand.twist.angular.z = 0;
        }

        twistCommandPublisher.publish(twistCommand);
        ros::spinOnce();
        rate.sleep();
    }
   
    ros::waitForShutdown();
    return 0;
}