#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/PandaGripper.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ur_msgs/SetIO.h>

// === 全局变量 ===
geometry_msgs::Point32 target_point;
bool target_received = false;
ros::Subscriber target_sub;

// 坐标变换函数
geometry_msgs::Point32 transformToRobotFrame(const geometry_msgs::Point32& pt) {
    geometry_msgs::Point32 result;
    result.x = -0.45 - pt.y - 0.11;
    result.y = -0.586 + pt.x + 0.071;
    result.z = 0.55 - 0.09;
    return result;
}

// 回调函数
void targetCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    if (!msg->points.empty() && !target_received) {
        target_point = transformToRobotFrame(msg->points[0]);
        target_received = true;
        target_sub.shutdown();  // 只处理一次

        ROS_INFO("Transformed target point received: x=%.4f, y=%.4f, z=%.4f",
                 target_point.x, target_point.y, target_point.z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_arm_target_grasp_node");
    ros::AsyncSpinner spinner(0);
    ros::NodeHandle nh;

    spinner.start();
    ros::WallDuration(1.0).sleep();

    // 初始化机器人和可视化
    Robot robot;
    RobotVisualTools visualTools;
    robot.init();

    // 打开夹爪
    ros::ServiceClient ioClient = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    ur_msgs::SetIO ioSrv;
    ioSrv.request.fun = 1;
    ioSrv.request.pin = 16;
    ioSrv.request.state = 0.0;
    if (ioClient.call(ioSrv)) {
        ROS_INFO("🔓 Gripper opened: %s", ioSrv.response.success ? "true" : "false");
    } else {
        ROS_WARN("Failed to open gripper");
    }

    // 订阅目标点
    target_sub = nh.subscribe("/target", 1, targetCallback);
    ROS_INFO("Waiting for a single target point...");
    while (ros::ok() && !target_received) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // 创建姿态（抓取点）
    geometry_msgs::Pose graspPose;
    graspPose.position.x = target_point.x;
    graspPose.position.y = target_point.y;
    graspPose.position.z = target_point.z;

    double roll = 0.0, pitch = 0.0, yaw = -M_PI / 2.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    tf2::convert(q, graspPose.orientation);

    // 可视化目标
    visualTools.addSphere("grasp_sphere", graspPose, 0.05, false);

    // 抓取前预备点
    geometry_msgs::Pose preGraspPose = graspPose;
    preGraspPose.position.x += 0.10;

    // 抓取后提升点（上方 15cm）
    geometry_msgs::Pose postGraspPose = graspPose;
    postGraspPose.position.z += 0.3;

    // 放置点（x 方向 +20cm）
    geometry_msgs::Pose placePose = postGraspPose;
    placePose.position.z -= 0.25;
    placePose.position.y += 0.3;

    // ========== 移动流程 ==========

    // 1. 移动到预备点
    ROS_INFO("Moving to pre-grasp position...");
    robot.goToTarget(preGraspPose, true, true, true, false);

    // 2. 移动到抓取点
    ROS_INFO("Moving to grasp position...");
    robot.goToTarget(graspPose, true, true, true, false);

    // 3. 关闭夹爪（抓取）
    ioSrv.request.state = 1.0;
    if (ioClient.call(ioSrv)) {
        ROS_INFO("🛠️ Gripper closed: %s", ioSrv.response.success ? "true" : "false");
    } else {
        ROS_WARN("Failed to close gripper");
    }

    // 4. 抬起（提升点）
    ROS_INFO("Lifting object...");
    robot.goToTarget(postGraspPose, true, true, true, false);

    // 5. 移动到放置点
    ROS_INFO("Moving to place position...");
    robot.goToTarget(placePose, true, true, true, false);

    // 6. 打开夹爪（放下物体）
    ioSrv.request.state = 0.0;
    if (ioClient.call(ioSrv)) {
        ROS_INFO("📤 Gripper opened to place object: %s", ioSrv.response.success ? "true" : "false");
    } else {
        ROS_WARN("Failed to open gripper at place position");
    }

    // ==============================

    // 打印状态
    tf2::Matrix3x3(tf2::Quaternion(
        graspPose.orientation.x,
        graspPose.orientation.y,
        graspPose.orientation.z,
        graspPose.orientation.w)).getRPY(roll, pitch, yaw);
    ROS_INFO("📍 Grasp Pose: x=%.4f, y=%.4f, z=%.4f | rpy: %.2f, %.2f, %.2f",
             graspPose.position.x, graspPose.position.y, graspPose.position.z,
             roll, pitch, yaw);

    geometry_msgs::Pose actualPose = robot.getCurrentPose();
    ROS_INFO("📌 Actual pose: x=%.4f, y=%.4f, z=%.4f", actualPose.position.x, actualPose.position.y, actualPose.position.z);

    std::vector<double> jointsValues = robot.getCurrentJointsValues();
    for (size_t i = 0; i < jointsValues.size(); ++i) {
        ROS_INFO("Joint %lu: %.4f", i, jointsValues[i]);
    }

    // 清理可视化
    visualTools.deleteObject("grasp_sphere");

    ros::shutdown();
    return 0;
}
