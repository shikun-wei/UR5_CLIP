#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ur_msgs/SetIO.h>
#include <cstdlib>

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
        target_sub.shutdown();

        ROS_INFO("Transformed target point received: x=%.4f, y=%.4f, z=%.4f",
                 target_point.x, target_point.y, target_point.z);
    }
}

// 自动尝试可达的位姿
bool tryReachablePose(Robot& robot, geometry_msgs::Pose poseTarget,
    double step = 0.005, int maxAttempts = 5)
        {   
        for (int i = 0; i < maxAttempts; ++i)
        {
        try {
        robot.goToTarget(poseTarget, true, true, true, false);
        ROS_INFO("✅ Pose reached on attempt %d", i + 1);
        return true;
        } catch (const std::exception& e) {
        // 仅微调 z 高度（向上试）
        poseTarget.position.z += step;
        ROS_WARN("⚠️ Attempt %d failed. Adjusting Z up by %.3f", i + 1, step);
        }
        }

        ROS_ERROR("❌ Failed to reach pose after %d attempts.", maxAttempts);
        return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_arm_target_grasp_node");
    ros::AsyncSpinner spinner(0);
    ros::NodeHandle nh;
    srand(time(NULL)); // 初始化随机数生成器

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
    ioClient.call(ioSrv);

    // 订阅目标点
    target_sub = nh.subscribe("/target", 1, targetCallback);
    ROS_INFO("Waiting for a single target point...");
    while (ros::ok() && !target_received) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // 设置姿态（抓取点）
    geometry_msgs::Pose graspPose;
    graspPose.position.x = target_point.x;
    graspPose.position.y = target_point.y;
    graspPose.position.z = target_point.z-0.03;

    double roll = 0.0, pitch = 0.0, yaw = -M_PI / 2.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    tf2::convert(q, graspPose.orientation);

    visualTools.addSphere("grasp_sphere", graspPose, 0.05, false);

    // 抓取前预备点
    geometry_msgs::Pose preGraspPose = graspPose;
    preGraspPose.position.x += 0.10;
    preGraspPose.position.z += 0.01;
    
    // 抓取后提升点（上方 15cm）
    geometry_msgs::Pose postGraspPose = graspPose;
    postGraspPose.position.x += 0.3;
    postGraspPose.position.z += 0.3;

    // 放置点（x 方向 +20cm）
    geometry_msgs::Pose placePose = postGraspPose;
    placePose.position.x  -= 0.35;
    placePose.position.z -= 0.18;
    placePose.position.y = 0.10 - graspPose.position.y;

    // === 依次尝试每个动作点 ===
    if (!tryReachablePose(robot, preGraspPose)) return 1;
    if (!tryReachablePose(robot, graspPose)) return 1;

    // 抓取动作（关闭夹爪）
    ioSrv.request.state = 1.0;
    ioClient.call(ioSrv);
    ROS_INFO("Gripper closed.");

    if (!tryReachablePose(robot, postGraspPose)) return 1;
    if (!tryReachablePose(robot, placePose)) return 1;

    // 放置物体（打开夹爪）
    ioSrv.request.state = 0.0;
    ioClient.call(ioSrv);
    ROS_INFO("Gripper opened.");

    // 清理可视化
    visualTools.deleteObject("grasp_sphere");

    ros::shutdown();
    return 0;
}
