#include "robot_arm_tools/RobotTrajectories.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

void sphericInclinationTrajectory(geometry_msgs::Pose spherePose, double sphereRadius, double inclination, double azimuthMin, double azimuthMax, int waypointsNumber, std::vector<geometry_msgs::Pose> &waypoints, bool inward, double yawOffset)
{
    geometry_msgs::Pose currentPose;

    // General case
    if (waypointsNumber > 1)
    {
        // Compute azimuth delta
        double dazimuth = (azimuthMax - azimuthMin) - 2 * M_PI * floor((azimuthMax - azimuthMin) / (2 * M_PI));

        if (dazimuth != 0)
        {
            dazimuth /= waypointsNumber - 1;
        }

        // Complete circle: avoid double initial measurement !
        else
        {
            dazimuth = (azimuthMax - azimuthMin) / waypointsNumber;
        }

        // Waypoints filling loop
        for (int i = 0; i < waypointsNumber; i++)
        {
            currentPose = sphericPose(spherePose, sphereRadius, inclination, i * dazimuth + azimuthMin, inward, yawOffset);
            waypoints.push_back(currentPose);
        }
    }

    // Only one waypoint case
    else
    {
        currentPose = sphericPose(spherePose, sphereRadius, inclination, (azimuthMin + azimuthMax) / 2, inward, yawOffset);
        waypoints.push_back(currentPose);
    }
}

void sphericAzimuthTrajectory(geometry_msgs::Pose spherePose, double sphereRadius, double azimuth, double inclinationMin, double inclinationMax, int waypointsNumber, std::vector<geometry_msgs::Pose> &waypoints, bool inward, double yawOffset)
{
    geometry_msgs::Pose currentPose;

    // General case
    if (waypointsNumber > 1)
    {
        // Compute inclination delta
        double dinclination = (inclinationMax - inclinationMin) - 2 * M_PI * floor((inclinationMax - inclinationMin) / (2 * M_PI));

        if (dinclination != 0)
        {
            dinclination /= waypointsNumber - 1;
        }

        // Complete circle: avoid double initial measurement !
        else
        {
            dinclination = (inclinationMax - inclinationMin) / waypointsNumber;
        }

        // Waypoints filling loop
        for (int i = 0; i < waypointsNumber; i++)
        {
            currentPose = sphericPose(spherePose, sphereRadius, i * dinclination + inclinationMin, azimuth, inward, yawOffset);
            waypoints.push_back(currentPose);
        }
    }

    // Only one waypoint case
    else
    {
        currentPose = sphericPose(spherePose, sphereRadius, (inclinationMin + inclinationMax) / 2, azimuth, inward, yawOffset);
        waypoints.push_back(currentPose);
    }
}

void straightTrajectory(geometry_msgs::Pose startingPose, geometry_msgs::Pose endingPose, int waypointsNumber, std::vector<geometry_msgs::Pose> &waypoints)
{
    // Compute translation deltas
    double dx, dy, dz, dRx, dRy, dRz;

    dx = (endingPose.position.x - startingPose.position.x) / (waypointsNumber - 1);
    dy = (endingPose.position.y - startingPose.position.y) / (waypointsNumber - 1);
    dz = (endingPose.position.z - startingPose.position.z) / (waypointsNumber - 1);

    // Compute rotation deltas
    double startingR, startingP, startingY;
    double endingR, endingP, endingY;

    tf2::Quaternion startingQuaternion, endingQuaternion, currentQuaternion;
    tf2::fromMsg(startingPose.orientation, startingQuaternion);
    tf2::fromMsg(endingPose.orientation, endingQuaternion);

    tf2::Matrix3x3 m(startingQuaternion);
    m.getRPY(startingR, startingP, startingY);
    m = tf2::Matrix3x3(endingQuaternion);
    m.getRPY(endingR, endingP, endingY);

    dRx = (endingR - startingR) / (waypointsNumber - 1);
    dRy = (endingP - startingP) / (waypointsNumber - 1);
    dRz = (endingY - startingY) / (waypointsNumber - 1);

    // Initialise first waypoint at the starting pose
    geometry_msgs::Pose currentPose = startingPose;

    double currentR, currentP, currentY;
    currentR = startingR;
    currentP = startingP;
    currentY = startingY;

    waypoints.push_back(currentPose);

    // Waypoints filling loop
    for (int i = 0; i < waypointsNumber - 1; i++)
    {
        // Translation
        currentPose.position.x += dx;
        currentPose.position.y += dy;
        currentPose.position.z += dz;

        // Rotation
        currentR += dRx;
        currentP += dRy;
        currentY += dRz;

        currentQuaternion.setRPY(currentR, currentP, currentY);
        currentPose.orientation = tf2::toMsg(currentQuaternion);

        waypoints.push_back(currentPose);
    }
}

geometry_msgs::Pose sphericPose(geometry_msgs::Pose spherePose, double sphereRadius, double poseInclination, double poseAzimuth, bool inward, double yawOffset)
{
    geometry_msgs::Pose pose;
    tf2::Quaternion quaternion, offsetQuaternion;

    // Position
    pose.position.x = spherePose.position.x + sphereRadius * std::sin(poseInclination) * std::cos(poseAzimuth);
    pose.position.y = spherePose.position.y + sphereRadius * std::sin(poseInclination) * std::sin(poseAzimuth);
    pose.position.z = spherePose.position.z + sphereRadius * std::cos(poseInclination);

    // Orientation
    inward ? quaternion.setRPY(M_PI, poseInclination, poseAzimuth) : quaternion.setRPY(0, poseInclination, poseAzimuth);
    offsetQuaternion.setRPY(0, 0, yawOffset);
    pose.orientation = tf2::toMsg(quaternion * offsetQuaternion);

    return (pose);
}

void rotateTrajectory(std::vector<geometry_msgs::Pose> &waypoints, geometry_msgs::Point rotationCenter, double rotationRoll, double rotationPitch, double rotationYaw)
{
    // Build the corresponding rotation matrix
    tf2::Matrix3x3 rotationMatrix;
    rotationMatrix.setRPY(rotationRoll, rotationPitch, rotationYaw);

    tf2::Vector3 posePosition;
    tf2::Quaternion poseOrientation;
    tf2::Matrix3x3 poseMatrix;

    for (int i = 0; i < waypoints.size(); i++)
    {
        posePosition = tf2::Vector3(waypoints[i].position.x - rotationCenter.x, waypoints[i].position.y - rotationCenter.y, waypoints[i].position.z - rotationCenter.z);
        posePosition = rotationMatrix * posePosition;

        tf2::fromMsg(waypoints[i].orientation, poseOrientation);
        poseMatrix = tf2::Matrix3x3(poseOrientation);

        (rotationMatrix * poseMatrix).getRotation(poseOrientation);

        //double roll, pitch, yaw;
        //std::cout << "(Before) Pose " << i << " : " << waypoints[i].position.x << " " << waypoints[i].position.y << " " << waypoints[i].position.z << " " << roll << " " << pitch << " " << yaw << std::endl;

        waypoints[i].position.x = rotationCenter.x + posePosition.getX();
        waypoints[i].position.y = rotationCenter.y + posePosition.getY();
        waypoints[i].position.z = rotationCenter.z + posePosition.getZ();

        waypoints[i].orientation = tf2::toMsg(poseOrientation);

        //tf2::fromMsg(waypoints[i].orientation, poseOrientation);
        //tf2::Matrix3x3(poseOrientation).getRPY(roll, pitch, yaw);
        //std::cout << "(After) Pose " << i << " : " << waypoints[i].position.x << " " << waypoints[i].position.y << " " << waypoints[i].position.z << " " << roll << " " << pitch << " " << yaw << std::endl;
    }
}

void translateTrajectory(std::vector<geometry_msgs::Pose> &waypoints, double translationX, double translationY, double translationZ)
{
    for (int i = 0; i < waypoints.size(); i++)
    {
        waypoints[i].position.x += translationX;
        waypoints[i].position.y += translationY;
        waypoints[i].position.z += translationZ;
    }
}

void trajectoryFromFile(std::vector<geometry_msgs::Pose> &waypoints, std::string filePath, int startIndex, std::string waypointsKey)
{
    YAML::Node posesFile;
    try
    {
        posesFile = YAML::LoadFile(filePath);
        if (!posesFile[waypointsKey] || posesFile[waypointsKey].size() == 0)
        {
            throw std::runtime_error("INVALID POSES FILE");
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Unable to open poses file !");
        throw std::runtime_error("INVALID POSES VALUES FILE");
    }

    YAML::Node posesNode = posesFile[waypointsKey];
    std::vector<std::vector<double>> poses;
    std::vector<double> tmpVector;

    for (int i = startIndex; i < posesNode.size(); i++)
    {
        for (int j = 0; j < posesNode[i].size(); j++)
        {
            tmpVector.push_back(posesNode[i][j].as<double>());
        }
        poses.push_back(tmpVector);
        tmpVector.clear();
    }

    geometry_msgs::Pose tmpPose;
    tf2::Quaternion tmpQuaternion;
    for (std::vector<std::vector<double>>::iterator it = poses.begin(); it != poses.end(); it++)
    {
        tmpPose.position.x = (*it)[0];
        tmpPose.position.y = (*it)[1];
        tmpPose.position.z = (*it)[2];
        tmpQuaternion.setRPY((*it)[3], (*it)[4], (*it)[5]);
        tmpPose.orientation = tf2::toMsg(tmpQuaternion);
        waypoints.push_back(tmpPose);
    }
}

void trajectoryFromParameter(std::vector<geometry_msgs::Pose> &waypoints, std::string parameterName, int startIndex)
{
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue tmpPoses;
    geometry_msgs::Pose tmpPose;
    tf2::Quaternion tmpQuaternion;

    if (!nh.getParam(parameterName, tmpPoses))
    {
        ROS_ERROR("Unable to retrieve waypoints poses !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    for (int i = startIndex; i < tmpPoses.size(); i++)
    {
        tmpPose.position.x = tmpPoses[i][0];
        tmpPose.position.y = tmpPoses[i][1];
        tmpPose.position.z = tmpPoses[i][2];
        tmpQuaternion.setRPY(tmpPoses[i][3], tmpPoses[i][4], tmpPoses[i][5]);
        tmpPose.orientation = tf2::toMsg(tmpQuaternion);
        waypoints.push_back(tmpPose);
    }
}

void trajectoryFromFile(std::vector<std::vector<double>> &waypoints, std::string filePath, int startIndex, std::string waypointsKey)
{
    YAML::Node jointsFile;
    try
    {
        jointsFile = YAML::LoadFile(filePath);
        if (!jointsFile[waypointsKey] || jointsFile[waypointsKey].size() == 0)
        {
            throw std::runtime_error("INVALID JOINTS VALUES FILE");
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Unable to open joints values file !");
        throw std::runtime_error("INVALID JOINTS VALUES FILE");
    }

    YAML::Node jointsNode = jointsFile[waypointsKey];
    std::vector<double> tmpVector;

    for (int i = startIndex; i < jointsNode.size(); i++)
    {
        for (int j = 0; j < jointsNode[i].size(); j++)
        {
            tmpVector.push_back(jointsNode[i][j].as<double>());
        }
        waypoints.push_back(tmpVector);
        tmpVector.clear();
    }
}

void trajectoryFromParameter(std::vector<std::vector<double>> &waypoints, std::string parameterName, int startIndex)
{
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue tmpJointsValues;

    if (!nh.getParam(parameterName, tmpJointsValues))
    {
        ROS_ERROR("Unable to retrieve waypoints joints values !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    std::vector<double> tmpVector;
    for (int i = startIndex; i < tmpJointsValues.size(); i++)
    {
        for (int j = 0; j < tmpJointsValues[i].size(); j++)
        {
            tmpVector.push_back(tmpJointsValues[i][j]);
        }
        waypoints.push_back(tmpVector);
        tmpVector.clear();
    }
}