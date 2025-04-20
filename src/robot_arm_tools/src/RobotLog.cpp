#include "robot_arm_tools/RobotLog.h"
#include "robot_arm_tools/RobotTrajectories.h"

#include <fstream>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <QApplication>
#include <QMessageBox>
#include <QAbstractButton>

//TODO Update waypoints in log file rather than storing index ?

RobotLog::RobotLog(std::vector<geometry_msgs::Pose>& waypoints, std::string filePath) : m_filePath(filePath), m_currentIndex(0),m_isRecovered(false), m_isRemoved(false)
{
    if(m_filePath == "")
    {
        m_filePath = "/tmp/" + ros::this_node::getName() + "_robot_log.yaml";
    }

    m_isRecovered = tryTrajectoryRecovery();
    if(m_isRecovered)
    {
        waypoints.clear();
        trajectoryFromFile(waypoints,m_filePath,m_currentIndex);
    }
    else
    {
        saveTrajectory(waypoints);
    }
    save();
}

RobotLog::RobotLog(std::vector<std::vector<double>>& waypoints, std::string filePath) : m_filePath(filePath), m_currentIndex(0), m_isRecovered(false), m_isRemoved(false)
{
    if(m_filePath == "")
    {
        m_filePath = "/tmp/" + ros::this_node::getName() + "_robot_log.yaml";
    }

    m_isRecovered = tryTrajectoryRecovery();
    if(m_isRecovered)
    {
        waypoints.clear();
        trajectoryFromFile(waypoints,m_filePath,m_currentIndex);
    }
    else
    {
        saveTrajectory(waypoints);
    }
    save();
}

RobotLog::~RobotLog()
{
    save();
}

bool RobotLog::tryTrajectoryRecovery()
{
    try
    {
        //Loading default log file
        m_node = YAML::LoadFile(m_filePath);

        //Yes/No choice window for trajectory recovery
        int tmp = 0;
        QApplication app(tmp,nullptr);
        QMessageBox msgBox;
        msgBox.setText("A log file has been found - Load recovery trajectory ?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.button(QMessageBox::Yes)->animateClick(5000);   //Default -> recover
        int ret = msgBox.exec();

        if(ret == QMessageBox::Yes)
        {
            //Recovering trajectory from log file
            ROS_INFO("Recovering trajectory...");
            m_currentIndex = m_node["current_index"].as<int>();
            return true;
        }
        else
        {
            //Wiping log file
            std::ofstream {m_filePath};
        }
    }
    catch(const std::exception& e)
    {
        //Creating default log file from scratch
        std::experimental::filesystem::path path(m_filePath);
        std::experimental::filesystem::create_directories(path.parent_path());
        std::ofstream {m_filePath};
    }

    //Yes/No choice window for log creation
    int tmp = 0;
    QApplication app(tmp,nullptr);
    QMessageBox msgBox;
    msgBox.setText("Create log file for current trajectory ?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.button(QMessageBox::Yes)->animateClick(5000);   //Default -> create
    int ret = msgBox.exec();

    if(ret == QMessageBox::Yes)
    {
        //Loading new/wiped log file
        m_node = YAML::LoadFile(m_filePath);        
        ROS_INFO("Creating log file at %s",m_filePath.c_str());
        m_node["current_index"] = m_currentIndex;
    }
    else
    {
        //Null YAML node
        m_node = YAML::Load("");
        remove();
    }

    return false;
}

void RobotLog::saveTrajectory(std::vector<std::vector<double>>& waypoints)
{
    if(m_node.Type() != YAML::NodeType::Null)
    {
        for (std::vector<std::vector<double>>::iterator it = waypoints.begin(); it != waypoints.end(); it++)
        {
            m_node["jointsValues"].push_back(*it);
        }
    }
}

void RobotLog::saveTrajectory(std::vector<geometry_msgs::Pose>& waypoints)
{
    if(m_node.Type() != YAML::NodeType::Null)
    {
        tf2::Quaternion tmpQuaternion;
        tf2::Matrix3x3 tmpMatrix;
        std::vector<double> tmpVector;
        double roll, pitch, yaw;
        for (std::vector<geometry_msgs::Pose>::iterator it = waypoints.begin(); it != waypoints.end(); it++)
        {
            tmpVector.push_back(it->position.x);
            tmpVector.push_back(it->position.y);
            tmpVector.push_back(it->position.z);

            tf2::fromMsg(it->orientation,tmpQuaternion);
            tmpMatrix = tf2::Matrix3x3(tmpQuaternion);
            tmpMatrix.getRPY(roll,pitch,yaw);

            tmpVector.push_back(roll);
            tmpVector.push_back(pitch);
            tmpVector.push_back(yaw);

            m_node["poses"].push_back(tmpVector);
            tmpVector.clear();
        }
    }
}

void RobotLog::remove()
{
    std::experimental::filesystem::remove(m_filePath);
    m_currentIndex = 0;
    m_isRemoved = true;
}

void RobotLog::save()
{
    if(m_node.Type() != YAML::NodeType::Null && !m_isRemoved)
    {
        std::ofstream fout(m_filePath);
        fout << m_node;
    }
}

void RobotLog::setCurrentIndex(int newIndex)
{
    if(m_node.Type() != YAML::NodeType::Null)
    {
        m_currentIndex = newIndex;
        m_node["current_index"] = m_currentIndex; 
        save();
    }
}

int RobotLog::getCurrentIndex()
{
    return(m_currentIndex);
}

void RobotLog::iterateCurrentIndex()
{
    if(m_node.Type() != YAML::NodeType::Null)
    {
        m_currentIndex++;
        m_node["current_index"] = m_currentIndex;
        save();
    }
}

bool RobotLog::isRecovered()
{
    return(m_isRecovered);
}

void RobotLog::addFaultyWaypoint(int waypointIndex, std::string reason)
{
    m_node[reason + "_index"].push_back(waypointIndex);
}