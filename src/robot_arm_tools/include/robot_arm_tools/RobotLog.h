/**
 * \file RobotLog.h
 * \brief Header file of the RobotLog plugin class.
 *
 * Header file of the RobotLog plugin class.
 * Defines the methods used to handle the robot progression logging along a list of waypoints.
 *
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

class RobotLog
{
    public:

        /*!
        * \brief Constructor (Poses case).
        * \param waypoints The vector containing the waypoints.
        * \param filePath The log file path (.yaml).
        */
        RobotLog(std::vector<geometry_msgs::Pose>& waypoints, std::string filePath = "");

        /*!
        * \brief Constructor (Joints values case).
        * \param waypoints The vector containing the waypoints.
        * \param filePath The log file path (.yaml).
        */
        RobotLog(std::vector<std::vector<double>>& waypoints, std::string filePath = "");

        /*!
        * \brief Destructor.
        */
        ~RobotLog();

        /*!
         * \brief Tries to recover trajectory from log file depending on user input.
         * \return True if a trajectory was recovered, false otherwise.
         */
        bool tryTrajectoryRecovery();

        /*!
         * \brief Saves a joint values trajectory in the log file. 
         * \param waypoints The joints values waypoints list.
         */
        void saveTrajectory(std::vector<std::vector<double>>& waypoints);

        /*!
         * \brief Saves a poses trajectory in the log file. 
         * \param waypoints The poses waypoints list.
         */
        void saveTrajectory(std::vector<geometry_msgs::Pose>& waypoints);

        /*!
         * \brief Removes log file
         */
        void remove();

        /*!
         * \brief Saves log file
         */
        void save();

        /*!
         * \brief Sets the current index value.
         * \param newIndex The new current index value.
         */
        void setCurrentIndex(int newIndex);

        /*!
         * \brief Gets the current index value.
         * \return The current index value.
         */
        int getCurrentIndex();

        /*!
         * \brief Iterates current index by one.
         */
        void iterateCurrentIndex();

        /*!
         * \brief Returns the recovery status of the current trajectory.
         * \return True if the current trajectory is recovered, false otherwise.
         */
        bool isRecovered();

        /*!
         * \brief Marks a given waypoint as faulty for some reason
         * \param waypointIndex The waypoint index
         * \param reason The fault reason (optionnal)
         */
        void addFaultyWaypoint(int waypointIndex, std::string reason = "faulty");

    private:

        YAML::Node m_node;   /*!< The yaml node in which the log is stored */
        std::string m_filePath; /*!< The log file path */
        int m_currentIndex; /*!< The current robot progression index along the waypoints */

        bool m_isRecovered;  /*!< Wether the current trajectory is recovered or not */

        bool m_isRemoved;   /*!< Wether the log file is removed or not */
};