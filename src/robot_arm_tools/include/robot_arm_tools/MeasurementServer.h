/**
 * \file MeasurementServer.h
 * \brief Header file of the MeasurementServer abstract class
 *
 * Header file of the MeasurementServer abstract class - Defines a generic interface for measurments ROS service servers
 *
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

#include "robot_arm_tools/Int.h"

 /*! \class MeasurementServer
  * \brief Generic interface for measurments ROS service servers
  */
class MeasurementServer
{
    public:
        /*!
         *  \brief Constructor
         *  \param measurementServerName The name of the measurement ROS service server
         *  \param measurementServerDisplay Wether to display measurements or not
         *  \param measurementServerStorageFolder The name of the measurement ROS service storage folder
         */
        MeasurementServer(std::string measurementServerName, bool measurementServerDisplay = false, std::string measurementServerStorageFolder = "/tmp/Measurements/");

        /*!
         *  \brief Constructor
         */
        MeasurementServer();

        /*!
         *  \brief Destructor
         */
        virtual ~MeasurementServer(){};

        /*!
         *  \brief Triggers the measurement, *virtual* method, returns true by default.
         *  \return True if the measurement succeeded, false otherwise.
         */
        virtual bool measure(){return true;}

        /*!
         *  \brief Triggers the measuring device recovery, *virtual* method, returns true by default.
         *  \return True if the recovery succeeded, false otherwise. 
         */
        virtual bool recovery(){return true;}

    protected:
        ros::NodeHandle m_nodeHandle;   /*!< ROS node handle */

        std::string m_measurementServerName;    /*!< ROS serivce name */
        bool m_measurementServerDisplay;    /*!< ROS service measurement display option */
        std::string m_measurementServerStorageFolder;   /*!< ROS service measurement storage folder */
        int m_measurementServerCounter;   /*!< ROS service measurement counter */

        ros::ServiceServer m_measurementServer;    /*!< ROS service used to trigger the measurement */
        ros::ServiceServer m_measurementCounterServer;  /*!< ROS service used to update the measurements counter*/

    private:
        /*!
         *  \brief ROS service callback - Triggers the measurement and eventual recovery
         *  \param req ROS Empty service request.
         *  \param res ROS Empty service response.
         */
        bool m_measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        /*!
         *  \brief ROS service callback - Updates the measurements counter after an eventual recovery/user action
         *  \param req Custom robot_arm_tool Int service request.
         *  \param res Custom robot_arm_tool Int service response.
         */
        bool m_updateCounter(robot_arm_tools::Int::Request & req, robot_arm_tools::Int::Response &res);
};
