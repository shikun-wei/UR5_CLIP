/**
 * \file RobotLowLevel.h
 * \brief Header file of the RobotLowLevel plugin class.
 *
 * Header file of the RobotLowLevel plugin class.
 * Defines the methods used to handle low level robot arm communication and actions.
 *
 */

#pragma once
#include <ros/ros.h>

/*! \class RobotLowLevel
 * \brief Defines the methods used to handle low level robot arm communication and actions.
 */
class RobotLowLevel
{
    public:

        /*!
         * \brief Constructor
         */
        RobotLowLevel(){};

        /*!
        * \brief Triggers robot arm startup
        */
        virtual void triggerStartup(){}

        /*!
        * \brief Triggers robot arm shutdown
        */
        virtual void triggerShutdown(){}

        /*!
        * \brief Triggers robot arm recovery procedure
        */
        virtual void recovery(){}

        /*!
        * \brief Destructor
        */
       virtual ~RobotLowLevel(){}

    protected:
        ros::NodeHandle m_nodeHandle;   /*!< ROS Node Handle */
};