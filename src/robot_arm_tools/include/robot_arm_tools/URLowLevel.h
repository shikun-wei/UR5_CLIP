/**
 * \file URLowLevel.h
 * \brief Header file of the RobotLowLevel plugin class for UR robot arm.
 *
 * Header file of the RobotLowLevel plugin class for UR robot arm.
 * Defines the methods used to handle low level UR robot arm communication and actions.
 *
 */

#pragma once
#include "robot_arm_tools/RobotLowLevel.h"

/*! \class URLowLevel
 * \brief Defines the methods used to handle low level UR robot arm communication and actions..
 */
class URLowLevel : public RobotLowLevel 
{
    /*!
    * \brief Triggers robot arm startup
    */
    virtual void triggerStartup();

    /*!
    * \brief Triggers robot arm shutdown
    */
    virtual void triggerShutdown();

    /*!
    * \brief Triggers robot arm recovery procedure
    */
    virtual void recovery();
};