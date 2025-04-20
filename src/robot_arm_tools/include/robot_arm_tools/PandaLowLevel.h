/**
 * \file PandaLowLevel.h
 * \brief Header file of the RobotLowLevel plugin class for Panda robot arm.
 *
 * Header file of the RobotLowLevel plugin class for Panda robot arm.
 * Defines the methods used to handle low level Panda robot arm communication and actions.
 *
 */

#pragma once
#include "robot_arm_tools/RobotLowLevel.h"

/*! \class PandaLowLevel
 * \brief Defines the methods used to handle low level Panda robot arm communication and actions..
 */
class PandaLowLevel : public RobotLowLevel 
{
    /*!
    * \brief Triggers robot arm recovery procedure
    */
    virtual void recovery();
};