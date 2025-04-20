/**
 * \file PandaGripper.h
 * \brief Header file of the PandaGripper class.
 *
 * Header file of the PandaGripper class.
 * Defines the methods used to configure, monitor and operate a Franka gripper mounted on a Panda robot arm.
 *
 */

//TODO Pluginify !

#pragma once

#include "robot_arm_tools/Robot.h"
#include "robot_arm_tools/RobotVisualTools.h"

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>

/*! \class PandaGripper
 * \brief Defines the methods used to configure, monitor and operate a Franka gripper mounted on a Panda robot arm
 */
class PandaGripper
{
    public:

        /*!
         * \brief Constructor.
         * \param robot Constant reference to the robot the gripper will be mounted on.
         */
        PandaGripper(const Robot& robot);

        /*!
         * \brief Destructor.
         */
        ~PandaGripper(){};

        /*!
         * \brief Moves the gripper to its initial configuration.
         * \param visualCheck Wether or not to perform a visual check in RViz before motion execution.
         * \param executeMotion Wether or not to actually move the gripper.
         */
        void init(bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Closes the gripper with a given width closure target.
         * \param targetWidth The width closure target between the two fingers.
         * \param visualCheck Wether or not to perform a visual check in RViz before motion execution.
         * \param executeMotion Wether or not to actually move the gripper.
         */
        void close(double targetWidth = 0.0, bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Grasps an object with a given grasping force target.
         * \param targetForce The grasping force target.
         * \param visualCheck Wether or not to perform a visual check in RViz before motion execution.
         * \param executeMotion Wether or not to actually move the gripper.
         */
        void grasp(double targetForce, bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Fully opens the gripper.
         * \param visualCheck Wether or not to perform a visual check in RViz before motion execution.
         * \param executeMotion Wether or not to actually move the gripper.
         */
        void open(bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Sets the gripper valocity.
         * \param velocityRatio Velocity ratio - 1 means maximum acceleration
         */
        void setVelocity(double velocityRatio);

        /*!
         * \brief Sets the gripper acceleration.
         * \param accelerationRatio Acceleration ratio - 1 means maximum acceleration
         */
        void setAcceleration(double accelerationRatio);

    private:

        /*!
         * \brief Performs motion planning and execution to reach a target.
         * \param visualCheck Wether or not to perform a visual check in RViz before motion execution.
         * \param executeMotion Wether or not to actuelly move the gripper.
         */
        void moveGripper(bool visualCheck = true, bool executeMotion = true);

        moveit::planning_interface::MoveGroupInterface m_gripperMoveGroup; /*!< Panda gripper move group interface */
        
        actionlib::SimpleActionClient<franka_gripper::HomingAction> m_homingActionClient;   /*!< Gripper homing action client */
        actionlib::SimpleActionClient<franka_gripper::GraspAction> m_graspActionClient;    /*!< Gripper grasp action client */

        double m_velocityRatio; /*!< Gripper velocity ratio */

        RobotVisualTools m_visualTools; /*!< Visual tools used for motion visual checks */

        bool m_simulation;  /*!< Wether the gripper is simulated or not */

        ros::NodeHandle m_nodeHandle;    /*! ROS node handle*/
};