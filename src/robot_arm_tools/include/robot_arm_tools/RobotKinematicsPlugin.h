/**
 * \file RobotKinematicsPlugin.h
 * \brief Header file of the RobotKinematicsPlugin class.
 *
 * Header file of the RobotKinematicsPlugin class.
 * Defines the methods used to configure and use a robot arm kinematic model.
 *
 */

#pragma once

#include <vector>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

/*! \class RobotKinematicsPlugin
 * \brief Defines the methods used to configure and use a robot arm kinematic model.
 */
class RobotKinematicsPlugin : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
    public:

        /*!
         * \brief Default constructor
         */
        RobotKinematicsPlugin();

        /*!
         * \brief Constructor
         * \param jointsLimits The robot joints positions limits;
         * \param toolTransform The transformation lining the robot flange frame to the tool frame.
         * \param baseTransform The transformation lining the world frame to the robot base frame.
         */
        RobotKinematicsPlugin(std::vector<std::vector<double>> jointsLimits, geometry_msgs::Transform toolTransform = geometry_msgs::Transform(), geometry_msgs::Transform baseTransform = geometry_msgs::Transform());

        /*!
         * \brief Destructor
         */
        ~RobotKinematicsPlugin(){};

        /*!
         * \brief Generates a random set of joints values within joints positions limits.
         * \return A random set of joints values.
         */
        KDL::JntArray generateRandomJointsValues() const;

        /*!
         * \brief Generates a random set of joints values near by a specified set of joints values and within a specified set of maximum distances.
         * \param nearByJointsValues The near by set of joints values.
         * \param distanceToJointsValues The set of maximum distances.
         * \return A random set of joints values.
         */
        KDL::JntArray generateRandomJointsValuesNearBy(KDL::JntArray nearByJointsValues, std::vector<double> distanceToJointsValues) const;

        /*!
         * \brief [CUSTOM] Computes the forward kinematic model of the robot arm.
         * \param desiredJointsValues The desired joints values.
         * \return The corresponding pose.
         */
        geometry_msgs::Pose forwardKinematics(std::vector<double> desiredJointsValues) const;

        /*!
         * \brief [CUSTOM] Computes the inverse kinematic model of the robot arm.
         * \param desiredPose The desired pose.
         * \return The corresponding joints values.
         */
        std::vector<double> inverseKinematics(geometry_msgs::Pose desiredPose, std::vector<double> initialGuess = {}, double timeout = 0.05);

        // KDLKinematicsPlugin overriden methods

        bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles, std::vector<geometry_msgs::Pose>& poses) const override;

        bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name, const std::string& base_frame, const std::vector<std::string>& tip_frames, double search_discretization) override;

        const std::vector<std::string>& getJointNames() const override;

        const std::vector<std::string>& getLinkNames() const override;

    private:

        std::vector<int> m_jointsAxis; /*!< Joints axis */
        KDL::JntArray m_jointsLimitsMin, m_jointsLimitsMax;    /*!< Joints positions limits */

        KDL::Frame m_robotBase;    /*!< Robot base transformation matrix */
        KDL::Frame m_robotFlange;    /*!< Robot flange transformation matrix */
        std::vector<double> m_robotParameters;  /*!< Robot geometric parameters */

        ros::NodeHandle m_nodeHandle;   /*!< ROS node handle */

        KDL::Chain m_robotChain;    /*!< KDL robot kinematic chain */
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> m_solverFK; /*!< KDL robot forward kinematic solver */
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> m_solverIKV; /*!< KDL robot inverse velocity solver */
        std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> m_solverIK; /*!< KDL robot inverse kinematic solver */

        std::vector<double> m_lastJointsValues;   /*!< Last computed joints values */

        std::vector<std::string> m_jointsNames, m_linksNames;   /*!< KinematicsBase links and joints names */
};

