/**
 * \file Robot.h
 * \brief Header file of the Robot class.
 *
 * Header file of the Robot class.
 * Defines the methods used to configure, monitor and operate an robot arm.
 *
 */

#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "robot_arm_tools/RobotVisualTools.h"
#include "robot_arm_tools/RobotLowLevel.h"

#include <actionlib/server/simple_action_server.h>
#include "robot_arm_tools/PoseTargetAction.h"
#include "robot_arm_tools/JointsValuesTargetAction.h"
#include "robot_arm_tools/PoseWaypointsAction.h"
#include "robot_arm_tools/JointsValuesWaypointsAction.h"

#include <vector>

#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Bool.h>

/*! \class Robot
 * \brief Defines the methods used to configure, monitor and operate an robot arm.
 */
class Robot
{
    public:

        /*!
         * \brief ROS parameters based constructor.
         */
        Robot();

        /*!
         * \brief Destructor.
         */
        ~Robot();

        /*!
         * \brief Sets up all the tools used to control the robot
         */
        void setupRobot();

        /*!
         * \brief Switches between two controllers
         * \param startController The name of the controller to start
         * \param stopController The name of the controller to stop
         */
        void switchController(std::vector<std::string> startController, std::vector<std::string> stopController);

        /*!
         * \brief Gets a copy of the PlanningScene associated to the robot
         * \return The robot PlanningScene
         */
        std::shared_ptr<planning_scene::PlanningScene> getPlanningScene() const;

        /*!
         * \brief Gets the RobotModel associated to the robot
         * \return The robot RobotModel
         */
        std::shared_ptr<const moveit::core::RobotModel> getRobotModel() const;

        /*!
         * \brief Gets the number of joints in the robot arm
         * \return Number of joints in the robot arm
         */
        int getJointsNumber() const;

        /*!
         * \brief Gets the robot arm geometrical joints limits
         * \return Robot arm geometrical joints limits
         */
        std::vector<std::vector<double>> getJointsLimits() const;

        /*!
         * \brief Gets the named joints configuration of the robot arm in its initial configuration.
         * \return The named joints configuration of the robot arm in its initial configuration.
         */
        std::map<std::string,double> getInitialNamedJointsValues() const;

        /*!
         * \brief Gets the joints configuration of the robot arm in its initial configuration.
         * \return The joints configuration of the robot arm in its initial configuration.
         */
        std::vector<double> getInitialJointsValues() const;

        /*!
         * \brief Gets the pose of the robot arm end effector in the robot arm initial configuration.
         * \return The pose of the robot arm end effector in the robot arm initial configuration.
         */
        geometry_msgs::Pose getInitialPose() const;

        /*!
         * \brief Moves the robot to its initial configuration.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         */
        void init(bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Sets the robot velocity.
         * \param velocityRatio Velocity ratio - 1 means maximum velocity
         */
        void setVelocity(double velocityRatio);

        /*!
         * \brief Sets the robot acceleration.
         * \param accelerationRatio Acceleration ratio - 1 means maximum acceleration
         */
        void setAcceleration(double accelerationRatio);

        /*!
         * \brief Gets the current pose of the robot arm end effector.
         * \return The current pose of the robot arm end effector.
         */
        geometry_msgs::Pose getCurrentPose();   //Cannot be const because of FK service call

        /*!
         * \brief Gets the current joints configuration of the robot.
         * \return The current joints configuration of the robot.
         */
        std::vector<double> getCurrentJointsValues() const;

        /*!
         * \brief Moves the robot to the requested named joints target.
         * \param namedJointsTargetValues The named joints target.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         */
        void goToTarget(std::map<std::string,double> namedJointsTargetValues, bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Moves the robot to the requested joints target.
         * \param jointsTargetValues The joints target.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         */
        void goToTarget(std::vector<double> jointsTargetValues, bool visualCheck = true, bool executeMotion = true);

        /*!
         * \brief Moves the robot to the requested pose target.
         * \param poseTarget The pose target.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         * \param constrainedYaw Wether to constrain the end effector pose yaw or not.
         * \param visibilityConstraint Wether to enforce the visibilty constraint or not.
         */
        void goToTarget(geometry_msgs::Pose poseTarget, bool visualCheck = true, bool executeMotion = true, bool constrainedYaw = true, bool visibilityConstraint = false);

        /*!
         * \brief Checks wether a target pose is reachable by the robot arm.
         * \param poseTarget The pose target.
         * \param collision Wether to take collisions into account or not - default : false.
         * \return true if the target pose is reachable, false otherwise.
         */
        bool isReachable(geometry_msgs::Pose poseTarget, bool collisions = true);

        /*!
         * \brief Checks wether a target pose is reachable by the robot arm.
         * \param poseTarget The pose target.
         * \param jointsValues If the pose target is reachable, the corresponding joints configuration.
         * \param collision Wether to take collisions into account or not - default : false.
         * \return true if the target pose is reachable, false otherwise.
         */
        bool isReachable(geometry_msgs::Pose poseTarget, std::vector<double>& jointsValues, bool collisions = true);

        /*!
         * \brief Checks wether a target pose is reachable by the robot arm.
         * \param jointsTargetValues The joints target.
         * \param collision Wether to take collisions into account or not - default : false.
         * \return true if the target pose is reachable, false otherwise.
         */
        bool isReachable(std::vector<double> jointsTargetValues, bool collisions = true);

        /*!
         * \brief Computes the forward kinematic model of the robot arm.
         * \param desiredJointsValues The desired joints values.
         * \return The corresponding pose.
         */
        geometry_msgs::Pose forwardKinematics(std::vector<double> desiredJointsValues);

        /*!
         * \brief Computes the inverse kinematic model of the robot arm - VANILLA
         * \param desiredPose The desired pose.
         * \return The corresponding joints values.
         */
        std::vector<double> inverseKinematics(geometry_msgs::Pose desiredPose);

        /*!
         * \brief Returns the robot tool name.
         * \return The robot tool name.
         */
        std::string getToolName() const;

        /*!
         * \brief Returns the robot name.
         * \return The robot name.
         */
        std::string getName() const;

        /*!
         * \brief Returns the robot group name.
         * \return The robot group name.
         */
        std::string getGroupName() const;

        /*!
         * \brief Returns the robot end effector name.
         * \return The robot end effector name.
         */
        std::string getEndEffectorName() const;

        /*!
         * \brief Sets the robot end effector.
         * \param endEffectorName The robot end effector name.
         * \return true if the end effector has been set, false otherwise.
         */
        bool setEndEffector(std::string endEffectorName);

        /*!
         * \brief Check wether the robot is simulated or not
         * \return true if the robot is simulated, false otherwise
         */
        bool isSimulated() const;

        /*!
         * \brief Saves the robot end effector current state.
         * \param fileName Name of the file where the state is saved.
         */
        void saveCurrentState(std::string fileName);

        /*!
         * \brief Saves the robot end effector current position.
         * \param fileName Name of the file where the position is saved.
         */
        void saveCurrentPose(std::string fileName); 

        /*!
         * \brief Logs the robot end effector current position in a trajectory log file.
         * \param fileName Name of the file where the position is logged.
         * \param index Index of the position in the trajectory.
         */
        void logCurrentPose(std::string fileName, int index = 0);

        /*!
         * \brief Saves the robot current joints values.
         * \param fileName Name of the file where the joints values are saved.
         */
        void saveCurrentJointsValues(std::string fileName);

        /*!
         * \brief Performs a measurement routine along a given list of end effector poses.
         * \param waypoints The list of end effector poses.
         * \param initialMeasurement Wether or not to perform a measurement in the initial configuration of the robot.
         * \param saveStates Wether or not to save measurements states.
         * \param yawOffset Value of the eventual allowed yaw offset around the axis tool. Must be contained between 0 (included) and 2*M_PI (excluded), and defaults to 0. A negative value will be interpreted as any yaw offset allowed !
         * \param visibilityConstraint Wether to enforce the visibility constraint of not. Defaults to false.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         */
        void runMeasurementRoutine(std::vector<geometry_msgs::Pose> waypoints, bool initialMeasurement = false, bool saveStates = true, double yawOffset = 0, bool visibilityConstraint = false, bool visualCheck = false);

        /*!
         * \brief Performs a measurement routine along a given list of joints configurations.
         * \param waypoints The list of joints configurations.
         * \param initialMeasurement Wether or not to perform a measurement in the initial configuration of the robot.
         * \param saveStates Wether or not to save measurements states.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         */
        void runMeasurementRoutine(std::vector<std::vector<double>> waypoints, bool initialMeasurement = false, bool saveStates = true, bool visualCheck = false);

        /*!
         * \brief Performs a continuous trajectory along a given list of end effector poses.
         * \param waypoints The list of enf effector poses.
         * \param interpolationResolution Resolution of the trajectory interpolation.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         */
        void runTrajectory(std::vector<geometry_msgs::Pose> waypoints, double interpolationResolution = 0.01, bool visualCheck = true, bool executeMotion = true);   

        /*!
         * \brief Starts Robot action servers
         */
        void startActions();

    private:

        /*!
         * \brief Performs motion planning and execution to reach a target.
         * \param visualCheck Wether to perform a visual check in RViz before motion execution or not.
         * \param executeMotion Wether to actually move the robot or not.
         * \param constrainedYaw Wether to constrain the end effector pose yaw or not.
         * \param visibilityConstraint Wether to enforce the visibilty constraint or not.
         */
        void m_goToTarget(bool visualCheck = true, bool executeMotion = true, bool constrainedYaw = true, bool visibilityConstraint = false);

        void m_executePT(const robot_arm_tools::PoseTargetGoalConstPtr& goal);
        void m_executeJVT(const robot_arm_tools::JointsValuesTargetGoalConstPtr& goal);
        void m_executePW(const robot_arm_tools::PoseWaypointsGoalConstPtr& goal);
        void m_executeJVW(const robot_arm_tools::JointsValuesWaypointsGoalConstPtr& goal);
        
        /*!
         * \brief Pause subscriber callback method.
         * \param msg Pause message.
         */
        void pauseCallback(const std_msgs::Bool::ConstPtr& msg);

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroup;    /*!< Moveit! move group interface used to move the robot */    

        std::shared_ptr<const moveit::core::RobotModel> m_robotModel;  /*!< Moveit! robot model */
        std::shared_ptr<const moveit::core::JointModelGroup> m_jointModelGroup;    /*!< Moveit! joint model group */
        //Add RobotState ?
               
        std::string m_robotName; /*!< Name of the robot arm */
        std::string m_groupName;    /*!< Name of the group assocaited to the robot arm */
        std::string m_toolName;    /*!< Name of the tool mounted on the robot arm */
        std::string m_endEffectorName;    /*!< Name of the robot arm end effector */

        bool m_simulation;  /*!< Wether the robot is simuated or not */
        bool m_calibration;  /*!< Wether a calibrated model is provided or not */

        double m_velocityRatio; /*!< Current maximum velocity ratio */
        double m_accelerationRatio; /*!< Current maximum acceleration ratio */

        std::vector<std::vector<double>> m_jointsLimits;    /*!< Robot arm geometrical joints limits */
        int m_jointsNumber; /*!< Number of joints in the robot arm */

        std::map<std::string,double> m_initialNamedJointsValues;  /*!< joints configuration of the robot arm in its initial configuration */
        std::vector<double> m_initialJointsValues;  /*!< joints configuration of the robot arm in its initial configuration */
        geometry_msgs::Pose m_initialPose;    /*!< Pose of the robot arm end effector in the robot arm initial configuration */

        RobotVisualTools m_visualTools; /*!< Visual tools used for motion visual checks and poses display */
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_planningSceneMonitor;    /*!< Planning scene monitor used for collision checks */

        ros::NodeHandle m_nodeHandle;    /*!< ROS node handle*/
        
        ros::ServiceClient m_FKClient;   /*!< ROS service client used for forward kinematics computations */
        ros::ServiceClient m_IKClient;   /*!< ROS service client used for inverse kinematics computations */
        std::string m_kinematicsSolver; /*!< Name of the kinematics solver */

        actionlib::SimpleActionServer<robot_arm_tools::PoseTargetAction> m_poseTargetServer;    /*!< Custom ROS action server for pose target motion */
        actionlib::SimpleActionServer<robot_arm_tools::JointsValuesTargetAction> m_jointsValuesTargetServer;    /*!< Custom ROS action server for joints values target motion */
        actionlib::SimpleActionServer<robot_arm_tools::PoseWaypointsAction> m_poseWaypointsServer;  /*!< Custom ROS action server for pose waypoints motion */
        actionlib::SimpleActionServer<robot_arm_tools::JointsValuesWaypointsAction> m_jointsValuesWaypointsServer;  /*!< Custom ROS action server for joints values waypoints motion */

        boost::shared_ptr<RobotLowLevel> m_robotLowLevel;   /*!< Robot low level interface (plugin) */
        pluginlib::ClassLoader<RobotLowLevel> m_robotLowLevelLoader;     /*!< Robot low level plugin loader */

        ros::Subscriber m_pauseSubscriber;   /*!< ROS subscriber for pause topic */
        bool m_paused;   /*!< Robot paused state */
        actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> m_executionActionClient;  /*!< ROS action client for trajectory execution (no MoveGroup interface)*/
};