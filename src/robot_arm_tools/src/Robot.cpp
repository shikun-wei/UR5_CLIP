#include "robot_arm_tools/Robot.h"

// Moveit robot trajectory
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// Moveit messages
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>

// Joints limits interface
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

// Measurement service messages
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include "robot_arm_tools/Int.h"

// TF tools
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// File I/O
#include <fstream>
#include <iostream>
#include <experimental/filesystem>

// Controllers tools
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_manager.h>

// Robot low level plugin loader
#include <pluginlib/class_loader.h>

// Robot IK
#include <bio_ik/bio_ik.h>

// Nullspace exploration
#include "robot_arm_tools/NullSpaceExploration.h"

// Robot logging
#include "robot_arm_tools/RobotLog.h"

// Robot errors
#include "robot_arm_tools/RobotErrors.h"

// Custom bio_ik goals
#include "robot_arm_tools/CustomGoals.hpp"

//Other messages
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#define MAX_RECOVERY_ATTEMPTS 2
#define MAX_REPLAN_ATTEMPTS 2

// TODO Generally, check moveit services calls for errors codes !
// TODO remove unused private attributes (robot name, tool name, initial named joints values, etc) ?
// TODO Add getJointsNames method ?
Robot::Robot() : m_velocityRatio(1.0), m_accelerationRatio(1.0), m_paused(false),
                 m_robotLowLevelLoader("robot_arm_tools", "RobotLowLevel"),
                 m_executionActionClient(m_nodeHandle, move_group::EXECUTE_ACTION_NAME, false),
                 m_poseTargetServer(m_nodeHandle, "robot_pose_target_action", boost::bind(&Robot::m_executePT, this, _1), false),
                 m_jointsValuesTargetServer(m_nodeHandle, "robot_joints_values_target_action", boost::bind(&Robot::m_executeJVT, this, _1), false),
                 m_poseWaypointsServer(m_nodeHandle, "robot_pose_waypoints_action", boost::bind(&Robot::m_executePW, this, _1), false),
                 m_jointsValuesWaypointsServer(m_nodeHandle, "robot_joints_values_waypoints_action", boost::bind(&Robot::m_executeJVW, this, _1), false)
{
    if (!m_nodeHandle.getParam("robotName", m_robotName))
    {
        ROS_ERROR("Unable to retrieve robot name parameter !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if (!m_nodeHandle.getParam("groupName", m_groupName))
    {
        ROS_ERROR("Unable to retrieve robot group name parameter !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if (!m_nodeHandle.getParam("toolName", m_toolName))
    {
        ROS_ERROR("Unable to retrieve tool name parameter !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    m_toolName = m_toolName == "none" ? "" : m_toolName;

    if (!m_nodeHandle.getParam("endEffectorName", m_endEffectorName))
    {
        ROS_ERROR("Unable to retrieve end effector name parameter !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if (!m_nodeHandle.getParam("simulation", m_simulation))
    {
        ROS_ERROR("Unable to retrieve simulation status !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if (!m_nodeHandle.getParam("calibration", m_calibration))
    {
        ROS_ERROR("Unable to retrieve calibration status !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    m_nodeHandle.param<std::string>("robot_description_kinematics/" + m_groupName + "/kinematics_solver", m_kinematicsSolver, "none");
    m_kinematicsSolver = m_kinematicsSolver.substr(0, m_kinematicsSolver.find("/"));

    setupRobot();
}

Robot::~Robot()
{
    m_robotLowLevel->triggerShutdown();
}

void Robot::setupRobot()
{
    // Create a dedicated Moveit move group interface, robot model and joint group model
    m_moveGroup.reset(new moveit::planning_interface::MoveGroupInterface(m_groupName));
    m_robotModel = m_moveGroup->getRobotModel();
    m_jointModelGroup.reset(m_robotModel->getJointModelGroup(m_groupName));

    // Create a dedicated Moveit planning scene monitor, and start monitoring
    m_planningSceneMonitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if (!m_planningSceneMonitor->requestPlanningSceneState("/get_planning_scene"))
    {
        ROS_WARN("Could not get initial planning scene for further monitoring !");
    }
    m_planningSceneMonitor->startSceneMonitor("/move_group/monitored_planning_scene");

    // Set the proper end effector for position control
    bool endEffectorChanged = (m_moveGroup->getEndEffectorLink() != m_endEffectorName);
    bool result = m_moveGroup->setEndEffectorLink(m_endEffectorName);

    if (result && endEffectorChanged)
    {
        ROS_INFO("Successfully setting end effector to %s", m_moveGroup->getEndEffectorLink().c_str());
    }
    else if (!endEffectorChanged)
    {
        ROS_INFO("End effector already set to %s", m_moveGroup->getEndEffectorLink().c_str());
    }
    else
    {
        ROS_WARN("Failed to change end effector ! Default value (%s) is kept", m_moveGroup->getEndEffectorLink().c_str());
    }

    //Get joints number and limits
    //m_jointsNumber = m_moveGroup->getJointNames().size();
    m_jointsNumber = m_moveGroup->getVariableNames().size();

    moveit::core::JointBoundsVector limits = m_jointModelGroup->getActiveJointModelsBounds();
    for (std::size_t i = 0; i < m_jointsNumber; i++)
    {
        m_jointsLimits.push_back({(*limits[i])[0].min_position_, (*limits[i])[0].max_position_});
    }

    // Give as ROS param the mapping between URDF and joint_states joints ordering
    std::vector<std::string> statesNames = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states")->name;
    //std::vector<std::string> urdfNames = m_moveGroup->getJointNames();
    std::vector<std::string> urdfNames = m_moveGroup->getVariableNames();
    std::vector<int> mapping;
    std::vector<std::string>::iterator output;

    for (std::vector<std::string>::iterator it = urdfNames.begin(); it != urdfNames.end(); it++)
    {
        output = std::find(statesNames.begin(), statesNames.end(), *it);
        if (output == statesNames.end())
        {
            ROS_ERROR("JOINTS NAMES MAPPING ERROR !");
            throw(std::runtime_error("Error while mapping joint states and URDF joints names !"));
        }
        mapping.push_back(output - statesNames.begin());
    }
    m_nodeHandle.setParam("/jointsNamesMapping", mapping);

    // Create the tools for IK and FK methods
    m_FKClient = m_nodeHandle.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    m_FKClient.waitForExistence();

    m_IKClient = m_nodeHandle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    m_IKClient.waitForExistence();

    // Set default joints, position and orientation tolerances to the same value as IK //TODO TEST ON REAL HARDWARE
    m_moveGroup->setGoalJointTolerance(1e-6);
    m_moveGroup->setGoalPositionTolerance(1e-6);
    m_moveGroup->setGoalOrientationTolerance(1e-5);

    // Set general parameters for planning -> TODO adapt when the tool is a depth sensor ?
    bool robotPerception = false;

    if (m_nodeHandle.getParam("robotPerception", robotPerception) && robotPerception)
    {
        m_moveGroup->allowReplanning(true);
        m_moveGroup->setReplanAttempts(2);
        m_moveGroup->setReplanDelay(1.0);

        m_moveGroup->allowLooking(true);
        m_moveGroup->setLookAroundAttempts(2);
    }
    else
    {
        m_moveGroup->allowReplanning(false);
        m_moveGroup->allowLooking(false);
    }
    m_moveGroup->setNumPlanningAttempts(1);

    // Get the initial joint configuration
    std::string initialConfigurationName;
    if (!m_nodeHandle.getParam("initialConfigurationName", initialConfigurationName))
    {
        ROS_ERROR("Unable to retrieve robot initial configuration name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    m_initialNamedJointsValues = m_moveGroup->getNamedTargetValues(initialConfigurationName);
    for (std::map<std::string, double>::iterator it = m_initialNamedJointsValues.begin(); it != m_initialNamedJointsValues.end(); it++)
    {
        m_initialJointsValues.push_back(it->second);
    }

    // Get the initial end effector pose
    try
    {
        m_initialPose = forwardKinematics(m_initialJointsValues);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("FORWARD KINMATICS ERROR !");
        throw(std::out_of_range("Error while solving forward kinematics !"));
    }

    // Get and set the motion planner (if needed)
    std::string plannerName;
    if (m_nodeHandle.getParam("plannerName", plannerName))
    {
        m_moveGroup->setPlannerId(plannerName);

        // TODO custom objective ?
        std::string plannerObjective;
        if (m_nodeHandle.getParam("plannerObjective", plannerObjective))
        {
            std::map<std::string, std::string> newObjective{{"optimization_objective", "plannerObjective"}};
            m_moveGroup->setPlannerParams(plannerName, m_groupName, newObjective, false);
        }
    }
    double planningTime = m_nodeHandle.param<double>("planningTime",5.0);
    m_moveGroup->setPlanningTime(planningTime);

    if(!m_simulation)
    {
        // Set low acceleration and velocity for security purposes
        setAcceleration(0.05);
        setVelocity(0.1);
    }
    else
    {
        //There are no risks in simulation ;)
        setAcceleration(1.0);
        setVelocity(1.0);
    }

    // Wait for controller to start properly for real life execution mode
    if(!m_simulation && m_robotName != "kuka")
    {
        ros::ServiceClient listControllers = m_nodeHandle.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
        controller_manager_msgs::ListControllers listControllersIO;
        bool controllerStarted = false;
        double startTime = ros::WallTime::now().toSec();

        while (!controllerStarted && ros::WallTime::now().toSec() - startTime < 10.0)
        {
            if (listControllers.call(listControllersIO))
            {
                for (int i = 0; i < listControllersIO.response.controller.size(); i++)
                {
                    std::string type = (listControllersIO.response.controller[i].type).substr(0, (listControllersIO.response.controller[i].type).find("/"));
                    if (type == "position_controllers" && listControllersIO.response.controller[i].state == "running")
                    {
                        controllerStarted = true;
                        ROS_INFO("Controller %s started properly !", (listControllersIO.response.controller[i].name).c_str());
                    }
                }
            }
        }
        if (!controllerStarted)
        {
            ROS_ERROR("Unable to properly start controller !");
            throw std::runtime_error("UNSTARTABLE CONTROLLER");
        }
    }

    // Load the robot low level plugin for startup, shutdown and recovery purposes
    // TODO More generic -> Parameter ?
    if (!m_simulation)
    {
        try
        {
            if (m_robotName == "panda")
            {
                m_robotLowLevel = m_robotLowLevelLoader.createInstance("PandaLowLevel");
            }
            else if (m_robotName.find(std::string("ur")) != std::string::npos)
            {
                m_robotLowLevel = m_robotLowLevelLoader.createInstance("URLowLevel");
            }
            else
            {
                throw pluginlib::PluginlibException("UNKNOWN ROBOT");
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            ROS_WARN("Could not load robot low level plugin, switching to default (empty) plugin");
            m_robotLowLevel = boost::shared_ptr<RobotLowLevel>(new RobotLowLevel);
        }
    }
    else
    {
        m_robotLowLevel = boost::shared_ptr<RobotLowLevel>(new RobotLowLevel);
    }

    // Handle pausing system
    m_pauseSubscriber = m_nodeHandle.subscribe("pause", 1, &Robot::pauseCallback, this);

    // Waiting for motion execution server (mandatory !)
    m_executionActionClient.waitForServer();

    m_robotLowLevel->triggerStartup();

    // Wait for planning scene to be fully set (i.e. wait for pending environement setting nodes to finish)
    // TODO : Find the environment nodes with their types rather than their names : https://answers.ros.org/question/363029/how-to-xmlrpc-client-of-another-node-in-c/
    bool planningScenceSet = false;
    std::vector<std::string> startupNodes, pendingNodes;

    while(!planningScenceSet)
    {
        //Get all running nodes
        ros::master::getNodes(startupNodes);
        for (std::vector<std::string>::iterator it = startupNodes.begin(); it != startupNodes.end(); it++)
        {
            if(it->find("robot_arm_tools_environment_node") != std::string::npos)
            {
                pendingNodes.push_back(*it);
            }
        }

        //list penging nodes
        if(pendingNodes.size() != 0)
        {
            ROS_INFO("Waiting for planning scene to be set, the following nodes are still running :");
            for (std::vector<std::string>::iterator it = pendingNodes.begin(); it != pendingNodes.end(); it++)
            {
                ROS_INFO(it->c_str());
            }
            ros::WallDuration(1.0).sleep();
            pendingNodes.clear();
            startupNodes.clear();
        }
        else
        {
            planningScenceSet = true;
        }
    }

    ROS_INFO("ROBOT SETUP OK");
}

void Robot::switchController(std::vector<std::string> startController, std::vector<std::string> stopController)
{
    ros::service::waitForService("controller_manager/switch_controller", ros::Duration(5.0));
    ros::ServiceClient switchController = m_nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switchControllerIO;
    switchControllerIO.request.start_controllers = startController;
    switchControllerIO.request.stop_controllers = stopController;
    switchControllerIO.request.strictness = 0;
    switchControllerIO.request.start_asap = false;
    switchControllerIO.request.timeout = 0.0;

    if (!(switchController.call(switchControllerIO)) || !(switchControllerIO.response.ok))
    {
        ROS_ERROR("Unable to switch controllers !");
        throw std::runtime_error("CANNOT SWITCH CONTROLLERS");
    }
    else
    {
        ROS_INFO("Controllers switched correctly");
    }
}

std::shared_ptr<planning_scene::PlanningScene> Robot::getPlanningScene() const
{
    //Safe planning scene getter
    planning_scene_monitor::LockedPlanningSceneRO safePlanningScene(m_planningSceneMonitor);
    return(safePlanningScene->clone(safePlanningScene));
}

std::shared_ptr<const moveit::core::RobotModel> Robot::getRobotModel() const
{
    return (m_robotModel);
}

int Robot::getJointsNumber() const
{
    return (m_jointsNumber);
}

std::vector<std::vector<double>> Robot::getJointsLimits() const
{
    return (m_jointsLimits);
}

std::map<std::string, double> Robot::getInitialNamedJointsValues() const
{
    return (m_initialNamedJointsValues);
}

std::vector<double> Robot::getInitialJointsValues() const
{
    return (m_initialJointsValues);
}

geometry_msgs::Pose Robot::getInitialPose() const
{
    return (m_initialPose);
}

void Robot::init(bool visualCheck, bool executeMotion)
{
    // Motion planning and execution
    goToTarget(m_initialJointsValues, visualCheck, executeMotion);
}

void Robot::setVelocity(double velocityRatio)
{
    m_moveGroup->setMaxVelocityScalingFactor(velocityRatio);
    m_velocityRatio = velocityRatio;
}

void Robot::setAcceleration(double accelerationRatio)
{
    m_moveGroup->setMaxAccelerationScalingFactor(accelerationRatio);
    m_accelerationRatio = accelerationRatio;
}

geometry_msgs::Pose Robot::getCurrentPose()
{
    // Get the current end effector pose
    try
    {
        return (forwardKinematics(getCurrentJointsValues()));
    }
    catch (const std::exception &e)
    {
        // Something went wrong with FK, return current move group state (should be equivalent when using a calibrated plugin !)
        return (m_moveGroup->getCurrentPose().pose);
    }
}

std::vector<double> Robot::getCurrentJointsValues() const
{
    return (m_moveGroup->getCurrentJointValues());
}

void Robot::m_goToTarget(bool visualCheck, bool executeMotion, bool constrainedYaw, bool visibilityConstraint)
{
    // Construct planning goal
    moveit_msgs::MoveGroupGoal planningGoal;
    m_moveGroup->constructMotionPlanRequest(planningGoal.request);
    planningGoal.planning_options.plan_only = true;
    planningGoal.planning_options.planning_scene_diff.is_diff = true;
    planningGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // Construct bio_ik goals
    bio_ik::BioIKKinematicsQueryOptions ikOptions;
    if(m_kinematicsSolver == "bio_ik")
    {
        ikOptions.return_approximate_solution = false;
        ikOptions.replace = true; // If false, default ikOptions will overrride custom ikOptions !

        // Construct collision avoidance goal
        CollisionAvoidanceGoal *collisionAvoidanceGoal = new CollisionAvoidanceGoal(*this, m_endEffectorName);
        ikOptions.goals.emplace_back(collisionAvoidanceGoal);
    }

    // DEBUG
    ROS_INFO("Raw Moveit Goal Constraints :");
    ROS_INFO("JOINT %ld", planningGoal.request.goal_constraints.back().joint_constraints.size());
    ROS_INFO("POSITION %ld", planningGoal.request.goal_constraints.back().position_constraints.size());
    ROS_INFO("ORIENTATION %ld", planningGoal.request.goal_constraints.back().orientation_constraints.size());
    ROS_INFO("VISIBILITY %ld", planningGoal.request.goal_constraints.back().visibility_constraints.size());

    // Case where a pose target is provided (assumption : joint targets are already fully defined)
    bool isPoseTarget = (planningGoal.request.goal_constraints.back().position_constraints.size() != 0);

    ros::WallTime startTime = ros::WallTime::now();
    
    bool failure = false;
    if (isPoseTarget)
    {
        ROS_INFO("IK solver : %s", m_kinematicsSolver.c_str());

        //Get target pose
        geometry_msgs::Pose targetPose = m_moveGroup->getPoseTarget().pose;
        ROS_INFO("Target pose : %f %f %f", targetPose.position.x, targetPose.position.y, targetPose.position.z);

        //Yaw constraints
        if (!constrainedYaw)
        {
            ROS_INFO("Unconstrained yaw");
            // Defining constraints such that tool yaw orientation constraint has a large tolerance of +/- 2*M_PI
            planningGoal.request.goal_constraints.back().orientation_constraints.back().absolute_z_axis_tolerance =  M_PI / 6; // It already exists !

            if(m_kinematicsSolver == "bio_ik")
            {
                // Goal = Position + partial orientation
                bio_ik::PositionGoal *endEffectorPositionGoal = new bio_ik::PositionGoal();
                endEffectorPositionGoal->setLinkName(m_endEffectorName);

                tf2::Vector3 position;
                tf2::convert(targetPose.position, position);
                endEffectorPositionGoal->setPosition(position);
                ikOptions.goals.emplace_back(endEffectorPositionGoal);

                bio_ik::DirectionGoal *endEffectorDirectionGoal = new bio_ik::DirectionGoal();
                endEffectorDirectionGoal->setLinkName(m_endEffectorName);

                tf2::Vector3 axis;
                axis = tf2::Vector3(0, 0, 1); // Yaw
                endEffectorDirectionGoal->setAxis(axis);

                tf2::Quaternion orientation;
                tf2::convert(targetPose.orientation, orientation);
                // tf2::Matrix3x3 orientationMatrix = tf2::Matrix3x3(orientation);
                // tf2::Vector3 direction = orientationMatrix.getColumn(2);    //Normal
                tf2::Vector3 direction;
                bio_ik::quat_mul_vec(orientation, tf2::Vector3(0, 0, 1), direction); // Normal
                endEffectorDirectionGoal->setDirection(direction);
                ikOptions.goals.emplace_back(endEffectorDirectionGoal);
            }
        }
        else
        {
            ROS_INFO("Constrained yaw");
            if(m_kinematicsSolver == "bio_ik")
            {
                // Goal = Pose
                bio_ik::PoseGoal *endEffectorGoal = new bio_ik::PoseGoal();
                endEffectorGoal->setLinkName(m_endEffectorName);

                tf2::Vector3 position;
                tf2::convert(targetPose.position, position);
                endEffectorGoal->setPosition(position);
                tf2::Quaternion orientation;
                tf2::convert(targetPose.orientation, orientation);
                endEffectorGoal->setOrientation(orientation);
                ikOptions.goals.emplace_back(endEffectorGoal);
            }
        }

        // Save initial goal and state
        moveit::core::RobotState robotState(m_robotModel);
        robotState.setJointGroupPositions(m_jointModelGroup.get(),getCurrentJointsValues());
        robotState.update();

        moveit_msgs::MoveGroupGoal initialGoal = planningGoal;

        // Visibility constraints
        if(visibilityConstraint)
        {
            ROS_INFO("Visibility constraint");

            // Default visibility axis
            Eigen::Vector3d visibilityAxis(0.0, 0.0, 1.0);
    
            // Retrieve visibility radius
            double visibilityRadius = m_nodeHandle.param<double>("visibilityRadius", 0.1);

            // Retrieve visibility distance, TF name or visibilityPose
            double visibilityDistance = 0.5;
            std::string visibilityTFName;
            std::vector<double> visibilityPoseArray;
            geometry_msgs::PoseStamped visibilityPose;

            // Distance case
            if (m_nodeHandle.param<double>("visibilityDistance", visibilityDistance, 0.5) || !m_nodeHandle.hasParam("visibilityDistance") && !m_nodeHandle.hasParam("visibilityTFName") && !m_nodeHandle.hasParam("visibilityPose") || m_nodeHandle.getParam("visibilityDistance", visibilityDistance))
            {
                tf2::Quaternion tmpQuaternion;
                tf2::fromMsg(targetPose.orientation, tmpQuaternion);
                tf2::Matrix3x3 tmpMatrix(tmpQuaternion);
                tf2::Vector3 Z = tmpMatrix.getColumn(2);
                visibilityPose.pose = targetPose;
                visibilityPose.pose.position.x += visibilityDistance * Z[0];
                visibilityPose.pose.position.y += visibilityDistance * Z[1];
                visibilityPose.pose.position.z += visibilityDistance * Z[2];
            }

            // No distance case
            else
            {
                // TF case
                if (m_nodeHandle.getParam("visibilityTFName", visibilityTFName))
                {
                    tf2_ros::Buffer tfBuffer;
                    tf2_ros::TransformListener tfListener(tfBuffer);
                    geometry_msgs::Transform tmpTransform;
                    try
                    {
                        tmpTransform = tfBuffer.lookupTransform(m_moveGroup->getPoseReferenceFrame(), visibilityTFName, ros::Time(0), ros::Duration(5.0)).transform;
                    }
                    catch (tf2::TransformException &ex)
                    {
                        throw std::runtime_error("CANNOT RETRIVE VISIBILITY TRANSFORM !");
                    }

                    visibilityPose.pose.position.x = tmpTransform.translation.x;
                    visibilityPose.pose.position.y = tmpTransform.translation.y;
                    visibilityPose.pose.position.z = tmpTransform.translation.z;
                    visibilityPose.pose.orientation = tmpTransform.rotation;
                }

                // Pose case
                else if (m_nodeHandle.getParam("visibilityPose", visibilityPoseArray))
                {
                    visibilityPose.pose.position.x = visibilityPoseArray[0];
                    visibilityPose.pose.position.y = visibilityPoseArray[1];
                    visibilityPose.pose.position.z = visibilityPoseArray[2];

                    tf2::Quaternion tmpQuaternion;
                    tmpQuaternion.setRPY(visibilityPoseArray[3], visibilityPoseArray[4], visibilityPoseArray[5]);
                    visibilityPose.pose.orientation = tf2::toMsg(tmpQuaternion);
                }

                visibilityDistance = sqrt(
                    (visibilityPose.pose.position.x - targetPose.position.x) * (visibilityPose.pose.position.x - targetPose.position.x) + (visibilityPose.pose.position.y - targetPose.position.y) * (visibilityPose.pose.position.y - targetPose.position.y) + (visibilityPose.pose.position.z - targetPose.position.z) * (visibilityPose.pose.position.z - targetPose.position.z));
                m_nodeHandle.setParam("visibilityDistance", visibilityDistance);
            }
        
            if (m_kinematicsSolver != "bio_ik")
            {
                // Initial dummy planning to get a reachable configuration
                planningGoal.request.planner_id = "RRTConnect";
                m_moveGroup->getMoveGroupClient().sendGoal(planningGoal);
                while (!m_moveGroup->getMoveGroupClient().getState().isDone())
                {
                    ros::WallDuration(0.1).sleep();
                    continue;
                }
                planningGoal.request.planner_id = m_moveGroup->getPlannerId();

                // Case where planning was unsuccessfull: failure
                if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    failure = true;
                }
            
                // Case where planning was successfull: explore nullspace around target (end of trajectory) configuration
                else
                {
                    Eigen::VectorXd initialConfiguration = Eigen::VectorXd::Map(&m_moveGroup->getMoveGroupClient().getResult()->planned_trajectory.joint_trajectory.points.back().positions[0],m_jointsNumber);

                    VisibilityNullSpaceExploration ns(*this,initialConfiguration,constrainedYaw,visibilityAxis,visibilityDistance,visibilityRadius);
                    Eigen::VectorXd bestConfiguration = ns.findBestLocalConfiguration();

                    // Case where no valid configuration was found : fallback on the initial planning goal (with better planner than RRTConnect)
                    if(initialConfiguration == bestConfiguration)
                    {
                        failure = true;
                    }
                    // Case where a valid configuration was found : set it as the target
                    else
                    {
                        robotState.setJointGroupPositions(m_jointModelGroup.get(),bestConfiguration);
                        m_moveGroup->setJointValueTarget(robotState);
                    }
                }

                /* Moveit visibility constraints
                // Define sensor pose as the target pose
                geometry_msgs::PoseStamped sensorPose;
                sensorPose.header.stamp = ros::Time::now();
                sensorPose.pose = targetPose; // TODO Custom frame defined according to target pose ? Bio_ik like implementation ?

                // Add offset to avoid self collisions
                tf2::Quaternion tmpQuaternion;
                tf2::fromMsg(targetPose.orientation, tmpQuaternion);
                tf2::Matrix3x3 tmpMatrix(tmpQuaternion);
                tf2::Vector3 Z = tmpMatrix.getColumn(2);
                sensorPose.pose.position.x += 0.05 * Z[0];
                sensorPose.pose.position.y += 0.05 * Z[1];
                sensorPose.pose.position.z += 0.05 * Z[2];

                // TODO Angles parameters and target orientation may be used to maximize normal camera measurements
                planningGoal.request.goal_constraints.back().visibility_constraints.emplace_back();
                planningGoal.request.goal_constraints.back().visibility_constraints.back().target_radius = visibilityRadius;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().target_pose = visibilityPose;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().cone_sides = 5;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().sensor_pose = sensorPose;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().sensor_view_direction = 0;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().max_view_angle = 0;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().max_range_angle = 0;
                planningGoal.request.goal_constraints.back().visibility_constraints.back().weight = 0.5;
                */
            }

            else
            {
                // Secondary goal => Visibility
                tf2::Vector3 visibilityAxisTF(visibilityAxis[0], visibilityAxis[1], visibilityAxis[2]);
                VisibilityGoal *visibilityGoal = new VisibilityGoal(*this, m_endEffectorName, visibilityAxisTF, visibilityDistance, visibilityRadius, true);
                ikOptions.goals.emplace_back(visibilityGoal);

                bool output = robotState.setFromIK(m_jointModelGroup.get(), EigenSTL::vector_Isometry3d(), std::vector<std::string>(), 0.0, moveit::core::GroupStateValidityCallbackFn(), ikOptions);

                if (output)
                {
                    m_moveGroup->setJointValueTarget(robotState);
                    failure = false;

                    /*//DEBUG
                    Eigen::Isometry3d pose;

                    //Translation
                    pose = Eigen::Translation3d(0.5*visibilityDistance*visibilityAxis);

                    //Rotation
                    Eigen::Vector3d orthogonalAxis = visibilityAxis.cross(Eigen::Vector3d::UnitX());
                    if(orthogonalAxis.norm() < 0.0001)
                    {
                        orthogonalAxis = visibilityAxis.cross(Eigen::Vector3d::UnitY());
                    }
                    pose.rotate(Eigen::AngleAxis<double>(M_PI,orthogonalAxis));

                    shapes::ShapeConstPtr visibilityCone = shapes::ShapeConstPtr(new shapes::Cone(visibilityRadius,visibilityDistance));
                    moveit::core::AttachedBody visibilityConeAttached(m_jointGroupModel->getLinkModel(m_endEffectorName),"visibility_cone",pose,{visibilityCone},EigenSTL::vector_Isometry3d(1, Eigen::Isometry3d::Identity()),std::set<std::string>(),trajectory_msgs::JointTrajectory());

                    //std::vector<moveit_msgs::AttachedCollisionObject> attachedCollisionObjects;
                    //moveit::core::attachedBodiesToAttachedCollisionObjectMsgs({&visibilityConeAttached},attachedCollisionObjects);

                    //moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
                    //attachedCollisionObjects[0].touch_links = m_jointGroupModel->getLinkModelNames();    //Collisions goes brrrr
                    //planningSceneInterface.applyAttachedCollisionObject(attachedCollisionObjects[0]);
                    */
                }
                else
                {
                    //Remove visibility goal
                    ikOptions.goals.pop_back();
                    failure = true;
                }
            }   

            if(!failure)
            {          
                ROS_INFO("Constrained IK suceeded");
                m_moveGroup->constructMotionPlanRequest(planningGoal.request);
            }
            else
            {
                ROS_INFO("Constrained IK failed");
            }
        }
        else
        {
            ROS_INFO("No visibility constraint");
        }

        if (!visibilityConstraint || failure)
        {
            if(m_kinematicsSolver == "bio_ik")
            {
                ROS_INFO("BIO IK failed, trying with less constraints...");
                // Secondary goal => Avoid joints limits, minimize displacement
                // bio_ik::AvoidJointLimitsGoal *avoidJointLimitsGoal = new bio_ik::AvoidJointLimitsGoal();
                // ikOptions.goals.emplace_back(avoidJointLimitsGoal);

                bio_ik::MinimalDisplacementGoal *minimalDisplacementGoal = new bio_ik::MinimalDisplacementGoal();
                ikOptions.goals.emplace_back(minimalDisplacementGoal);

                bool output = robotState.setFromIK(m_jointModelGroup.get(), EigenSTL::vector_Isometry3d(), std::vector<std::string>(), 0.0, moveit::core::GroupStateValidityCallbackFn(), ikOptions);

                if (output)
                {
                    m_moveGroup->setJointValueTarget(robotState);
                    m_moveGroup->constructMotionPlanRequest(planningGoal.request);
                }
                else
                {
                    ROS_WARN("BIO IK failed, defaulting to Moveit planner");
                    planningGoal = initialGoal;
                }
            }
            else
            {
                ROS_WARN("Defaulting to Moveit planner");
                planningGoal = initialGoal;
            }
        }   
    }
    
    double duration = (ros::WallTime::now() - startTime).toSec();

    int replanAttempt = 0;
    std::vector<double> intialJointsValues = getCurrentJointsValues();   //Save intial (safe) state for recovery
    while(replanAttempt < MAX_REPLAN_ATTEMPTS)
    {
        // TODO (?) Add minimal delay between two recoveries to detect an impossible recovery situation
        int recoveryAttempt = 0;
        while (recoveryAttempt < MAX_RECOVERY_ATTEMPTS)
        {
            // goal.request.allowed_planning_time = 5.0;   //TODO Set as parameter ?
            // m_moveGroup->setPlanningTime(5.0);
            ROS_INFO("Planning Moveit Goal Constraints :");
            ROS_INFO("JOINT %ld", planningGoal.request.goal_constraints.back().joint_constraints.size());
            ROS_INFO("POSITION %ld", planningGoal.request.goal_constraints.back().position_constraints.size());
            ROS_INFO("ORIENTATION %ld", planningGoal.request.goal_constraints.back().orientation_constraints.size());
            ROS_INFO("VISIBILITY %ld", planningGoal.request.goal_constraints.back().visibility_constraints.size());

            // Send goal for planning
            m_moveGroup->getMoveGroupClient().sendGoal(planningGoal);
            while (!m_moveGroup->getMoveGroupClient().getState().isDone())
            {
                ros::WallDuration(0.1).sleep();
                continue;
            }

            if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED && m_moveGroup->getPlannerId() != "RRTConnect")
            {
                //Retry with RRTConnect !
                ROS_WARN("Planning failed with planner %s, trying with RRTConnect instead...", m_moveGroup->getPlannerId().c_str());
                planningGoal.request.planner_id = "RRTConnect";
                m_moveGroup->getMoveGroupClient().sendGoal(planningGoal);
                while (!m_moveGroup->getMoveGroupClient().getState().isDone())
                {
                    ros::WallDuration(0.1).sleep();
                    continue;
                }
                planningGoal.request.planner_id = m_moveGroup->getPlannerId();
            }

            bool benchmark = false;
            if (benchmark)
            {
                // BENCHMARK
                std::string path;
                if (constrainedYaw && visibilityConstraint)
                {
                    path = "/tmp/" + m_kinematicsSolver + "_constrained_yaw_visibility_fitness.txt";
                }
                else if (constrainedYaw && !visibilityConstraint)
                {
                    path = "/tmp/" + m_kinematicsSolver + "_constrained_yaw_fitness.txt";
                }
                else if (!constrainedYaw && visibilityConstraint)
                {
                    path = "/tmp/" + m_kinematicsSolver + "_unconstrained_yaw_visibility_fitness.txt";
                }
                else
                {
                    path = "/tmp/" + m_kinematicsSolver + "_unconstrained_yaw_fitness.txt";
                }
                std::ofstream file(path, std::ios::app);

                // Case where planning was unsuccessfull
                if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    file << "-1";
                    file << ";";
                    file << duration;
                    file << ";";
                    file << (int)failure;
                    file << "\n";
                }
                else
                {
                    // Visibility parameters
                    Eigen::Vector3d visibilityAxis(0.0, 0.0, 1.0);
                    double visibilityRadius = m_nodeHandle.param<double>("visibilityRadius", 0.1);
                    double visibilityDistance = m_nodeHandle.param<double>("visibilityDistance", 0.5);

                    tf2::Vector3 visibilityAxisTF(visibilityAxis[0], visibilityAxis[1], visibilityAxis[2]);
                    VisibilityGoal visibilityGoal(*this, m_endEffectorName, visibilityAxisTF, visibilityDistance, visibilityRadius, true);

                    double visibilityFitness = visibilityGoal.evaluate(&m_moveGroup->getMoveGroupClient().getResult()->planned_trajectory.joint_trajectory.points.back().positions[0], true);
                    
                    ROS_INFO("Final visibility fitness : %f", visibilityFitness);
                    ROS_INFO("IK computation duration : %f", duration);

                    file << visibilityFitness;
                    file << ";";
                    file << duration;
                    file << ";";
                    file << (int)failure;
                    file << "\n";
                }
            }

            if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_WARN("Fail: %s : %s", m_moveGroup->getMoveGroupClient().getState().toString().c_str(), m_moveGroup->getMoveGroupClient().getState().getText().c_str());
                ROS_ERROR("Robot planning failed with error type : %i", (int)m_moveGroup->getMoveGroupClient().getResult()->error_code.val);
                throw std::out_of_range("ROBOT PLANNING FAILED");
            }

            // Set RViz trigger for visual check
            if (visualCheck)
            {
                m_visualTools.trigger();
                m_visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue !");
            }

            // Execute motion
            if (executeMotion)
            {
                moveit_msgs::ExecuteTrajectoryGoal executionGoal;
                executionGoal.trajectory = m_moveGroup->getMoveGroupClient().getResult()->planned_trajectory;

                while (m_paused)
                {
                    // Loop until motion is unpaused...
                    ros::WallDuration(0.1).sleep();
                    continue;
                }

                m_executionActionClient.sendGoal(executionGoal);

                while (!m_executionActionClient.getState().isDone())
                {
                    // Loop until motion is completed, interrupted or paused
                    if (m_paused)
                    {
                        // Stop motion and escape loop
                        m_executionActionClient.cancelAllGoals();
                        break;
                    }
                }

                // Case where motion was paused
                if (m_paused)
                {
                    // If a motion is interrupted during execution, there is a high probability that the current robot position will differ from the initially planned path. To avoid any related error, we go back to the planning step !
                    continue;
                }

                // Case where motion was not successfully completed
                else if (m_executionActionClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("%s : %s", m_executionActionClient.getState().toString().c_str(), m_executionActionClient.getState().getText().c_str());
                    ROS_WARN("Robot execution failed with error type : %i - Trying to recover", (int)m_executionActionClient.getResult()->error_code.val);

                    // Recovery
                    m_robotLowLevel->recovery();
                    recoveryAttempt++;

                    // If a motion is interrupted during execution, there is a high probability that the current robot position will differ from the initially planned path. To avoid any related error, we go back to the planning step !
                    continue;
                }

                // Case where motion was successfully completed
                else
                {
                    // Escape the loop
                    break;
                }
            }

            else
            {
                // Escape the loop
                break;
            }
        }

        if(recoveryAttempt < MAX_RECOVERY_ATTEMPTS)
        {
            // Escape the loop
            break;
        }

        else
        {
            // Rewind and replan
            ROS_WARN("Recovery failed, rewinding and replanning...");

            // Rewind
            moveit_msgs::MoveGroupGoal rewindGoal;
            m_moveGroup->setJointValueTarget(intialJointsValues);
            m_moveGroup->constructMotionPlanRequest(rewindGoal.request);
            rewindGoal.planning_options.plan_only = true;
            rewindGoal.planning_options.planning_scene_diff.is_diff = true;
            rewindGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;

            int rewindRecoveryAttempt = 0;
            while(rewindRecoveryAttempt < MAX_RECOVERY_ATTEMPTS)
            {
                // Send goal for planning
                m_moveGroup->getMoveGroupClient().sendGoal(rewindGoal);
                while (!m_moveGroup->getMoveGroupClient().getState().isDone())
                {
                    ros::WallDuration(0.1).sleep();
                    continue;
                }

                if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED && m_moveGroup->getPlannerId() != "RRTConnect")
                {
                    //Retry with RRTConnect !
                    ROS_WARN("Planning failed with planner %s, trying with RRTConnect instead...", m_moveGroup->getPlannerId().c_str());
                    rewindGoal.request.planner_id = "RRTConnect";
                    m_moveGroup->getMoveGroupClient().sendGoal(rewindGoal);
                    while (!m_moveGroup->getMoveGroupClient().getState().isDone())
                    {
                        ros::WallDuration(0.1).sleep();
                        continue;
                    }
                    rewindGoal.request.planner_id = m_moveGroup->getPlannerId();
                }

                if (m_moveGroup->getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_WARN("Fail: %s : %s", m_moveGroup->getMoveGroupClient().getState().toString().c_str(), m_moveGroup->getMoveGroupClient().getState().getText().c_str());
                    ROS_ERROR("Robot planning failed with error type : %i", (int)m_moveGroup->getMoveGroupClient().getResult()->error_code.val);
                    throw std::out_of_range("ROBOT PLANNING FAILED");
                }

                // Set RViz trigger for visual check
                if (visualCheck)
                {
                    m_visualTools.trigger();
                    m_visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue !");
                }

                moveit_msgs::ExecuteTrajectoryGoal executionGoal;
                executionGoal.trajectory = m_moveGroup->getMoveGroupClient().getResult()->planned_trajectory;

                while (m_paused)
                {
                    // Loop until motion is unpaused...
                    ros::WallDuration(0.1).sleep();
                    continue;
                }

                m_executionActionClient.sendGoal(executionGoal);

                while (!m_executionActionClient.getState().isDone())
                {
                    // Loop until motion is completed, interrupted or paused
                    if (m_paused)
                    {
                        // Stop motion and escape loop
                        m_executionActionClient.cancelAllGoals();
                        break;
                    }
                }

                // Case where motion was paused
                if (m_paused)
                {
                    // If a motion is interrupted during execution, there is a high probability that the current robot position will differ from the initially planned path. To avoid any related error, we go back to the planning step !
                    continue;
                }

                // Case where motion was not successfully completed
                else if (m_executionActionClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("%s : %s", m_executionActionClient.getState().toString().c_str(), m_executionActionClient.getState().getText().c_str());
                    ROS_WARN("Robot execution failed with error type : %i - Trying to recover", (int)m_executionActionClient.getResult()->error_code.val);

                    // Recovery
                    m_robotLowLevel->recovery();
                    rewindRecoveryAttempt++;

                    // If a motion is interrupted during execution, there is a high probability that the current robot position will differ from the initially planned path. To avoid any related error, we go back to the planning step !
                    continue;
                }

                // Case where motion was successfully completed
                else
                {
                    // Escape the loop
                    break;
                }
            }

            if(rewindRecoveryAttempt >= MAX_RECOVERY_ATTEMPTS)
            {
                // Rewind failed, abort
                ROS_ERROR("Rewind failed, aborting...");
                throw std::out_of_range("RECOVERY FAILED");
            }

            // Replan
            replanAttempt++;
            continue;
        }
    }

    if(replanAttempt >= MAX_REPLAN_ATTEMPTS)
    {
        // Replan failed, abort
        ROS_ERROR("Replan failed, aborting...");
        throw std::out_of_range("RECOVERY FAILED");
    }
}

void Robot::goToTarget(std::map<std::string, double> namedJointsTargetValues, bool visualCheck, bool executeMotion)
{
    // Send joint target and plan motion
    m_moveGroup->setJointValueTarget(namedJointsTargetValues);

    // Motion planning and execution
    m_goToTarget(visualCheck, executeMotion);
}

void Robot::goToTarget(std::vector<double> jointsTargetValues, bool visualCheck, bool executeMotion)
{
    // Send joint target and plan motion
    m_moveGroup->setJointValueTarget(jointsTargetValues);

    // Motion planning and execution
    m_goToTarget(visualCheck, executeMotion);
}

void Robot::goToTarget(geometry_msgs::Pose poseTarget, bool visualCheck, bool executeMotion, bool constrainedYaw, bool visibilityConstraint)
{
    // Send pose target and plan motion
    m_moveGroup->setPoseTarget(poseTarget, m_endEffectorName);

    // Motion planning and execution
    m_goToTarget(visualCheck, executeMotion, constrainedYaw, visibilityConstraint);
}

bool Robot::isReachable(geometry_msgs::Pose targetPose, bool collisions)
{
    // Remark : no collisions in the final state does not imply reachability !

    if (!collisions)
    {
        try
        {
            inverseKinematics(targetPose);
            return (true);
        }
        catch (const std::invalid_argument &e)
        {
            return (false);
        }
    }
    else
    {
        try
        {
            goToTarget(targetPose, false, false);
            return (true);
        }
        catch (const std::exception &e)
        {
            return (false);
        }
    }
}

bool Robot::isReachable(geometry_msgs::Pose targetPose, std::vector<double> &jointsValues, bool collisions)
{
    // Remark : no collisions in the final state does not imply reachability !

    if (!collisions)
    {
        try
        {
            jointsValues = inverseKinematics(targetPose);
            return (true);
        }
        catch (const std::invalid_argument &e)
        {
            return (false);
        }
    }
    else
    {
        // Send joint target and plan motion
        m_moveGroup->setPoseTarget(targetPose, m_endEffectorName);

        // Plan motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode error = m_moveGroup->plan(plan);

        // Planning error handling
        if (error != moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("Robot planning failed with error type : %i", error.val);
            return (false);
        }

        else
        {
            jointsValues = plan.trajectory_.joint_trajectory.points.back().positions;
            return (true);
        }
    }
}

bool Robot::isReachable(std::vector<double> jointsTargetValues, bool collisions)
{
    // Remark : no collisions in the final state does not imply reachability !

    if (!collisions)
    {
        try
        {
            forwardKinematics(jointsTargetValues);
            return (true);
        }
        catch (const std::invalid_argument &e)
        {
            return (false);
        }
    }
    else
    {
        try
        {
            goToTarget(jointsTargetValues, false, false);
            return (true);
        }
        catch (const std::exception &e)
        {
            return (false);
        }
    }
}

geometry_msgs::Pose Robot::forwardKinematics(std::vector<double> desiredJointsValues)
{
    moveit_msgs::GetPositionFK srvFK;
    srvFK.request.fk_link_names = {m_endEffectorName};
    
    //srvFK.request.robot_state.joint_state.name = m_moveGroup->getJointNames();
    srvFK.request.robot_state.joint_state.name = m_moveGroup->getVariableNames();
    srvFK.request.robot_state.joint_state.position = desiredJointsValues;

    if (!m_FKClient.call(srvFK))
    {
        throw(std::out_of_range("Error while solving forward kinematics !"));
    }
    else
    {
        return (srvFK.response.pose_stamped[0].pose);
    }
}

std::vector<double> Robot::inverseKinematics(geometry_msgs::Pose desiredPose)
{
    moveit_msgs::GetPositionIK srvIK;
    srvIK.request.ik_request.group_name = m_groupName;

    //srvIK.request.ik_request.robot_state.joint_state.name = m_moveGroup->getJointNames();
    srvIK.request.ik_request.robot_state.joint_state.name = m_moveGroup->getVariableNames();
    srvIK.request.ik_request.robot_state.joint_state.position = m_initialJointsValues;

    srvIK.request.ik_request.avoid_collisions = true;

    srvIK.request.ik_request.ik_link_name = m_endEffectorName;

    srvIK.request.ik_request.pose_stamped.pose = desiredPose;

    // srvIK.request.ik_request.attempts = 100;
    srvIK.request.ik_request.timeout = ros::Duration(1.0);

    if (!m_IKClient.call(srvIK))
    {
        throw(std::out_of_range("Error while solving inverse kinematics !"));
    }
    else
    {
        return (srvIK.response.solution.joint_state.position);
    }
}

std::string Robot::getToolName() const
{
    return (m_toolName);
}

std::string Robot::getName() const
{
    return (m_robotName);
}

std::string Robot::getGroupName() const
{
    return (m_groupName);
}

std::string Robot::getEndEffectorName() const
{
    return (m_endEffectorName);
}

bool Robot::setEndEffector(std::string endEffectorName)
{
    bool result = m_moveGroup->setEndEffectorLink(endEffectorName);
    if(result)
    {
        m_endEffectorName = endEffectorName;
        ROS_INFO("End effector link set to %s", endEffectorName.c_str());
    }
    else
    {
        ROS_WARN("Could not set end effector link to %s", endEffectorName.c_str());
    }
    return(result);
}

bool Robot::isSimulated() const
{
    return (m_simulation);
}

void Robot::saveCurrentState(std::string fileName)
{
    // Open or Create/open .csv file for positions recording
    std::ofstream file(fileName, std::ios::app);

    // Get current pose and compute roll, pitch and yaw angles
    geometry_msgs::Pose currentPose = getCurrentPose();

    tf2::Quaternion quaternion;
    tf2::fromMsg(currentPose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // Write the position in the file
    file << currentPose.position.x;
    file << ",";
    file << currentPose.position.y;
    file << ",";
    file << currentPose.position.z;
    file << ",";
    file << roll;
    file << ",";
    file << pitch;
    file << ",";
    file << yaw;
    file << ",---,";

    if(!m_simulation && m_robotName == "kuka")
    {
        std_msgs::Float64MultiArray::ConstPtr controllerPose;
        controllerPose = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/robot_cart_position",m_nodeHandle,ros::Duration(0.1));

        if(controllerPose == NULL || controllerPose->data.size() != 6)
        {
            ROS_WARN("Unable to retrieve KUKA controller pose !");
        }

        else
        {
            //TODO Assert good size (6)

            //Write the position in the file
            file << controllerPose->data[0]*0.001;
            file << ",";
            file << -controllerPose->data[1]*0.001;
            file << ",";
            file << -controllerPose->data[2]*0.001;
            file << ",";
            file << angles::from_degrees(controllerPose->data[5]) - M_PI;
            file << ",";
            file << -angles::from_degrees(controllerPose->data[4]);
            file << ",";
            file << -angles::from_degrees(controllerPose->data[3]);
            file << ",---,";
        }
    }

    //Get current joint values
    std::vector<double> currentJointsValues = getCurrentJointsValues();

    // Write the joint values in the file
    for (int i = 0; i < currentJointsValues.size(); i++)
    {
        if (i != 0)
        {
            file << ",";
        }
        file << currentJointsValues[i];
    }

    file << "\n";

    // Close the file
    file.close();
}

void Robot::saveCurrentPose(std::string fileName)
{
    // Create/open .csv file for positions recording
    std::ofstream file(fileName, std::ios::app);

    // Get current pose and compute roll, pitch and yaw angles
    geometry_msgs::Pose currentPose = getCurrentPose();

    tf2::Quaternion quaternion;
    tf2::fromMsg(currentPose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // Write the pose in the file
    file << currentPose.position.x;
    file << ",";
    file << currentPose.position.y;
    file << ",";
    file << currentPose.position.z;
    file << ",";
    file << roll;
    file << ",";
    file << pitch;
    file << ",";
    file << yaw;
    file << "\n";

    // Close the file
    file.close();
}

void Robot::logCurrentPose(std::string fileName, int index)
{
    // Create/open .log file for positions recording
    std::ofstream file(fileName, std::ios::app); 

    // Get current pose and compute rotation matrix
    geometry_msgs::Pose currentPose = getCurrentPose();

    tf2::Quaternion quaternion;
    tf2::fromMsg(currentPose.orientation, quaternion);
    tf2::Matrix3x3 matrix(quaternion);

    // Write header
    file << "0" << " " << "-1 " << " " << index << "\n";

    // Write the pose transformation matrix in the file
    file << matrix.getRow(0).getX() << " " << matrix.getRow(0).getY() << " " << matrix.getRow(0).getZ() << " " << currentPose.position.x << "\n";
    file << matrix.getRow(1).getX() << " " << matrix.getRow(1).getY() << " " << matrix.getRow(1).getZ() << " " << currentPose.position.y << "\n";
    file << matrix.getRow(2).getX() << " " << matrix.getRow(2).getY() << " " << matrix.getRow(2).getZ() << " " << currentPose.position.z << "\n";
    file << "0" << " " << "0" << " " << "0" << " " << "1" << "\n";

    // Close the file
    file.close();
}

void Robot::saveCurrentJointsValues(std::string fileName)
{
    // Create/open .csv file for positions recording
    std::ofstream file(fileName, std::ios::app);

    // Get current joint values
    std::vector<double> currentJointsValues = getCurrentJointsValues();

    // Write the joint values in the file
    for (int i = 0; i < currentJointsValues.size(); i++)
    {
        if (i != 0)
        {
            file << ",";
        }
        file << currentJointsValues[i];
    }

    file << "\n";

    // Close the file
    file.close();
}

// TODO clean !
void Robot::runMeasurementRoutine(std::vector<geometry_msgs::Pose> waypoints, bool initialMeasurement, bool saveStates, double yawOffset, bool visibilityConstraint, bool visualCheck)
{
    // Logging
    RobotLog log(waypoints);
    int initialWaypointIndex = log.getCurrentIndex();

    std::string measurementServerName, measurementServerStorageFolder;
    bool measurementServerDisplay;

    if(!m_nodeHandle.getParam("measurementServerName",measurementServerName) || measurementServerName == "")
    {
        ROS_WARN("No measurement server name specified !");
        measurementServerName = "";
    }
    else
    {
        ros::ServiceClient measurementServerCounterClient = m_nodeHandle.serviceClient<robot_arm_tools::Int>(measurementServerName + "_counter");
        robot_arm_tools::Int srv;
        srv.request.data = initialWaypointIndex;
        if (measurementServerCounterClient.call(srv))
        {
            ROS_INFO("Recovery taken into account for %s service", measurementServerName.c_str());
        }
        else
        {
            ROS_WARN("Recovery not taken into account for %s service: content may be overwitten !", measurementServerName.c_str());
        }
    }

    if (!m_nodeHandle.getParam("measurementServerDisplay", measurementServerDisplay))
    {
        ROS_WARN("No measurement server display specified, switching to no display");
        measurementServerDisplay = false;
    }

    if ((!m_nodeHandle.getParam("measurementServerStorageFolder", measurementServerStorageFolder) || measurementServerStorageFolder == "") && saveStates)
    {
        ROS_WARN("No measurement server storage folder specified, switching to /tmp/Measurements/");
        measurementServerStorageFolder = "/tmp/Measurements/";
    }

    if (saveStates)
    {
        std::experimental::filesystem::path filePathStates{measurementServerStorageFolder + "States.csv"};
        std::experimental::filesystem::path filePathTrajectory{measurementServerStorageFolder + "Trajectory.log"};

        // Check wether the storage CSV file exists
        if (std::experimental::filesystem::exists(filePathStates) && !log.isRecovered())
        {
            // Existing path + no recovery => wipe file.
            ROS_WARN("States and trajectory log files already exist : their contents will be overwritten !");
            std::experimental::filesystem::remove(filePathStates);
            std::experimental::filesystem::remove(filePathTrajectory);
            std::ofstream{filePathStates};
            std::ofstream{filePathTrajectory};
        }
        else if (!std::experimental::filesystem::exists(filePathStates))
        {
            // No existing path => create file.
            ROS_INFO("Creating states and trajectory log files ...");
            std::experimental::filesystem::create_directories(filePathStates.parent_path());
            std::experimental::filesystem::create_directories(filePathTrajectory.parent_path());
            std::ofstream{filePathStates};
            std::ofstream{filePathTrajectory};
        }
        else
        { /*Existing path + recovery => nothing.*/
        }
    }

    // Initialise measurement client
    ros::ServiceClient measurementClient;
    std_srvs::Empty request;
    bool measurementSuccess = true;

    if (measurementServerName != "")
    {
        // Get ROS service measurement server name and create client
        measurementClient = m_nodeHandle.serviceClient<std_srvs::Empty>(measurementServerName);
        measurementClient.waitForExistence();
    }

    int trajectoryIndex = 1;

    //Initial measurement
    //TODO Remove ?
    if(initialMeasurement)
    {
        ROS_INFO("Initial measurement...");
        ros::WallDuration(1.0).sleep();

        if(measurementServerName != "")
        {
            if(measurementClient.call(request))
            {
                ROS_INFO("Measurement - done !");
                measurementSuccess = true;

            }
            else
            {
                ROS_WARN("ERROR DURING MEASUREMENT !");
                measurementSuccess = false;
            }
        }

        if(saveStates && measurementSuccess)
        {
            saveCurrentState(measurementServerStorageFolder + "States.csv");
            logCurrentPose(measurementServerStorageFolder + "Trajectory.log",trajectoryIndex);
            trajectoryIndex++;
        }
    }

    // Acquisition loop
    tf2::Quaternion quaternion, offsetQuaternion;
    double lastYawOffset = 0.0;

    bool constrainedYaw = true;
    if (yawOffset < 0)
    {
        yawOffset = 0;
        constrainedYaw = false;
    }

    tf2::Quaternion debugQuaternion;
    double debugRoll, debugPitch, debugYaw;
    for (int i = 0; i < waypoints.size(); i++)
    {
        ROS_INFO("Waypoint %i out of %i", initialWaypointIndex + i + 1, initialWaypointIndex + (int)waypoints.size());

        tf2::fromMsg(waypoints[i].orientation, debugQuaternion);
        tf2::Matrix3x3(debugQuaternion).getRPY(debugRoll, debugPitch, debugYaw);
        ROS_DEBUG("%f %f %f %f %f %f", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z, debugRoll, debugPitch, debugYaw);

        m_visualTools.addFrame("waypoint", waypoints[i]);

        try
        {
            if (yawOffset != 0)
            {
                offsetQuaternion.setRPY(0.0, 0.0, lastYawOffset);
                tf2::fromMsg(waypoints[i].orientation, quaternion);
                waypoints[i].orientation = tf2::toMsg(quaternion * offsetQuaternion);
            }

            goToTarget(waypoints[i], visualCheck, true, constrainedYaw, visibilityConstraint);
            m_visualTools.deleteObject("waypoint");
        }

        catch (const std::out_of_range &e)
        {
            bool success = false;

            if (yawOffset != 0)
            {
                for (int j = 1; j < (2 * M_PI) / yawOffset; j++)
                {
                    try
                    {
                        ROS_WARN("Unreachable waypoint - Trying with a %f * M_PI yaw offset...", yawOffset / M_PI);
                        m_visualTools.deleteObject("waypoint");

                        // Adding yaw offset
                        offsetQuaternion.setRPY(0.0, 0.0, yawOffset);
                        tf2::fromMsg(waypoints[i].orientation, quaternion);
                        waypoints[i].orientation = tf2::toMsg(quaternion * offsetQuaternion);

                        m_visualTools.addFrame("waypoint", waypoints[i]);
                        goToTarget(waypoints[i], visualCheck, true, 0.0, visibilityConstraint);
                        m_visualTools.deleteObject("waypoint");

                        success = true;
                        lastYawOffset += j * yawOffset;
                        if (lastYawOffset >= 2 * M_PI)
                        {
                            lastYawOffset -= 2 * M_PI;
                        }

                        break;
                    }

                    catch (const std::out_of_range &e)
                    {
                        // Waypoint out of range
                        m_visualTools.deleteObject("waypoint");
                        continue;
                    }

                    catch (const std::runtime_error &e)
                    {
                        // Something went wrong...
                        log.save();
                        ROS_ERROR("Robot failure during execution !");
                        throw std::runtime_error("ROBOT EXECUTION FAILURE !");
                    }
                }
            }

            if (!success)
            {
                ROS_WARN("Skipping unreachable waypoint");

                // Waypoint not reached, but still added to logfile
                log.addFaultyWaypoint(log.getCurrentIndex(), "unreachable");
                log.iterateCurrentIndex();

                lastYawOffset = 0.0;
                continue;
            }
        }

        ros::WallDuration(1.0).sleep();

        if (measurementServerName != "")
        {
            if (measurementClient.call(request))
            {
                ROS_INFO("Measurement - done !");
                measurementSuccess = true;
            }
            else
            {
                ROS_WARN("ERROR DURING MEASUREMENT !");
                log.addFaultyWaypoint(log.getCurrentIndex(),"measurement_error");
                measurementSuccess = false;
            }
        }

        if(saveStates && measurementSuccess)
        {
            saveCurrentState(measurementServerStorageFolder + "States.csv");
            logCurrentPose(measurementServerStorageFolder + "Trajectory.log",trajectoryIndex);
            trajectoryIndex++;
        }

        //Waypoint completed successfully
        log.iterateCurrentIndex();
    }

    // Everything went fine : no log needed ;)
    log.remove();
}

void Robot::runMeasurementRoutine(std::vector<std::vector<double>> waypoints, bool initialMeasurement, bool saveStates, bool visualCheck)
{
    // Logging
    RobotLog log(waypoints);
    int initialWaypointIndex = log.getCurrentIndex();

    std::string measurementServerName, measurementServerStorageFolder;
    bool measurementServerDisplay;

    if(!m_nodeHandle.getParam("measurementServerName",measurementServerName) || measurementServerName == "")
    {
        ROS_WARN("No measurement server name specified");
        measurementServerName = "";
    }
    else
    {
        ros::ServiceClient measurementServerCounterClient = m_nodeHandle.serviceClient<robot_arm_tools::Int>(measurementServerName + "_counter");
        robot_arm_tools::Int srv;
        srv.request.data = initialWaypointIndex;
        if (measurementServerCounterClient.call(srv))
        {
            ROS_INFO("Recovery taken into account for %s service", measurementServerName.c_str());
        }
        else
        {
            ROS_WARN("Recovery not taken into account for %s service: content may be overwitten !", measurementServerName.c_str());
        }
    }

    if (!m_nodeHandle.getParam("measurementServerDisplay", measurementServerDisplay))
    {
        ROS_WARN("No measurement server display specified, switching to no display");
        measurementServerDisplay = false;
    }

    if ((!m_nodeHandle.getParam("measurementServerStorageFolder", measurementServerStorageFolder) || measurementServerStorageFolder == "") && saveStates)
    {
        ROS_WARN("No measurement server storage folder specified, switching to /tmp/Measurements/");
        measurementServerStorageFolder = "/tmp/Measurements/";
    }

    if (saveStates)
    {
        std::experimental::filesystem::path filePathStates{measurementServerStorageFolder + "States.csv"};
        std::experimental::filesystem::path filePathTrajectory{measurementServerStorageFolder + "Trajectory.log"};

        // Check wether the storage CSV file exists
        if (std::experimental::filesystem::exists(filePathStates) && !log.isRecovered())
        {
            // Existing path + no recovery => wipe file.
            ROS_WARN("States and trajectory log files already exist : their contents will be overwritten !");
            std::experimental::filesystem::remove(filePathStates);
            std::experimental::filesystem::remove(filePathTrajectory);
            std::ofstream{filePathStates};
            std::ofstream{filePathTrajectory};
        }
        else if (!std::experimental::filesystem::exists(filePathStates))
        {
            // No existing path => create file.
            ROS_INFO("Creating states and trajectory log files ...");
            std::experimental::filesystem::create_directories(filePathStates.parent_path());
            std::experimental::filesystem::create_directories(filePathTrajectory.parent_path());
            std::ofstream{filePathStates};
            std::ofstream{filePathTrajectory};
        }
        else
        { /*Existing path + recovery => nothing.*/
        }
    }

    // Initialise measurement client
    ros::ServiceClient measurementClient;
    std_srvs::Empty request;
    bool measurementSuccess = true;

    if (measurementServerName != "")
    {
        // Get ROS service measurement server name and create client
        measurementClient = m_nodeHandle.serviceClient<std_srvs::Empty>(measurementServerName);
        measurementClient.waitForExistence();
    }

    int trajectoryIndex = 1;

    //Initial measurement
    //TODO Remove ?
    if(initialMeasurement)
    {
        ROS_INFO("Initial measurement...");
        ros::WallDuration(1.0).sleep();

        if(measurementServerName != "")
        {
            if(measurementClient.call(request))
            {
                ROS_INFO("Measurement - done !");
                measurementSuccess = true;

            }
            else
            {
                ROS_WARN("ERROR DURING MEASUREMENT !");
                measurementSuccess = false;
            }
        }

        if(saveStates && measurementSuccess)
        {
            saveCurrentState(measurementServerStorageFolder + "States.csv");
            logCurrentPose(measurementServerStorageFolder + "Trajectory.log",trajectoryIndex);
            trajectoryIndex++;
        }
    }

    // Acquisition loop
    for (int i = 0; i < waypoints.size(); i++)
    {
        ROS_INFO("Waypoint %i out of %i", initialWaypointIndex + i + 1, initialWaypointIndex + (int)waypoints.size());

        try
        {
            goToTarget(waypoints[i], visualCheck, true);
        }

        catch (const std::out_of_range &e)
        {
            ROS_WARN("Skipping unreachable waypoint");

            // Waypoint not reached, but still added to logfile
            log.addFaultyWaypoint(log.getCurrentIndex(), "unreachable");
            log.iterateCurrentIndex();

            continue;
        }

        catch (const std::runtime_error &e)
        {
            // Something went wrong...
            log.save();
            ROS_ERROR("Robot failure during execution !");
            throw std::runtime_error("ROBOT EXECUTION FAILURE !");
        }

        ros::WallDuration(1.0).sleep();

        if (measurementServerName != "")
        {
            if (measurementClient.call(request))
            {
                ROS_INFO("Measurement - done !");
                measurementSuccess = true;
            }
            else
            {
                ROS_WARN("ERROR DURING MEASUREMENT !");
                log.addFaultyWaypoint(log.getCurrentIndex(),"measurement_error");
                measurementSuccess = false;
            }
        }

        if(saveStates && measurementSuccess)
        {
            saveCurrentState(measurementServerStorageFolder + "States.csv");
            logCurrentPose(measurementServerStorageFolder + "Trajectory.log",trajectoryIndex);
            trajectoryIndex++;
        }

        //Waypoint completed successfully
        log.iterateCurrentIndex();
    }

    // Everything went fine : no log needed ;)
    log.remove();
}

// TODO Pause compatibility !!
// TODO Review : this is not a trajectory following feature, but rather a cartesian motion feature !
void Robot::runTrajectory(std::vector<geometry_msgs::Pose> waypoints, double interpolationResolution, bool visualCheck, bool executeMotion)
{
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::MoveItErrorCodes error;

    // Compute trajectory
    double result = m_moveGroup->computeCartesianPath(waypoints, interpolationResolution, 0.0, trajectory, true, &error);

    // Planning error handling
    if (error.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR("Robot trajectory planning failed with error type : %i", error.val);
        throw std::runtime_error("ROBOT TRAJECTORY PLANNING FAILED");
    }

    // Incomplete planning handling
    if (result != 1.0)
    {
        ROS_ERROR("Robot trajectory planning failed at %f %%", result * 100);
        throw std::runtime_error("ROBOT TRAJECTORY PLANNING FAILED");
    }

    // Compute trajectory time stamps to match the velocity and acceleration ratios
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    robot_trajectory::RobotTrajectory robotTrajectory(m_robotModel, m_groupName);
    robotTrajectory.setRobotTrajectoryMsg(*m_moveGroup->getCurrentState(), trajectory);
    iptp.computeTimeStamps(robotTrajectory, m_velocityRatio, m_accelerationRatio);
    robotTrajectory.getRobotTrajectoryMsg(trajectory);

    // Set RViz trigger for visual check before trajectory execution
    if (visualCheck)
    {
        m_visualTools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
        m_visualTools.trigger();
        m_visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue !");
    }

    // Execute motion
    if (executeMotion)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        moveit::core::MoveItErrorCode error = m_moveGroup->execute(plan);

        // Execution error handling
        if (error != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_ERROR("Robot trajectory execution failed with error type : %i", error.val);
            throw std::runtime_error("ROBOT TRAJECTORY EXECUTION FAILED");
        }
    }
}

// TODO Trajectory action ?
void Robot::m_executePT(const robot_arm_tools::PoseTargetGoalConstPtr &goal)
{
    robot_arm_tools::PoseTargetFeedback feedback;
    robot_arm_tools::PoseTargetResult result;

    feedback.state = "Target recieved";
    m_poseTargetServer.publishFeedback(feedback);

    try
    {
        feedback.state = "Target forwarded";
        m_poseTargetServer.publishFeedback(feedback);

        goToTarget(goal->pose_target, true, true);

        feedback.state = "Target reached";
        m_poseTargetServer.publishFeedback(feedback);

        result.result = true;
        result.error_code.val = moveit::core::MoveItErrorCode::SUCCESS;
        m_poseTargetServer.setSucceeded(result);
    }
    catch (const std::out_of_range &e)
    {
        result.result = false;
        result.error_code.val = moveit::core::MoveItErrorCode::FAILURE;
        m_poseTargetServer.setAborted(result);
    }
}

void Robot::m_executeJVT(const robot_arm_tools::JointsValuesTargetGoalConstPtr &goal)
{
    robot_arm_tools::JointsValuesTargetFeedback feedback;
    robot_arm_tools::JointsValuesTargetResult result;

    feedback.state = "Target recieved";
    m_jointsValuesTargetServer.publishFeedback(feedback);

    try
    {
        feedback.state = "Target forwarded";
        m_jointsValuesTargetServer.publishFeedback(feedback);

        goToTarget(goal->joints_values_target, true, true);

        feedback.state = "Target reached";
        m_jointsValuesTargetServer.publishFeedback(feedback);

        result.result = true;
        result.error_code.val = moveit::core::MoveItErrorCode::SUCCESS;
        m_jointsValuesTargetServer.setSucceeded(result);
    }
    catch (const std::out_of_range &e)
    {
        result.result = false;
        result.error_code.val = moveit::core::MoveItErrorCode::FAILURE;
        m_jointsValuesTargetServer.setAborted(result);
    }
}

void Robot::m_executePW(const robot_arm_tools::PoseWaypointsGoalConstPtr &goal)
{
    robot_arm_tools::PoseWaypointsFeedback feedback;
    robot_arm_tools::PoseWaypointsResult result;

    feedback.state = "Waypoints recieved";
    m_poseWaypointsServer.publishFeedback(feedback);

    try
    {
        feedback.state = "Waypoints forwarded";
        m_poseWaypointsServer.publishFeedback(feedback);

        runMeasurementRoutine(goal->pose_waypoints);

        feedback.state = "Waypoints completed";
        m_poseWaypointsServer.publishFeedback(feedback);

        result.result = true;
        result.error_code.val = moveit::core::MoveItErrorCode::SUCCESS;
        m_poseWaypointsServer.setSucceeded(result);
    }
    catch (const std::out_of_range &e)
    {
        result.result = false;
        result.error_code.val = moveit::core::MoveItErrorCode::FAILURE;
        m_poseWaypointsServer.setAborted(result);
    }
}

void Robot::m_executeJVW(const robot_arm_tools::JointsValuesWaypointsGoalConstPtr &goal)
{
    std::vector<std::vector<double>> waypoints;
    std::vector<double> tmpVector;

    for (int i = 0; i < goal->joints_values_waypoints.size(); i++)
    {
        tmpVector.push_back(goal->joints_values_waypoints[i]);
        if (tmpVector.size() == m_jointsNumber)
        {
            waypoints.push_back(tmpVector);
            tmpVector.clear();
        }
    }

    robot_arm_tools::JointsValuesWaypointsFeedback feedback;
    robot_arm_tools::JointsValuesWaypointsResult result;

    feedback.state = "Waypoints recieved";
    m_jointsValuesWaypointsServer.publishFeedback(feedback);

    try
    {
        feedback.state = "Waypoints forwarded";
        m_jointsValuesWaypointsServer.publishFeedback(feedback);

        runMeasurementRoutine(waypoints);

        feedback.state = "Waypoints completed";
        m_jointsValuesWaypointsServer.publishFeedback(feedback);

        result.result = true;
        result.error_code.val = moveit::core::MoveItErrorCode::SUCCESS;
        m_jointsValuesWaypointsServer.setSucceeded(result);
    }
    catch (const std::out_of_range &e)
    {
        result.result = false;
        result.error_code.val = moveit::core::MoveItErrorCode::FAILURE;
        m_jointsValuesWaypointsServer.setAborted(result);
    }
}

void Robot::startActions()
{
    m_poseTargetServer.start();
    m_jointsValuesTargetServer.start();
    m_poseWaypointsServer.start();
    m_jointsValuesWaypointsServer.start();
}

void Robot::pauseCallback(const std_msgs::Bool::ConstPtr &msg)
{
    m_paused = msg->data;
}