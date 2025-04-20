#include "robot_arm_tools/PandaGripper.h"

PandaGripper::PandaGripper(const Robot& robot) : m_gripperMoveGroup("hand"), m_simulation(true), m_homingActionClient("franka_gripper/homing",true), m_graspActionClient("franka_gripper/grasp",true), m_velocityRatio(1.0)
{
    //Check wehter the robot is equiped with a Panda gripper
    if(robot.getToolName() != "hand" || robot.getName() != "panda") 
    {
        //TODO Differenciate robot name and robot type
        throw std::invalid_argument("CANNOT FIND GRIPPER ON ROBOT OR INVALID ROBOT TYPE!");
    }

    //Get simulation state
    if(!m_nodeHandle.getParam("simulation", m_simulation))
    {
        ROS_ERROR("Unable to retrieve simulation status !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!m_simulation)
    {
        //Wait for the action servers to start...
        m_homingActionClient.waitForServer();
        m_graspActionClient.waitForServer();
    }
}
void PandaGripper::moveGripper(bool visualCheck, bool executeMotion)
{
    //Plan motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = m_gripperMoveGroup.plan(plan);

    //Planning error handling
    if(error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_ERROR("Gripper planning failed with error type : %i",error.val);
        throw std::runtime_error("GRIPPER PLANNING FAILED");
    }

    //Set RViz trigger for visual check
    if(visualCheck)
    {
        m_visualTools.trigger();
        m_visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue !"); 
    }

    //Execute motion
    if(executeMotion)
    {
        error = m_gripperMoveGroup.execute(plan);

        //Execution error handling
        if(error != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_ERROR("Gripper execution failed with error type : %i",error.val);
            throw std::runtime_error("GRIPPER EXECUTION FAILED");
        }
    }
}

void PandaGripper::init(bool visualCheck, bool executeMotion)
{
    //RViz visual check
    if(visualCheck)
    {
        //Send joint target and plan motion
        m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("open"));
        moveGripper(false,false);
        m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("close"));
        moveGripper(true,false);
    }

    if(executeMotion && !m_simulation)
    {
        franka_gripper::HomingGoal homingAction;
        m_homingActionClient.sendGoal(homingAction);

        bool success = m_homingActionClient.waitForResult(ros::Duration(30.0));

        //Execution error handling
        if(!success)
        {
            throw std::runtime_error("HOMING EXECUTION FAILED");
        }
    }

    else
    {
        //Send joint target and plan motion
        m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("open"));
        moveGripper(false,true);
        m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("close"));
        moveGripper(false,true);
    }
}

void PandaGripper::close(double targetWidth, bool visualCheck, bool executeMotion)
{
    //Send joint target and plan motion
    m_gripperMoveGroup.setJointValueTarget(std::vector<double> {targetWidth/2,targetWidth/2});
    moveGripper(visualCheck,executeMotion);
}

void PandaGripper::grasp(double targetForce, bool visualCheck, bool executeMotion)
{      
    //RViz visual check
    if(visualCheck)
    {
        //Send joint target and plan motion
        m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("close"));
        moveGripper(true,false);
    }

    if(executeMotion && !m_simulation)
    {
        franka_gripper::GraspGoal graspAction;

        graspAction.width = 0.0;
        graspAction.speed = m_velocityRatio*0.1;
        graspAction.force = targetForce;
        graspAction.epsilon.inner = 0.0;
        graspAction.epsilon.outer = 0.035*2;

        m_graspActionClient.sendGoal(graspAction);

        bool success = m_graspActionClient.waitForResult(ros::Duration(30.0));

        //Execution error handling
        if(!success)
        {
            throw std::runtime_error("GRASPING EXECUTION FAILED");
        }
    }

    else
    {
        //Send joint target and plan motion
        moveGripper(false,true);
    }
}

void PandaGripper::open(bool visualCheck, bool executeMotion)
{
    //Send joint target and plan motion
    m_gripperMoveGroup.setJointValueTarget(m_gripperMoveGroup.getNamedTargetValues("open"));
    moveGripper(visualCheck,executeMotion);
}

void PandaGripper::setVelocity(double velocityRatio)
{
    m_gripperMoveGroup.setMaxVelocityScalingFactor(velocityRatio);
    m_velocityRatio = velocityRatio;
}

void PandaGripper::setAcceleration(double accelerationRatio)
{
    m_gripperMoveGroup.setMaxAccelerationScalingFactor(accelerationRatio);
}