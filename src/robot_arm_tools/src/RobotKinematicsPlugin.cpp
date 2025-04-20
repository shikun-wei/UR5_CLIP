#include "robot_arm_tools/RobotKinematicsPlugin.h"

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <random>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RobotKinematicsPlugin, kinematics::KinematicsBase)

//Builds transformation matrix from equivalent row-wise vector
KDL::Frame fromArray(const std::vector<double>& input)
{
    KDL::Frame output;

    for(int i = 0; i < 3; i++)
    {
        output.p(i) = input[4*i+3];

        for(int j = 0; j < 3; j++)
        {
            output.M(i,j) = input[4*i+j];
        }
    }

    return(output);
}

//Builds transformation matrix from equivalent pose
KDL::Frame fromPose(const geometry_msgs::Pose& input)
{
    KDL::Frame output;

    output.p.data[0] = input.position.x;
    output.p.data[1] = input.position.y;
    output.p.data[2] = input.position.z;

    output.M = KDL::Rotation::Quaternion(input.orientation.x,input.orientation.y,input.orientation.z,input.orientation.w);

    return(output);
}

//Builds pose from equivalent transformation matrix 
geometry_msgs::Pose fromFrame(KDL::Frame input)
{
    geometry_msgs::Pose output;

    output.position.x = input.p.data[0];
    output.position.y = input.p.data[1];
    output.position.z = input.p.data[2];

    input.M.GetQuaternion(output.orientation.x,output.orientation.y,output.orientation.z,output.orientation.w);

    return(output);
}

RobotKinematicsPlugin::RobotKinematicsPlugin() : kdl_kinematics_plugin::KDLKinematicsPlugin::KDLKinematicsPlugin() {}

RobotKinematicsPlugin::RobotKinematicsPlugin(std::vector<std::vector<double>> jointsLimits,geometry_msgs::Transform toolTransform,geometry_msgs::Transform baseTransform) : kdl_kinematics_plugin::KDLKinematicsPlugin::KDLKinematicsPlugin()
{
    //Retriveing robot model parameters
    if(!m_nodeHandle.getParam("jointsAxis",m_jointsAxis))
    {
        ROS_ERROR("Unable to retrieve joints axis !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    std::vector<double> tmpMatrix;
    if(!m_nodeHandle.getParam("robotBase",tmpMatrix))
    {
        ROS_ERROR("Unable to retrieve robot base transformation !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    m_robotBase = fromArray(tmpMatrix);

    if(!m_nodeHandle.getParam("robotFlange",tmpMatrix))
    {
        ROS_ERROR("Unable to retrieve robot flange transformation !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    m_robotFlange = fromArray(tmpMatrix);

    if(!m_nodeHandle.getParam("robotParameters",m_robotParameters))
    {
        ROS_ERROR("Unable to retrieve robot parameters !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //Robot base
    geometry_msgs::Pose basePose;
    basePose.position.x = baseTransform.translation.x;
    basePose.position.y = baseTransform.translation.y;
    basePose.position.z = baseTransform.translation.z;
    
    if(baseTransform.rotation.x == 0 && baseTransform.rotation.y == 0 && baseTransform.rotation.z == 0 && baseTransform.rotation.w == 0){baseTransform.rotation.w = 1;}

    basePose.orientation = baseTransform.rotation;

    ROS_WARN("%f,%f,%f,%f",basePose.orientation.x,basePose.orientation.y,basePose.orientation.z,basePose.orientation.z);
    KDL::Segment robotBase = KDL::Segment(KDL::Joint(KDL::Joint::None),fromPose(basePose)*m_robotBase);
    
    m_robotChain.addSegment(robotBase);
    m_linksNames.push_back("base_link");

    //Robot joints and links
    for(int i = 0; i < m_jointsAxis.size(); i++)
    {
        //Joint rotation and offset
        char jointOrientation = abs(m_jointsAxis[i]);
        char jointDirection = (m_jointsAxis[i] > 0) - (m_jointsAxis[i] < 0);
        KDL::Joint joint(jointOrientation == 1 ? KDL::Joint::RotX : (jointOrientation == 2 ? KDL::Joint::RotY : KDL::Joint::RotZ),jointDirection,jointDirection*m_robotParameters[7*i]);

        //Link translation and rotation
        KDL::Vector linkTranslation(m_robotParameters[1 + 7*i],m_robotParameters[2 + 7*i],m_robotParameters[3 + 7*i]);
        KDL::Rotation linkRotation = KDL::Rotation::RPY(m_robotParameters[4 + 7*i],m_robotParameters[5 + 7*i],m_robotParameters[6 + 7*i]);

        KDL::Segment segment = KDL::Segment(joint,KDL::Frame(linkRotation,linkTranslation));
        m_robotChain.addSegment(segment);
        m_jointsNames.push_back("joint_" + i+1);
        m_linksNames.push_back("link_" + i+1);
    }

    //Robot flange
    KDL::Segment robotFlange = KDL::Segment(KDL::Joint(KDL::Joint::None),m_robotFlange);
    m_robotChain.addSegment(robotFlange);
    m_linksNames.push_back("robot_flange");

    //Robot tool
    geometry_msgs::Pose toolPose;
    toolPose.position.x = toolTransform.translation.x;
    toolPose.position.y = toolTransform.translation.y;
    toolPose.position.z = toolTransform.translation.z;
    
    if(toolTransform.rotation.x == 0 && toolTransform.rotation.y == 0 && toolTransform.rotation.z == 0 && toolTransform.rotation.w == 0){toolTransform.rotation.w = 1;}

    toolPose.orientation = toolTransform.rotation;

    KDL::Segment robotTool = KDL::Segment(KDL::Joint(KDL::Joint::None),fromPose(toolPose));
    m_robotChain.addSegment(robotTool);
    m_linksNames.push_back("robot_tool");

    //Robot joints positions limits
    m_jointsLimitsMin.resize(m_jointsAxis.size());
    m_jointsLimitsMax.resize(m_jointsAxis.size());
    
    for(int i = 0; i < m_jointsAxis.size(); i++)
    {
        m_jointsLimitsMin(i) = jointsLimits[i][0];
        m_jointsLimitsMax(i) = jointsLimits[i][1];
        m_lastJointsValues.push_back(0.0);
    }

    //Solvers
    m_solverFK.reset(new KDL::ChainFkSolverPos_recursive(m_robotChain));
    m_solverIKV.reset(new KDL::ChainIkSolverVel_pinv(m_robotChain));
    m_solverIK.reset(new KDL::ChainIkSolverPos_NR_JL(m_robotChain,m_jointsLimitsMin,m_jointsLimitsMax,*m_solverFK,*m_solverIKV));

    std::srand(std::time(nullptr));

    ROS_INFO("ROBOT MODEL SETUP OK");
}

KDL::JntArray RobotKinematicsPlugin::generateRandomJointsValues() const
{
    KDL::JntArray jointsValues(m_jointsAxis.size());
    for(int i = 0; i < m_jointsAxis.size(); i++)
    {
        jointsValues(i) = m_jointsLimitsMin(i) + ((double)std::rand()/(double)RAND_MAX)*(m_jointsLimitsMax(i)-m_jointsLimitsMin(i));
    }
    return(jointsValues);
}

KDL::JntArray RobotKinematicsPlugin::generateRandomJointsValuesNearBy(KDL::JntArray nearByJointsValues, std::vector<double> jointsValuesRange) const
{
    KDL::JntArray jointsValues = nearByJointsValues;
    for(int i = 0; i < jointsValuesRange.size(); i++)
    {
        jointsValues(i) += ((2*(double)std::rand()/(double)RAND_MAX) - 1)*jointsValuesRange[i];
    }
    return(jointsValues);
}

geometry_msgs::Pose RobotKinematicsPlugin::forwardKinematics(std::vector<double> desiredJointsValues) const
{
    KDL::Frame outputFrame;
    KDL::JntArray inputJointsValues(m_jointsAxis.size());
    for(int i = 0; i < m_jointsAxis.size(); i++)
    {
        if(desiredJointsValues[i] > m_jointsLimitsMax(i) || desiredJointsValues[i] < m_jointsLimitsMin(i))
        {
            ROS_ERROR("FK exited with out of range input");
            throw std::out_of_range("FK EXITED OUT OF RANGE"); 
        }

        inputJointsValues(i) = desiredJointsValues[i];
    }
    
    int result = m_solverFK->JntToCart(inputJointsValues,outputFrame);

    if(result < 0)
    {
        ROS_ERROR("FK exited with error : %s",m_solverFK->strError(result));
        throw std::out_of_range("FK EXITED WITH ERROR");
    }

    geometry_msgs::Pose outputPose;
    outputPose.position.x = outputFrame.p.data[0];
    outputPose.position.y = outputFrame.p.data[1];
    outputPose.position.z = outputFrame.p.data[2];
    outputFrame.M.GetQuaternion(outputPose.orientation.x,outputPose.orientation.y,outputPose.orientation.z,outputPose.orientation.w);

    return(outputPose);
}

std::vector<double> RobotKinematicsPlugin::inverseKinematics(geometry_msgs::Pose desiredPose, std::vector<double> initialGuess, double timeout)
{
    KDL::JntArray outputJointsValues(m_jointsAxis.size());
    KDL::JntArray initialJointsValues(initialGuess.size());
    for(int i = 0; i < initialGuess.size(); i++)
    {
        initialJointsValues(i) = initialGuess[i];
    }

    KDL::Frame inputFrame;
    inputFrame.p = KDL::Vector(desiredPose.position.x,desiredPose.position.y,desiredPose.position.z);
    inputFrame.M = KDL::Rotation::Quaternion(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);

    int result = -1;
    ros::WallTime startTime = ros::WallTime::now();

    while((ros::WallTime::now() - startTime).toSec() <= timeout && result < 0)
    {
        result = m_solverIK->CartToJnt(initialJointsValues,inputFrame,outputJointsValues);

        if(result < 0)
        {
            initialJointsValues = generateRandomJointsValues();
        }
    }

    if(result < 0)
    {
        ROS_ERROR("IK timed out and exited with error : %s",m_solverIK->strError(result));
        throw std::out_of_range("IK EXITED WITH ERROR");
    }

    m_lastJointsValues.clear();
    std::vector<double> outputVector;
    for(int i = 0; i < m_jointsAxis.size(); i++)
    {
        outputVector.push_back(outputJointsValues(i));
        m_lastJointsValues.push_back(outputJointsValues(i));
    }

    return(outputVector);
}

bool RobotKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
    std::vector<double> consistency_limits;
    //Limit search to a single attempt by setting a timeout of zero
    return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool RobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool RobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool RobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code, options);
}

bool RobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
    return kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback,error_code, options);
}

bool RobotKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles, std::vector<geometry_msgs::Pose>& poses) const
{
    return kdl_kinematics_plugin::KDLKinematicsPlugin::getPositionFK(link_names,joint_angles,poses);
}

bool RobotKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name, const std::string& base_frame, const std::vector<std::string>& tip_frames, double search_discretization)
{
    return kdl_kinematics_plugin::KDLKinematicsPlugin::initialize(robot_model, group_name, base_frame, tip_frames, search_discretization);
}

const std::vector<std::string>& RobotKinematicsPlugin::getJointNames() const
{
    return kdl_kinematics_plugin::KDLKinematicsPlugin::getJointNames();
}

const std::vector<std::string>& RobotKinematicsPlugin::getLinkNames() const
{
    return kdl_kinematics_plugin::KDLKinematicsPlugin::getLinkNames();
}