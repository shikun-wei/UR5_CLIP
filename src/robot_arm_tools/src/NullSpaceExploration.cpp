#include "robot_arm_tools/NullSpaceExploration.h"
#include "robot_arm_tools/CustomGoalsMetrics.hpp"

// Bullet collision detection and distance computation
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#define EPSILON_POSITION 1e-6
#define EPSILON_ORIENTATION 1e-5
#define DELTA 1e-5
#define LOCAL_ITER 100
#define GLOBAL_ITER 100000000

Eigen::Index NullSpaceExploration::findMatching(const Eigen::VectorXd &key, const Eigen::MatrixXd &haystack, const Eigen::VectorXi &available, double &sign)
{
    Eigen::Index result = available.array().maxCoeff();
    double best_match = 0.0;
    for (unsigned int i = 0; i < available.rows(); ++i)
    {
        int index = available[i];
        if (index < 0) // index already taken
            continue;
        if (index >= haystack.cols())
            return result;
        double match = haystack.col(available[i]).transpose() * key;
        double abs_match = std::abs(match);
        if (abs_match > 0.5 && abs_match > best_match)
        {
            best_match = abs_match;
            result = index;
            sign = match > 0 ? 1.0 : -1.0;
        }
    }
    return result;
}

NullSpaceExploration::NullSpaceExploration(const Robot &robot, const Eigen::VectorXd &initialConfiguration, bool constrainedYaw) : m_initialConfiguration(initialConfiguration), m_constrainedYaw(constrainedYaw)
{
    m_jointModelGroup = std::make_shared<const moveit::core::JointModelGroup>(*robot.getRobotModel()->getJointModelGroup(robot.getGroupName()));
    // m_jointModelGroup.reset(robot.getRobotModel()->getJointModelGroup(robot.getGroupName())); SEGFAULT ON FREE !!
    m_robotState.reset(new moveit::core::RobotState(robot.getRobotModel()));
    m_endEffectorName = robot.getEndEffectorName();

    m_referencePose = m_robotState->getGlobalLinkTransform(m_endEffectorName).inverse();
    m_targetPose = Eigen::Isometry3d::Identity();
}

Eigen::MatrixXd NullSpaceExploration::computeJacobian(const Eigen::VectorXd &currentConfiguration, bool quaternionRepresentation)
{
    // Update joints poisitons
    m_robotState->setJointGroupPositions(m_jointModelGroup.get(), currentConfiguration);
    m_robotState->harmonizePositions(m_jointModelGroup.get());
    m_robotState->update();

    const robot_model::JointModel *root_joint_model = m_jointModelGroup->getJointModels()[0];
    Eigen::Isometry3d reference_transform = m_robotState->getGlobalLinkTransform(m_endEffectorName).inverse();

    int rows = quaternionRepresentation ? 7 : 6;
    int columns = m_jointModelGroup->getVariableCount();
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, columns);

    Eigen::Isometry3d link_transform = reference_transform * m_robotState->getGlobalLinkTransform(m_endEffectorName);
    Eigen::Vector3d point_transform = Eigen::Vector3d::Zero();

    Eigen::Vector3d joint_axis;
    Eigen::Isometry3d joint_transform;

    const robot_model::LinkModel *link = m_jointModelGroup->getLinkModel(m_endEffectorName);
    while (link)
    {
        const robot_model::JointModel *pjm = link->getParentJointModel();
        if (pjm->getVariableCount() > 0)
        {
            if (!m_jointModelGroup->hasJointModel(pjm->getName()))
            {
                link = pjm->getParentLinkModel();
                continue;
            }
            unsigned int joint_index = m_jointModelGroup->getVariableGroupIndex(pjm->getName());
            if (pjm->getType() == robot_model::JointModel::REVOLUTE)
            {
                joint_transform = reference_transform * m_robotState->getGlobalLinkTransform(link);
                joint_axis = joint_transform.rotation() * static_cast<const robot_model::RevoluteJointModel *>(pjm)->getAxis();
                jacobian.block<3, 1>(0, joint_index) =
                    jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
                jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
            }
            else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
            {
                joint_transform = reference_transform * m_robotState->getGlobalLinkTransform(link);
                joint_axis = joint_transform.rotation() * static_cast<const robot_model::PrismaticJointModel *>(pjm)->getAxis();
                jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
            }
            else if (pjm->getType() == robot_model::JointModel::PLANAR)
            {
                joint_transform = reference_transform * m_robotState->getGlobalLinkTransform(link);
                joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
                jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
                joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
                jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
                joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
                jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) + joint_axis.cross(point_transform - joint_transform.translation());
                jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
            }
            else
            {
                //ROS_ERROR("Unknown type of joint in Jacobian computation");   
            }
        }

        if (pjm == root_joint_model)
        {
            break;
        }
        link = pjm->getParentLinkModel();
    }

    if(quaternionRepresentation)
    {
        // Quaternion representation
        // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
        // d/dt ( [x] ) = 1/2 * [  w -z  y ]  * [ omega_1 ]
        //        [y]           [  z  w -x ]    [ omega_2 ]
        //        [z]           [ -y  x  w ]    [ omega_3 ]
        //        [w]           [ -x -y -z ]
        Eigen::Quaterniond q(link_transform.rotation());
        double w = q.w(), x = q.x(), y = q.y(), z = q.z();
        Eigen::MatrixXd quaternion_update_matrix(4, 3);
        quaternion_update_matrix << w, -z, y, z, w, -x, -y, x, w, -x, -y, -z;
        jacobian.block(3, 0, 4, columns) = 0.5 * quaternion_update_matrix * jacobian.block(3, 0, 3, columns);
    }

    else if(!m_constrainedYaw)
    {
        std::size_t rowToRemove = jacobian.rows() - 1;
        jacobian.block(rowToRemove, 0, rows - 1 - rowToRemove, columns) = jacobian.block(rowToRemove + 1, 0, rows - 1 - rowToRemove, columns);
        jacobian.conservativeResize(rows - 1, columns);
    }

    return jacobian;
}

void NullSpaceExploration::updateNullSpace(const Eigen::VectorXd &currentConfiguration)
{
    // Compute jacobian
    Eigen::MatrixXd jacobian = computeJacobian(currentConfiguration);

    // Compute jacobian SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.setThreshold(0.001);
    svd.compute(jacobian, Eigen::ComputeFullV);

    // Get nullspace dimension
    std::size_t nullSpaceDimension = svd.cols() - svd.rank();

    // initialisation case
    if (m_nullSpace.rows() == 0 && m_nullSpace.cols() == 0)
    {
        m_nullSpace.resize(svd.cols(), nullSpaceDimension);
        for (std::size_t i = 0; i < nullSpaceDimension; i++)
        {
            m_nullSpace.col(i) = svd.matrixV().col(svd.rank() + i);
        }
    }

    // Update case : find matching nullspace basis vector in the existing nullspace to ensure continuity
    else
    {
        Eigen::MatrixXd nullSpace(svd.cols(), nullSpaceDimension);

        Eigen::VectorXi available(nullSpaceDimension);
        for (std::size_t i = 0; i < nullSpaceDimension; i++)
        {
            available[i] = i;
        }

        for (std::size_t i = 0; i < nullSpaceDimension; i++)
        {
            double sign = 1.0;
            Eigen::VectorXd tmp = svd.matrixV().col(svd.rank() + i);
            Eigen::Index index = findMatching(tmp, m_nullSpace, available, sign);
            nullSpace.col(index) = sign * tmp;
            available[index] = -1;
        }
        m_nullSpace = nullSpace;
    }
}

Eigen::VectorXd NullSpaceExploration::findBestLocalConfiguration()
{
    // Compute targeted pose
    m_robotState->setJointGroupPositions(m_jointModelGroup.get(), m_initialConfiguration);
    m_robotState->harmonizePositions(m_jointModelGroup.get());
    m_robotState->update();
    m_referencePose = m_robotState->getGlobalLinkTransform(m_endEffectorName).inverse();

    // Compute initial fitness
    double bestFitness = fitness(m_initialConfiguration);
    Eigen::VectorXd bestConfiguration = m_initialConfiguration;

    ROS_INFO("Initial fitness : %f", bestFitness);
    std::cout << "Initial configuration : " << m_initialConfiguration << std::endl;

    double tmpFitness;
    Eigen::VectorXd intermediateConfiguration = m_initialConfiguration;
    Eigen::VectorXd tmpConfiguration = m_initialConfiguration;

    // TODO List of potential directions depending on nullspace dimension, non-working dimensions are deleted when the fitness is too high.
    for (std::size_t k = 0; k < GLOBAL_ITER; k++)
    {
        // Compute initial nullspace
        updateNullSpace(intermediateConfiguration);
        std::size_t nullSpaceDimension = m_nullSpace.cols();

        //ROS_INFO("Inter. fitness : %f", fitness(intermediateConfiguration));
        //std::cout << "Inter. configuration : " << tmpConfiguration << std::endl;

        Eigen::MatrixXd directions = Eigen::MatrixXd::Zero(nullSpaceDimension, 2 * nullSpaceDimension);
        for (std::size_t i = 0; i < nullSpaceDimension; i++)
        {
            directions(i, 2 * i) = DELTA;
            directions(i, 2 * i + 1) = -DELTA;
        }

        for (std::size_t i = 0; i < 2 * nullSpaceDimension; i++)
        {
            tmpConfiguration = intermediateConfiguration;

            for (std::size_t j = 0; j < LOCAL_ITER; j++)
            {
                tmpConfiguration = m_nullSpace * directions.col(i) + tmpConfiguration;
                updateNullSpace(tmpConfiguration);
            }

            tmpFitness = fitness(tmpConfiguration);
            //ROS_INFO("Fitness : %f", tmpFitness);
            //std::cout << "Configuration : " << tmpConfiguration << std::endl;
            if (tmpFitness < bestFitness)
            {
                bestFitness = tmpFitness;
                bestConfiguration = tmpConfiguration;
            }
        }    

        if (intermediateConfiguration == bestConfiguration)
        {
            break;
        }
        else
        {
            intermediateConfiguration = bestConfiguration;
        }
    }

    ROS_INFO("Final fitness : %f", bestFitness);
    std::cout << "Final configuration : " << bestConfiguration << std::endl;

    // Compute pose error
    m_robotState->setJointGroupPositions(m_jointModelGroup.get(), bestConfiguration);
    m_robotState->harmonizePositions(m_jointModelGroup.get());
    m_robotState->update();
    Eigen::Isometry3d finalPose = m_referencePose * m_robotState->getGlobalLinkTransform(m_endEffectorName);
    Eigen::VectorXd errorVector = Eigen::VectorXd::Zero(7);
    Eigen::Vector3d deltaPosition = m_targetPose.translation() - finalPose.translation();
    errorVector.head(3) = deltaPosition;

    Eigen::Quaterniond deltaOrientation = Eigen::Quaterniond(m_targetPose.rotation()) * Eigen::Quaterniond(finalPose.rotation()).inverse();

    if(!m_constrainedYaw)
    {
        //Pure witchcraft, but it works
        Eigen::AngleAxisd angleAxis(deltaOrientation);
        Eigen::Vector3d axis = angleAxis.axis();
        axis[2] = 0.0;
        deltaOrientation = Eigen::Quaterniond(Eigen::AngleAxisd(angleAxis.angle(),axis));
    }

    errorVector.tail(4) = deltaOrientation.coeffs();

    double errorPosition = deltaPosition.norm();
    Eigen::AngleAxisd deltaPositionAngle(deltaOrientation);
    double errorOrientation = deltaPositionAngle.angle();

    ROS_INFO("Final error to desired pose : %f %f", errorPosition, errorOrientation);

    return (bestConfiguration);
}

VisibilityNullSpaceExploration::VisibilityNullSpaceExploration(const Robot &robot, const Eigen::VectorXd &initialConfiguration, bool constrainedYaw, const Eigen::Vector3d &visibilityAxis, double visibilityDistance, double visibilityRadius) : NullSpaceExploration(robot, initialConfiguration, constrainedYaw), m_visibilityAxis(visibilityAxis), m_visibilityDistance(visibilityDistance), m_visibilityRadius(visibilityRadius)
{
    m_planningScene = robot.getPlanningScene();

    m_planningScene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);

    // Create and attach visibility cone to the robot
    shapes::ShapeConstPtr visibilityCone = shapes::ShapeConstPtr(new shapes::Cylinder(m_visibilityRadius, m_visibilityDistance));

    Eigen::Isometry3d pose;

    // Translation
    pose = Eigen::Translation3d(0.5 * m_visibilityDistance * m_visibilityAxis);

    // Rotation
    Eigen::Vector3d orthogonalAxis = m_visibilityAxis.cross(Eigen::Vector3d::UnitX());
    if (orthogonalAxis.norm() < 0.0001)
    {
        orthogonalAxis = m_visibilityAxis.cross(Eigen::Vector3d::UnitY());
    }
    pose.rotate(Eigen::AngleAxis<double>(M_PI, orthogonalAxis));

    // Attach visibility cone
    m_robotState->attachBody("visibility_cone", pose, {visibilityCone}, EigenSTL::vector_Isometry3d(1, Eigen::Isometry3d::Identity()), std::set<std::string>(), m_endEffectorName);

    // Handle allowed collisions matrix

    m_allowedCollisionMatrixDistance = m_planningScene->getAllowedCollisionMatrix();
    std::vector<std::string> collisionObjectRobot;
    m_allowedCollisionMatrixDistance.getAllEntryNames(collisionObjectRobot);

    collision_detection::AllowedCollision::Type collisionType;
    bool output;

    int collisionObjectCounter = 0;
    std::vector<std::string> collisionObjectBuffer;

    for (std::vector<std::string>::iterator it = collisionObjectRobot.begin(); it != collisionObjectRobot.end(); it++)
    {
        //ROS_INFO("%s", it->c_str());
        // Get collision object default behaviour: output is set to false if no default behaviour is defined
        output = m_allowedCollisionMatrixDistance.getDefaultEntry(*it, collisionType);

        // Remove robot collisions objects from the ACM (true => no collision checking, false => collision checking)
        m_allowedCollisionMatrixDistance.setEntry(*it, true);
        m_allowedCollisionMatrixDistance.setDefaultEntry(*it, true);

        // Ignore collisions with end effector, as it won't change over time
        if (*it == m_endEffectorName)
        {
            m_allowedCollisionMatrixDistance.setEntry("visibility_cone", *it, true);
        }

        // If the robot collision object does have a ALWAYS default collision behaviour (i.e. coarse mesh for faster gazebo computation): keep it for visibility cone
        else if (output && (collisionType == collision_detection::AllowedCollision::Type::ALWAYS))
        {
            m_allowedCollisionMatrixDistance.setEntry("visibility_cone", *it, false);
            collisionObjectCounter++;
        }
        // Else, store it for the possible case where no default behaviour is ever defined: in this case, all the collisions objects (except the EE) are kept for the visibility cone
        else
        {
            m_allowedCollisionMatrixDistance.setEntry("visibility_cone", *it, true);
            collisionObjectBuffer.push_back(*it);
        }
    }

    // Case where not default behavious is ever defined: all the collision objects (except the EE) are kept for the visibility cone
    if (collisionObjectCounter == 0)
    {
        for (std::vector<std::string>::iterator it = collisionObjectBuffer.begin(); it != collisionObjectBuffer.end(); it++)
        {
            m_allowedCollisionMatrixDistance.setEntry("visibility_cone", *it, false);
        }
    }

    // Handle attached collision objects
    std::vector<const moveit::core::AttachedBody *> attachedBodies;
    m_robotState->getAttachedBodies(attachedBodies);
    for (std::vector<const moveit::core::AttachedBody *>::iterator it = attachedBodies.begin(); it != attachedBodies.end(); it++)
    {
        // Beware : the visibility cone is an attached object !
        if ((*it)->getName() != "visibility_cone")
        {
            //ROS_INFO("%s", (*it)->getName().c_str());
            // Remove attached collisions objects from the ACM (true => no collision checking)
            m_allowedCollisionMatrixDistance.setEntry((*it)->getName(), true);
            m_allowedCollisionMatrixDistance.setDefaultEntry((*it)->getName(), true);
        }
    }

    // Handle world collision objects
    std::vector<std::string> collisionObjectWorld = m_planningScene->getWorld()->getObjectIds();
    for (std::vector<std::string>::iterator it = collisionObjectWorld.begin(); it != collisionObjectWorld.end(); it++)
    {
        //ROS_INFO("%s", it->c_str());
        // Remove world collisions objects from the ACM (true => no collision checking)
        m_allowedCollisionMatrixDistance.setEntry(*it, true);
        m_allowedCollisionMatrixDistance.setDefaultEntry(*it, true);
    }

    m_collisionRequestDistance.contacts = true;
    m_collisionRequestDistance.distance = true;
    m_collisionRequestDistance.max_contacts = 100;
    m_collisionRequestDistance.max_contacts_per_pair = 1;

    // Ignore collisions with visibility cone
    m_allowedCollisionMatrixCollision = m_planningScene->getAllowedCollisionMatrix();
    m_allowedCollisionMatrixCollision.setEntry("visibility_cone", true);
    m_allowedCollisionMatrixCollision.setDefaultEntry("visibility_cone", true);
}

double VisibilityNullSpaceExploration::fitness(const Eigen::VectorXd &currentConfiguration)
{
    double fitness = 0.0;

    // Update joints poisitons
    m_robotState->setJointGroupPositions(m_jointModelGroup.get(), currentConfiguration);
    m_robotState->harmonizePositions(m_jointModelGroup.get());
    fitness += boundsAvoidanceMetric(m_robotState->satisfiesBounds(m_jointModelGroup.get()));

    m_robotState->update();

    // Check collisions
    collision_detection::CollisionResult collisionResult;
    m_planningScene->checkCollision(m_collisionRequestCollision, collisionResult, *m_robotState, m_allowedCollisionMatrixCollision);
    fitness += collisionAvoidanceMetric(collisionResult.collision);

    // Check distance to the target pose
    Eigen::Isometry3d finalPose = m_referencePose * m_robotState->getGlobalLinkTransform(m_endEffectorName);

    // Compute pose error
    Eigen::VectorXd errorVector = Eigen::VectorXd::Zero(7);
    Eigen::Vector3d deltaPosition = m_targetPose.translation() - finalPose.translation();
    errorVector.head(3) = deltaPosition;

    Eigen::Quaterniond deltaOrientation = Eigen::Quaterniond(m_targetPose.rotation()) * Eigen::Quaterniond(finalPose.rotation()).inverse();

    if(!m_constrainedYaw)
    {
        //Pure witchcraft, but it works
        Eigen::AngleAxisd angleAxis(deltaOrientation);
        Eigen::Vector3d axis = angleAxis.axis();
        axis[2] = 0.0;
        deltaOrientation = Eigen::Quaterniond(Eigen::AngleAxisd(angleAxis.angle(),axis));
    }

    errorVector.tail(4) = deltaOrientation.coeffs();

    double errorPosition = deltaPosition.norm();
    Eigen::AngleAxisd deltaPositionAngle(deltaOrientation);
    double errorOrientation = deltaPositionAngle.angle();
    if(errorPosition > EPSILON_POSITION || errorOrientation > EPSILON_ORIENTATION)
    {
        fitness += collisionAvoidanceMetric(true);
    }

    // Check distance to the visibility cone
    collisionResult.clear();
    m_planningScene->checkCollision(m_collisionRequestDistance, collisionResult, *m_robotState, m_allowedCollisionMatrixDistance);

    for (std::map<const std::pair<std::string, std::string>, std::vector<collision_detection::Contact>>::iterator it = collisionResult.contacts.begin(); it != collisionResult.contacts.end(); it++)
    {
        // Compute the visibility constraint metric
        fitness += visibilityMetric((it->second)[0]);
    }

    return fitness;
}