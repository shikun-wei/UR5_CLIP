/**
 * \file CustomGoals.hpp
 * \brief Header file for custom inverse kinematics resolution goals.
 *
 * Header file for the the custom inverse kinematics resolution goals used by the bio_ik plugin.
 * Remark - The used syntax slightly differs from the usual one to match bio_ik syntax (maybe some PRs one day...)
 *
 */

#pragma once

#include <bio_ik/goal.h>
#include <bio_ik/robot_info.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>

#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

// robot_arm_tools
#include "robot_arm_tools/Robot.h"

#include "robot_arm_tools/CustomGoalsMetrics.hpp"

/*! \class PlanningSceneGoal
 * \brief Inverse kinematics resolution goal enforcing a planning scene related goal (collision, distance, etc.)
 */
class PlanningSceneGoal : public bio_ik::LinkGoalBase
{
    protected:
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_; /*!< Planning scene */
        std::shared_ptr<moveit::core::RobotState> robot_state_;         /*!< Robot state */

        collision_detection::AllowedCollisionMatrix acm_;         /*!< The allowed collisions matrix */
        collision_detection::CollisionRequest collision_request_; /*!< The standard collision request*/

    public:
        PlanningSceneGoal(const Robot &robot)
        {
            planning_scene_ = robot.getPlanningScene();
            acm_ = planning_scene_->getAllowedCollisionMatrix();
            robot_state_.reset(new moveit::core::RobotState(robot.getRobotModel()));
        }

        PlanningSceneGoal(const Robot &robot, const std::string &link_name, double weight = 1.0) : bio_ik::LinkGoalBase(link_name, weight)
        {
            planning_scene_ = robot.getPlanningScene();
            acm_ = planning_scene_->getAllowedCollisionMatrix();
            robot_state_.reset(new moveit::core::RobotState(robot.getRobotModel()));
        }
};

/*! \class VisibilityGoal
 * \brief Inverse kinematics resolution goal enforcing a visibility constraint modeled as a visibility cone, from which the robot must lay the further away
 */
class VisibilityGoal : public PlanningSceneGoal
{
    private:

        tf2::Vector3 visibility_axis_; /*!< The axis along which the visibility cone is oriented*/
        double visibility_distance_;   /*!< The height of the visibility cone (i.e. the distance between the sensor and the object of interest)*/
        double visibility_radius_;     /*< The base radius of the visibility cone (i.e. the sensor aperture on hte object of interest)*/

        // int const  *ptr; // ptr is a pointer to constant int
        // int *const ptr;  // ptr is a constant pointer to int

        bool fixed_cone_; /*!< Wether the visibility cone is fixed or not (i.e. Wether the end effector position, roll and pitch are fixed or not) - This hypothesis speed up computation time by supressing collision detection with fixed objects*/

        /*!
        *  \brief Initializes the elements required for the collision environment
        */
        void init()
        {
            planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);

            // Setup collision request : get distances to collision for each pair of collision objects
            collision_request_.contacts = true;
            collision_request_.distance = true;
            collision_request_.max_contacts = 50;
            collision_request_.max_contacts_per_pair = 1;

            // Create visibility cone shape
            shapes::ShapeConstPtr visibilityCone = shapes::ShapeConstPtr(new shapes::Cylinder(visibility_radius_, visibility_distance_));

            // Move the cone to the end effector position
            Eigen::Isometry3d pose;
            Eigen::Vector3d visibilityAxis(visibility_axis_.x(), visibility_axis_.y(), visibility_axis_.z());

            // Translation
            pose = Eigen::Translation3d(0.5 * visibility_distance_ * visibilityAxis);

            // Rotation
            Eigen::Vector3d orthogonalAxis = visibilityAxis.cross(Eigen::Vector3d::UnitX());
            if (orthogonalAxis.norm() < 0.0001)
            {
                orthogonalAxis = visibilityAxis.cross(Eigen::Vector3d::UnitY());
            }
            pose.rotate(Eigen::AngleAxis<double>(M_PI, orthogonalAxis));

            // Attach visibility cone
            robot_state_->attachBody("visibility_cone", pose, {visibilityCone}, EigenSTL::vector_Isometry3d(1, Eigen::Isometry3d::Identity()), std::set<std::string>(), getLinkName());

            // Handle robot collision objects
            std::vector<std::string> collisionObjectRobot;
            acm_.getAllEntryNames(collisionObjectRobot);

            collision_detection::AllowedCollision::Type collisionType;
            bool output;

            int collisionObjectCounter = 0;
            std::vector<std::string> collisionObjectBuffer;

            for (std::vector<std::string>::iterator it = collisionObjectRobot.begin(); it != collisionObjectRobot.end(); it++)
            {
                // ROS_INFO("%s",it->c_str());
                // Get collision object default behaviour: output is set to false if no default behaviour is defined
                output = acm_.getDefaultEntry(*it, collisionType);

                // Remove robot collisions objects from the ACM (true => no collision checking, false => collision checking)
                acm_.setEntry(*it, true);
                acm_.setDefaultEntry(*it, true);

                // Ignore collisions with end effector, as it won't change over time
                if (*it == getLinkName())
                {
                    acm_.setEntry("visibility_cone", *it, true);
                }

                // If the robot collision object does have a ALWAYS default collision behaviour (i.e. coarse mesh for faster gazebo computation): keep it for visibility cone
                else if (output && (collisionType == collision_detection::AllowedCollision::Type::ALWAYS))
                {
                    acm_.setEntry("visibility_cone", *it, false);
                    collisionObjectCounter++;
                }
                // Else, store it for the possible case where no default behaviour is ever defined: in this case, all the collisions objects (except the EE) are kept for the visibility cone
                else
                {
                    acm_.setEntry("visibility_cone", *it, true);
                    collisionObjectBuffer.push_back(*it);
                }
            }

            // Case where not default behavious is ever defined: all the collision objects (except the EE) are kept for the visibility cone
            if (collisionObjectCounter == 0)
            {
                for (std::vector<std::string>::iterator it = collisionObjectBuffer.begin(); it != collisionObjectBuffer.end(); it++)
                {
                    acm_.setEntry("visibility_cone", *it, false);
                }
            }

            // Handle attached collision objects
            std::vector<const moveit::core::AttachedBody *> attachedBodies;
            robot_state_->getAttachedBodies(attachedBodies);
            for (std::vector<const moveit::core::AttachedBody *>::iterator it = attachedBodies.begin(); it != attachedBodies.end(); it++)
            {
                // Beware : the visibility cone is an attached object !
                if ((*it)->getName() != "visibility_cone")
                {
                    // ROS_INFO("%s",(*it)->getName().c_str());
                    // Remove attached collisions objects from the ACM (true => no collision checking)
                    acm_.setEntry((*it)->getName(), true);
                    acm_.setDefaultEntry((*it)->getName(), true);

                    // Keep checking collisions with visibility cone if it is not fixed
                    if (!fixed_cone_)
                    {
                        acm_.setEntry("visibility_cone", (*it)->getName(), false);
                    }
                }
            }

            // Handle world collision objects
            std::vector<std::string> collisionObjectWorld = planning_scene_->getWorld()->getObjectIds();

            for (std::vector<std::string>::iterator it = collisionObjectWorld.begin(); it != collisionObjectWorld.end(); it++)
            {
                // ROS_INFO("%s",it->c_str());
                // Remove world collisions objects from the ACM (true => no collision checking)
                acm_.setEntry(*it, true);
                acm_.setDefaultEntry(*it, true);

                // Keep checking collisions with visibility cone if it is not fixed
                if (!fixed_cone_)
                {
                    acm_.setEntry("visibility_cone", *it, false);
                }
            }
            // acm_.print(std::cout);
        }

    public:
        const tf2::Vector3 &getVisibilityAxis() const { return visibility_axis_; }
        const double &getVisibilityDistance() const { return visibility_distance_; }
        const double &getVisibilityRadius() const { return visibility_radius_; }

        void setVisibilityAxis(const tf2::Vector3 &axis) { visibility_axis_ = axis; }
        void setVisibilityDistance(double distance) { visibility_distance_ = distance; }
        void setVisibilityRadius(double radius) { visibility_radius_ = radius; }

        VisibilityGoal(const Robot &robot)
            : PlanningSceneGoal(robot), visibility_axis_(0, 0, 1), visibility_distance_(0.0), visibility_radius_(0.0), fixed_cone_(false)
        {
            secondary_ = true;
            init();
        }

        VisibilityGoal(const Robot &robot, const std::string &link_name, const tf2::Vector3 &visibility_axis, double visibility_distance, double visibility_radius, bool fixed_cone = false, bool secondary = true, double weight = 1.0)
            : PlanningSceneGoal(robot, link_name, weight), visibility_axis_(visibility_axis), visibility_distance_(visibility_distance), visibility_radius_(visibility_radius), fixed_cone_(fixed_cone)
        {
            secondary_ = secondary;
            init();
        }

        double evaluate(const double *variablePositions, bool verbose = false) const
        {
            double fitness = 0.0;
            collision_detection::CollisionResult collisionResult;

            moveit::core::RobotState localRobotState = *robot_state_; // Avoids dirty transforms warning !
            localRobotState.setVariablePositions(variablePositions);
            localRobotState.update();
            
            planning_scene_->checkCollision(collision_request_, collisionResult, localRobotState, acm_);

            // Retrieve the distances to collision for the visibility cone
            for (std::map<const std::pair<std::string, std::string>, std::vector<collision_detection::Contact>>::iterator it = collisionResult.contacts.begin(); it != collisionResult.contacts.end(); it++)
            {
                if (verbose)
                {
                    ROS_INFO("%s %s %f %f", (it->first).first.c_str(), (it->first).second.c_str(), ((it->second)[0].nearest_points[0] - (it->second)[0].nearest_points[1]).norm(), (it->second)[0].depth);
                }

                // Compute the visibility constraint metric
                fitness += visibilityMetric((it->second)[0]);
            }

            if (verbose)
            {
                ROS_INFO("Visibility goal : %f", fitness);
            }

            return fitness;
        }

        virtual double evaluate(const bio_ik::GoalContext &context) const
        {
            // Set robot state according to problem data
            std::vector<double> currentVariablePositions(context.getProblemVariableCount());
            for (size_t i = 0; i < context.getProblemVariableCount(); i++)
            {
                currentVariablePositions[context.getProblemVariableIndex(i)] = context.getProblemVariablePosition(i); // Should be OK
            }

            return (evaluate(&currentVariablePositions[0]));
        }
};

/*! \class CollisionAvoidanceGoal
 * \brief Inverse kinematics resolution goal enforcing a collision avoidance constraint
 */
class CollisionAvoidanceGoal : public PlanningSceneGoal
{
    public:
        CollisionAvoidanceGoal(const Robot &robot) : PlanningSceneGoal(robot) {}
        CollisionAvoidanceGoal(const Robot &robot, const std::string &link_name, double weight = 1.0) : PlanningSceneGoal(robot, link_name, weight) {}

        double evaluate(const double *variablePositions, bool verbose = false) const
        {
            double result = 0.0;
            collision_detection::CollisionResult collisionResult;

            moveit::core::RobotState localRobotState = *robot_state_; // Avoids dirty transforms warning !
            localRobotState.setVariablePositions(variablePositions);
            localRobotState.update();
            
            planning_scene_->checkCollision(collision_request_, collisionResult, localRobotState, acm_);

            result = collisionAvoidanceMetric(collisionResult.collision);

            if (verbose)
            {
                ROS_INFO("Collision goal : %f", result);
            }

            return result;
        }

        virtual double evaluate(const bio_ik::GoalContext &context) const
        {
            // Set robot state according to problem data
            std::vector<double> currentVariablePositions(context.getProblemVariableCount());
            for (size_t i = 0; i < context.getProblemVariableCount(); i++)
            {
                currentVariablePositions[context.getProblemVariableIndex(i)] = context.getProblemVariablePosition(i); // Should be OK
            }

            return (evaluate(&currentVariablePositions[0]));
        }
};
