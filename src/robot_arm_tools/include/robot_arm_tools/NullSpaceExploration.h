/**
 * \file NullSpaceExploration.h
 * \brief Header file of the NullSpaceExploration abstract class
 *
 * Header file of the NullSpaceExploration abstract class - Defines a generic interface for nullspace exploration around a given pose to find optimal configurations (basically, the fitness method is the only purely virtual method to implement in derived classes)
 *
 */

#pragma once

#include "robot_arm_tools/Robot.h"

/*! \class NullSpaceExploration
* \brief Generic interface for nullspace exploration around a given pose to find optimal configurations
*/
class NullSpaceExploration
{
    public:

        NullSpaceExploration(const Robot& robot, const Eigen::VectorXd& initialConfiguration, bool constrainedYaw = false);

        Eigen::VectorXd findBestLocalConfiguration();

        /*!
         *  \brief Fitness method, *pure virtual* method to implement in derived classes
         *  \param currentConfiguration The current configuration of the robot
         *  \return The fitness of the current configuration
         */
        virtual double fitness(const Eigen::VectorXd& currentConfiguration) = 0;

    protected:

        std::string m_endEffectorName; /*!< End effector name */
        Eigen::Isometry3d m_referencePose;
        Eigen::Isometry3d m_targetPose;

        bool m_constrainedYaw;  /*!< Constrained yaw option */

        std::shared_ptr<const moveit::core::JointModelGroup> m_jointModelGroup; /*!< Joint model group */
        std::shared_ptr<moveit::core::RobotState> m_robotState; /*!< Robot state */

        Eigen::VectorXd m_initialConfiguration; /*!< Initial configuration */

        Eigen::MatrixXd m_nullSpace; /*!< Current null space for continuity ! */

        static Eigen::Index findMatching(const Eigen::VectorXd &key, const Eigen::MatrixXd &haystack, const Eigen::VectorXi &available, double &sign);

        Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &currentConfiguration, bool quaternionRepresentation = false);

        void updateNullSpace(const Eigen::VectorXd &currentConfiguration);

};

class VisibilityNullSpaceExploration : public NullSpaceExploration
{
    public:

        VisibilityNullSpaceExploration(const Robot& robot, const Eigen::VectorXd& initialConfiguration, bool constrainedYaw = false, const Eigen::Vector3d& visibilityAxis = Eigen::Vector3d(0.0,0.0,1.0), double visibilityDistance = 0.5, double visibilityRadius = 0.1);

        /*!
         *  \brief Fitness method : collisions and distance to the visibility cone
         *  \param currentConfiguration The current configuration of the robot
         *  \return The fitness of the current configuration
         */
        double fitness(const Eigen::VectorXd& currentConfiguration);

    private:

        double m_visibilityDistance; /*!< Visibility distance */
        double m_visibilityRadius; /*!< Visibility radius */
        Eigen::Vector3d m_visibilityAxis; /*!< Visibility axis */

        std::shared_ptr<planning_scene::PlanningScene> m_planningScene; /*!< Planning scene */
        collision_detection::AllowedCollisionMatrix m_allowedCollisionMatrixDistance, m_allowedCollisionMatrixCollision; /*!< Allowed collision matrix */

        collision_detection::CollisionRequest m_collisionRequestDistance;
        collision_detection::CollisionRequest m_collisionRequestCollision;   
};