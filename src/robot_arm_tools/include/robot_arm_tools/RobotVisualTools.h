/**
 * \file RobotVisualTools.h
 * \brief Header file of the RobotVisualTools class.
 *
 * Header file of the RobotVisualTools class.
 * Defines the methods used to create and delete several types of collision and non-collision objectis within a RViz/Moveit! environment.
 *
 */

#pragma once

#include <map>
#include <string>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/*! \class RobotVisualTools
 * \brief Defines the methods used to create and delete several types of collision and non-collision objects within a RViz/Moveit! environment.
 */
class RobotVisualTools : public moveit_visual_tools::MoveItVisualTools
{
    public:
        /*!
         * \brief Constructor.
         */
        RobotVisualTools();

        /*!
         * \brief Destructor.
         */
        ~RobotVisualTools(){};

        /*!
         * \brief Adds a border plane to the robot environment.
         * \param borderName The ID name to give to the border.
         * \param borderAxis This axis along which the border is to be placed.
         * \param borderPosition The position of the border along borderAxis.
         * \param ignoreRobotBaseCollision Wether or not to ignore collisions between the border and the robot base.
         */
        void addBorder(std::string borderName, char borderAxis, double borderPosition, bool ignoreRobotBaseCollision = false);

        /*!
         * \brief Adds a sphere to the robot environment.
         * \param sphereName The ID name to give to the sphere.
         * \param spherePose The position of the center of the sphere.
         * \param sphereRadius The radius of the sphere.
         * \param ignoreCollisions Wether or not to ignore collisions between the sphere and the robot.
         */
        void addSphere(std::string sphereName, geometry_msgs::Pose spherePose, double sphereRadius, bool ignoreCollisions = false);

        /*!
         * \brief Adds a cylinder to the robot environment.
         * \param cylinderName The ID name to give to the cylinder.
         * \param cylinderPose The position of the center of the cylinder.
         * \param cylinderRadius The radius of the cylinder.
         * \param cylinderHeight The height of the cylinder.
         * \param ignoreCollisions Wether or not to ignore collisions between the cylinder and the robot.
         * \param ignoreRobotBaseCollision Wether or not to ignore collisions between the box and the robot base. 
         */
        void addCylinder(std::string cylinderName, geometry_msgs::Pose cylinderPose, double cylinderRadius, double cylinderHeight, bool ignoreCollisions = false, bool ignoreRobotBaseCollision = false);

        /*!
         * \brief Adds a box to the robot environment.
         * \param boxName The ID name to give to the box.
         * \param boxPose The position of the center of the box.
         * \param boxDx The dimension of the box along the x axis.
         * \param boxDy The dimension of the box along the y axis.
         * \param boxDz The dimension of the box along the z axis.
         * \param ignoreCollisions Wether or not to ignore collisions between the box and the robot.
         * \param ignoreRobotBaseCollision Wether or not to ignore collisions between the box and the robot base. 
         */
        void addBox(std::string boxName, geometry_msgs::Pose boxPose, double boxDx, double boxDy, double boxDz, bool ignoreCollisions = false, bool ignoreRobotBaseCollision = false);

        /*!
         * \brief Displays a frame (pose) in the robot environment.
         * \param frameName The ID name to give to the frame.
         * \param framePose The position and orientation of the frame.
         */
        void addFrame(std::string frameName, geometry_msgs::Pose framePose);

        /*!
         * \brief Displays a mesh in the robot environment.
         * \param meshName The ID name to give to the mesh.
         * \param meshFileName The mesh file name (.STL).
         * \param framePose The position and orientation of the mesh.
         * \param ignoreCollisions Wether or not to ignore collisions between the box and the robot.
         * \param ignoreRobotBaseCollision Wether or not to ignore collisions between the box and the robot base.
         */
        void addMesh(std::string meshName, std::string meshFileName, geometry_msgs::Pose meshPose, bool ignoreCollisions, bool ignoreRobotBaseCollision);

        /*!
         * \brief Delete a given object based on its ID name.
         * \param objectName The ID name of the deleted object.
         */
        void deleteObject(std::string objectName);

        /*!
         * \brief Retrive a given object based on its ID name.
         * \param collisionObjectName The ID name of the collision object.
         * \return The requested collision object.
         */
        moveit_msgs::CollisionObject getCollisionObject(std::string collisionObjectName);
        
    private:

        /*!
         * \brief Ignores the collisions between a collision object and the robot base links.
         * \param collisionObject The concerned collision object.
         * \return The corresponding attached collision object.
         */
        moveit_msgs::AttachedCollisionObject ignoreRobotBase(const moveit_msgs::CollisionObject &collisionObject);

        std::vector<std::string> m_collisionBaseLinksNames; /*!< Robot base links to be eventually ignored collision wise */

        moveit::planning_interface::PlanningSceneInterface m_planningSceneInterface;   /*!< Moveit! planning scene interface */

        std::map<std::string, std::vector<int>> m_IDs;  /*!< Map mapping objects ID names with ID numbers */

        static int m_ObjectID; /*!< Current object number ID */

        ros::NodeHandle m_nodeHandle;    /*!< ROS node handle*/
};
