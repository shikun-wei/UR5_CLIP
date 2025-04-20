#include "robot_arm_tools/RobotVisualTools.h"

#include <moveit_msgs/PlanningScene.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

int RobotVisualTools::m_ObjectID = 0;

RobotVisualTools::RobotVisualTools() : moveit_visual_tools::MoveItVisualTools()
{
    if(!m_nodeHandle.getParam("collisionBaseLinksNames", m_collisionBaseLinksNames))
    {
        if(base_frame_ == "world")
        {
            ROS_WARN("No robot base links defined for collision.");
            //TODO take robot first two links ?
            m_collisionBaseLinksNames = {};
        }
        else
        {
            ROS_WARN("No robot base links defined for collision, switching to %s.", base_frame_.c_str());
            m_collisionBaseLinksNames = {base_frame_};
        }    
    }
}

void RobotVisualTools::addBorder(std::string borderName, char borderAxis, double borderPosition, bool ignoreRobotBaseCollision)
{
    geometry_msgs::Pose borderPose;
    borderPose.orientation.w = 1;

    if(borderAxis == 'x')
    {
        borderPose.position.x = borderPosition + ((borderPosition > 0) - (borderPosition < 0))*0.0005;
        addBox(borderName, borderPose, 0.001, 5.0, 5.0, false, ignoreRobotBaseCollision);
    }
    else if(borderAxis == 'y')
    {
        borderPose.position.y = borderPosition + ((borderPosition > 0) - (borderPosition < 0))*0.0005;
        addBox(borderName, borderPose, 5.0, 0.001, 5.0, false, ignoreRobotBaseCollision);
    }
    else if(borderAxis == 'z')
    {
        borderPose.position.z = borderPosition + ((borderPosition > 0) - (borderPosition < 0))*0.0005;
        addBox(borderName, borderPose, 5.0, 5.0, 0.001, false, ignoreRobotBaseCollision);
    }
    else
    {
        ROS_ERROR("Invalid value for axis !");
        throw std::invalid_argument("INVALID AXIS - x/y/z");
    }
}

void RobotVisualTools::addSphere(std::string sphereName, geometry_msgs::Pose spherePose, double sphereRadius, bool ignoreCollisions)
{
    if(!ignoreCollisions)
    {
        //Create collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.header.frame_id = base_frame_;
        collisionObject.id = sphereName;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = sphereRadius;

        collisionObject.primitives.push_back(primitive);
        collisionObject.primitive_poses.push_back(spherePose);
        collisionObject.operation = collisionObject.ADD;

        //Add collision object to the planning scene and robot environment
        m_planningSceneInterface.applyCollisionObject(collisionObject);

        //Update ID name/ID number map - Collision object do not have ID numbers
        m_IDs[sphereName] = {};
    }

    else
    {
        sphere_marker_.id = m_ObjectID;
        m_ObjectID++;

        //Add non-collision object to the robot environment
        moveit_visual_tools::MoveItVisualTools::publishSphere(spherePose,rviz_visual_tools::BLUE,2*sphereRadius);
        moveit_visual_tools::MoveItVisualTools::trigger();

        //Update ID name/ID number map - Non-collision object must have ID numbers to be later re-identified
        m_IDs[sphereName] = {sphere_marker_.id};
    } 
}

void RobotVisualTools::addCylinder(std::string cylinderName, geometry_msgs::Pose cylinderPose, double cylinderRadius, double cylinderHeight, bool ignoreCollisions, bool ignoreRobotBaseCollision)
{
    if(!ignoreCollisions)
    {
        //Create collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.header.frame_id = base_frame_;
        collisionObject.id = cylinderName;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = cylinderHeight;
        primitive.dimensions[1] = cylinderRadius;

        collisionObject.primitives.push_back(primitive);
        collisionObject.primitive_poses.push_back(cylinderPose);
        collisionObject.operation = collisionObject.ADD;

        //Ignore collisions with robot base
        if(ignoreRobotBaseCollision && m_collisionBaseLinksNames.size() > 0)
        {
            moveit_msgs::AttachedCollisionObject attachedCollisionObject = ignoreRobotBase(collisionObject);
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyAttachedCollisionObject(attachedCollisionObject);
        }

        else
        {
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyCollisionObject(collisionObject);
        }

        //Update ID name/ID number map - Collision object do not have ID numbers
        m_IDs[cylinderName] = {};
    }

    else
    {
        cylinder_marker_.id = m_ObjectID;
        m_ObjectID++;

        //Add non-collision object to the robot environment
        moveit_visual_tools::MoveItVisualTools::publishCylinder(cylinderPose,rviz_visual_tools::BLUE,cylinderHeight,cylinderRadius);
        moveit_visual_tools::MoveItVisualTools::trigger();

        //Update ID name/ID number map - Non-collision object must have ID numbers to be later re-identified
        m_IDs[cylinderName] = {cylinder_marker_.id};
    } 
}

void RobotVisualTools::addBox(std::string boxName, geometry_msgs::Pose boxPose, double boxDx, double boxDy, double boxDz, bool ignoreCollisions, bool ignoreRobotBaseCollision)
{
    if(!ignoreCollisions)
    {
        //Create collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.header.frame_id = base_frame_;
        collisionObject.id = boxName;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = boxDx;
        primitive.dimensions[1] = boxDy;
        primitive.dimensions[2] = boxDz;

        collisionObject.primitives.push_back(primitive);
        collisionObject.primitive_poses.push_back(boxPose);
        collisionObject.operation = collisionObject.ADD;

        //Ignore collisions with robot base
        if(ignoreRobotBaseCollision  && m_collisionBaseLinksNames.size() > 0)
        {
            moveit_msgs::AttachedCollisionObject attachedCollisionObject = ignoreRobotBase(collisionObject);
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyAttachedCollisionObject(attachedCollisionObject);
        }

        else
        {
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyCollisionObject(collisionObject);
        }

        //Update ID name/ID number map - Collision object do not have ID numbers
        m_IDs[boxName] = {}; 
    }

    else
    {
        cuboid_marker_.id = m_ObjectID;
        m_ObjectID++;

        //Add non-collision object to the robot environment
        moveit_visual_tools::MoveItVisualTools::publishCuboid(boxPose,boxDy,boxDx,boxDz,rviz_visual_tools::BLUE);
        moveit_visual_tools::MoveItVisualTools::trigger();

        //Update ID name/ID number map - Non-collision object must have ID numbers to be later re-identified
        m_IDs[boxName] = {cuboid_marker_.id};
    } 
}

void RobotVisualTools::addFrame(std::string frameName, geometry_msgs::Pose framePose)
{
    cylinder_marker_.id = m_ObjectID;
    m_ObjectID += 3;

    //Add non-collision object to the robot environment
    moveit_visual_tools::MoveItVisualTools::publishAxis(framePose, rviz_visual_tools::MEDIUM, std::string("RobotVisualTools"));
    moveit_visual_tools::MoveItVisualTools::trigger();

    //Update ID name/ID number map for the three cylinders - Non-collision object must have ID numbers to be later re-identified
    m_IDs[frameName] = {cylinder_marker_.id, cylinder_marker_.id - 1, cylinder_marker_.id - 2};
}

void RobotVisualTools::addMesh(std::string meshName, std::string meshFileName, geometry_msgs::Pose meshPose, bool ignoreCollisions, bool ignoreRobotBaseCollision)
{
    if(!ignoreCollisions)
    {
        //Create collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.header.frame_id = base_frame_;
        collisionObject.id = meshName;

        shapes::ShapeMsg meshMsg;
        shapes::constructMsgFromShape(shapes::createMeshFromResource(meshFileName), meshMsg);

        collisionObject.meshes.push_back(boost::get<shape_msgs::Mesh>(meshMsg));
        collisionObject.mesh_poses.push_back(meshPose);
        collisionObject.operation = collisionObject.ADD;

        //Ignore collisions with robot base
        if(ignoreRobotBaseCollision  && m_collisionBaseLinksNames.size() > 0)
        {
            moveit_msgs::AttachedCollisionObject attachedCollisionObject = ignoreRobotBase(collisionObject);
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyAttachedCollisionObject(attachedCollisionObject);
        }

        else
        {
            //Add collision object to the planning scene and robot environment
            m_planningSceneInterface.applyCollisionObject(collisionObject);
        }

        //Update ID name/ID number map - Collision object do not have ID numbers
        m_IDs[meshName] = {}; 

    }

    else
    {
        mesh_marker_.id = m_ObjectID;
        m_ObjectID++;

        //Add non-collision object to the robot environment
        moveit_visual_tools::MoveItVisualTools::publishMesh(meshPose, meshFileName, rviz_visual_tools::BLUE);
        moveit_visual_tools::MoveItVisualTools::trigger();

        //Update ID name/ID number map - Non-collision object must have ID numbers to be later re-identified
        m_IDs[meshName] = {mesh_marker_.id};
    }
}

void RobotVisualTools::deleteObject(std::string objectName)
{
    //Delete a non-collision object
    if(!m_IDs[objectName].empty())
    {
        //Create the delete marker
        visualization_msgs::Marker deleteMarker;
        deleteMarker.header.frame_id = base_frame_;
        deleteMarker.header.stamp = ros::Time();
        deleteMarker.ns = "RobotVisualTools";
        deleteMarker.action = visualization_msgs::Marker::DELETE;
        deleteMarker.pose.orientation.w = 1;

        for(std::vector<int>::iterator it = m_IDs[objectName].begin(); it != m_IDs[objectName].end(); it++)
        {
            deleteMarker.id = *it;
            moveit_visual_tools::MoveItVisualTools::publishMarker(deleteMarker);
        }  
    }

    //Delete a collision object
    else
    {
        moveit_msgs::CollisionObject removeObject;
        removeObject.id = objectName;
        removeObject.header.frame_id = base_frame_;
        removeObject.operation = removeObject.REMOVE;
        m_planningSceneInterface.applyCollisionObject(removeObject);
    }
}

moveit_msgs::AttachedCollisionObject RobotVisualTools::ignoreRobotBase(const moveit_msgs::CollisionObject &collisionObject)
{
    moveit_msgs::AttachedCollisionObject attachedCollisionObject;

    attachedCollisionObject.link_name = m_collisionBaseLinksNames[0];
    attachedCollisionObject.touch_links = m_collisionBaseLinksNames;
    attachedCollisionObject.object = collisionObject;

    return(attachedCollisionObject);
}

moveit_msgs::CollisionObject RobotVisualTools::getCollisionObject(std::string collisionObjectName)
{
    return(m_planningSceneInterface.getObjects(std::vector<std::string> {collisionObjectName})[collisionObjectName]);   
}