#include <ros/ros.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_tools_environment_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Get private parameters names -> collision objects and additionnal parameters
    /*
    ros::NodeHandle n("~");
    std::vector<std::string> keys;
    n.getParamNames(keys);

    std::string nodeName = n.getNamespace();
    std::string collisionName;

    for(std::vector<std::string>::iterator it = keys.begin(); it != keys.end(); it++)
    {
        if (it->find(nodeName) == std::string::npos)
        {
            continue;
        }
        else
        {
            collisionName = it->substr(nodeName.size()+1);
            collisionName = collisionName.substr(0,collisionName.find('/'));
        }

        bool collisions;
        if(!n.getParam(collisionName + "/collisions", collisions))
        {
            ROS_INFO("Not a collision object - skipping object");
            continue;
        }
        else
        {
            ROS_INFO("Adding object %s into robot environment...", collisionName.c_str());
        }

        std::string type;
        n.getParam(collisionName + "/type", type);

        if(type == "border")
        {
            std::string axis;
            n.getParam(collisionName+"/axis", axis);
            double offset;
            n.getParam(collisionName+"/offset",offset);

            visualTools.addBorder(collisionName,axis.c_str()[0],offset,!collisions);
        }

        else
        {
            geometry_msgs::Pose collisionPose;
            n.getParam(collisionName+"/pose/x", collisionPose.position.x);
            n.getParam(collisionName+"/pose/y", collisionPose.position.y);
            n.getParam(collisionName+"/pose/z", collisionPose.position.z);

            tf2::Quaternion quaternion;
            double rx,ry,rz;
            n.getParam(collisionName+"/pose/rx", rx);
            n.getParam(collisionName+"/pose/ry", ry);
            n.getParam(collisionName+"/pose/rz", rz);
            quaternion.setRPY(rx,ry,rz);
            collisionPose.orientation = tf2::toMsg(quaternion);

            bool robotBaseCollisions;
            n.getParam(collisionName+"/robot_base_collisions", robotBaseCollisions);

            if(type == "sphere")
            {
                double radius;
                n.getParam(collisionName+"/size/radius",radius);
                visualTools.addSphere(collisionName,collisionPose,radius,!collisions);
            }
            else if(type == "cylinder")
            {
                double radius, height;
                n.getParam(collisionName+"/size/radius",radius);
                n.getParam(collisionName+"/size/height",height);
                visualTools.addCylinder(collisionName,collisionPose,radius,height,!collisions,!robotBaseCollisions);
            }
            else if(type == "box")
            {
                double dx,dy,dz;
                n.getParam(collisionName+"/size/dx",dx);
                n.getParam(collisionName+"/size/dy",dy);
                n.getParam(collisionName+"/size/dz",dz);
                visualTools.addBox(collisionName,collisionPose,dx,dy,dz,!collisions,!robotBaseCollisions);
            }
            else
            {
                ROS_INFO("Unknown object type - skipping object");
            }
        }
    }
    */

    ros::NodeHandle nGlobal;
    ros::NodeHandle nPrivate("~");
    std::string environmentFileName;
    if(!nPrivate.getParam("environmentFileName",environmentFileName))
    {
        ROS_ERROR("Unable to retrieve environment file name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(environmentFileName);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Unable to open environment configuration file !");
        throw std::runtime_error("INVALID CONFIGURATION FILE");
    }

    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it) 
    {
        bool collisions;
        try
        {
            collisions = config[it->first.as<std::string>()]["collisions"].as<bool>();
            ROS_INFO("Adding object %s into robot environment...", it->first.as<std::string>().c_str());
        }
        catch(const std::exception& e)
        {
            //Special case : objectPose and objectSize => Loaded as global parameters
            if(it->first.as<std::string>() == "objectPose")
            {
                XmlRpc::XmlRpcValue objectPose;
                objectPose.setSize(6);

                //TODO Go back to dictionnary like pose description to avoid understanding/parsing issures
                /*objectPose[0] = config["objectPose"]["pose"]["x"].as<double>();
                objectPose[1] = config["objectPose"]["pose"]["y"].as<double>();
                objectPose[2] = config["objectPose"]["pose"]["z"].as<double>();
                objectPose[3] = config["objectPose"]["pose"]["rx"].as<double>();
                objectPose[4] = config["objectPose"]["pose"]["ry"].as<double>();
                objectPose[5] = config["objectPose"]["pose"]["rz"].as<double>();*/

                for (int i = 0; i < config["objectPose"].size(); i++) 
                {
                    objectPose[i] = config["objectPose"][i].as<double>();
                }
                nGlobal.setParam("objectPose",objectPose);
            }
            else if(it->first.as<std::string>() == "objectSize")
            {
                nGlobal.setParam("objectSize",config["objectSize"].as<double>());
            }

            continue;
        }

        if(config[it->first.as<std::string>()]["type"].as<std::string>() == "border")
        {
            visualTools.addBorder(it->first.as<std::string>(),config[it->first.as<std::string>()]["axis"].as<char>(),config[it->first.as<std::string>()]["offset"].as<double>(),!collisions);
        }

        else
        {
            geometry_msgs::Pose collisionPose;
            collisionPose.position.x = config[it->first.as<std::string>()]["pose"]["x"].as<double>();
            collisionPose.position.y = config[it->first.as<std::string>()]["pose"]["y"].as<double>();
            collisionPose.position.z = config[it->first.as<std::string>()]["pose"]["z"].as<double>();

            tf2::Quaternion quaternion;
            quaternion.setRPY(config[it->first.as<std::string>()]["pose"]["rx"].as<double>(),config[it->first.as<std::string>()]["pose"]["ry"].as<double>(),config[it->first.as<std::string>()]["pose"]["rz"].as<double>());
            collisionPose.orientation = tf2::toMsg(quaternion);

            bool robotBaseCollisions = config[it->first.as<std::string>()]["robot_base_collisions"].as<bool>();

            if(config[it->first.as<std::string>()]["type"].as<std::string>() == "sphere")
            {
                visualTools.addSphere(it->first.as<std::string>(),collisionPose,config[it->first.as<std::string>()]["size"]["radius"].as<double>(),!collisions);
            }
            else if(config[it->first.as<std::string>()]["type"].as<std::string>() == "cylinder")
            {
                visualTools.addCylinder(it->first.as<std::string>(),collisionPose,config[it->first.as<std::string>()]["size"]["radius"].as<double>(),config[it->first.as<std::string>()]["size"]["height"].as<double>(),!collisions,!robotBaseCollisions);
            }
            else if(config[it->first.as<std::string>()]["type"].as<std::string>() == "box")
            {
                visualTools.addBox(it->first.as<std::string>(),collisionPose,config[it->first.as<std::string>()]["size"]["dx"].as<double>(),config[it->first.as<std::string>()]["size"]["dy"].as<double>(),config[it->first.as<std::string>()]["size"]["dz"].as<double>(),!collisions,!robotBaseCollisions);
            }
            else if(config[it->first.as<std::string>()]["type"].as<std::string>() == "mesh")
            {
                visualTools.addMesh(it->first.as<std::string>(),config[it->first.as<std::string>()]["file_name"].as<std::string>(),collisionPose,!collisions,!robotBaseCollisions);
            }
            else
            {
                ROS_INFO("Unknown object type - skipping object");
            }
        }
    }

    ros::shutdown();
    return 0;
}

