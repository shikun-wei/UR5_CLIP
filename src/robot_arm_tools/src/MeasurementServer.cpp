#include <fstream>
#include <experimental/filesystem>

#include "robot_arm_tools/MeasurementServer.h"

#define MAX_RECOVERY_ATTEMPTS 1

//TODO Private parameters

MeasurementServer::MeasurementServer(std::string measurementServerName, bool measurementServerDisplay, std::string measurementServerStorageFolder) : m_measurementServerName(measurementServerName), m_measurementServerDisplay(measurementServerDisplay), m_measurementServerStorageFolder(measurementServerStorageFolder), m_measurementServerCounter(0)
{
    //Handle ROS service measurements storage folder - clean or create
    std::experimental::filesystem::path folderPath{m_measurementServerStorageFolder};
    if(std::experimental::filesystem::exists(folderPath))
    {
        ROS_WARN("%s already exists : its contents will be overwritten !", m_measurementServerStorageFolder.c_str());
    }
    else if(m_measurementServerStorageFolder != "")
    {
        ROS_INFO("Creating %s ...", m_measurementServerStorageFolder.c_str());
        std::experimental::filesystem::create_directories(folderPath.parent_path());
    }

    m_measurementServer = m_nodeHandle.advertiseService(m_measurementServerName,&MeasurementServer::m_measure,this);
    m_measurementCounterServer = m_nodeHandle.advertiseService(m_measurementServerName + "_counter",&MeasurementServer::m_updateCounter,this);
}

MeasurementServer::MeasurementServer() : m_measurementServerCounter(0)
{
    if(!m_nodeHandle.getParam("measurementServerName",m_measurementServerName))
    {
        ROS_ERROR("Unable to retrieve measurement server name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //TODO Link to DEBUG ?
    if(!m_nodeHandle.getParam("measurementServerDisplay",m_measurementServerDisplay))
    {
        ROS_WARN("No measurement server display specified, switching to no display");
        m_measurementServerDisplay = false;
    }

    if(!m_nodeHandle.getParam("measurementServerStorageFolder",m_measurementServerStorageFolder) || m_measurementServerStorageFolder == "")
    {
        ROS_WARN("No measurement server storage folder specified, switching to /tmp/Measurements/");
        m_measurementServerStorageFolder = "/tmp/Measurements/";
    }

    //Handle ROS service measurements storage folder - clean or create
    std::experimental::filesystem::path folderPath{m_measurementServerStorageFolder};
    if(std::experimental::filesystem::exists(folderPath))
    {
        ROS_WARN("%s already exists : its contents will be overwritten !", m_measurementServerStorageFolder.c_str());
        //std::experimental::filesystem::remove_all(folderPath);
    }
    else if(m_measurementServerStorageFolder != "")
    {
        ROS_INFO("Creating %s ...", m_measurementServerStorageFolder.c_str());   
        std::experimental::filesystem::create_directories(folderPath);     
    }
    
    m_measurementServer = m_nodeHandle.advertiseService(m_measurementServerName,&MeasurementServer::m_measure,this);
    m_measurementCounterServer = m_nodeHandle.advertiseService(m_measurementServerName + "_counter",&MeasurementServer::m_updateCounter,this);
}

bool MeasurementServer::m_measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    for(int i = 0; i < MAX_RECOVERY_ATTEMPTS; i++)
    {
        if(measure())
        {
            m_measurementServerCounter++;
            return(true);
        }
        else
        {
            //Trying to recover in case of failure...
            if(! recovery())
            {
                return(false);
            }
        }
    }
    return(false);
}

bool MeasurementServer::m_updateCounter(robot_arm_tools::Int::Request & req, robot_arm_tools::Int::Response &res)
{
    m_measurementServerCounter = req.data;
    return(true);
}
