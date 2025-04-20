#include "robot_arm_tools/OKMeasurementServer.h"

bool OKMeasurementServer::measure()
{
    do 
    {
        ROS_INFO("Press enter to continue !");
    } while (std::cin.get() != '\n');

    return(true);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "ok_server");  

    OKMeasurementServer OKMeasurementServer;

	ros::spin();
    return 0;
}