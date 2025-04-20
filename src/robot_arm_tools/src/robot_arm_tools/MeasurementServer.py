#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty,EmptyResponse
from robot_arm_tools.srv import Int,IntResponse

import os
from abc import abstractmethod

MAX_RECOVERY_ATTEMPTS = 1

#TODO Private parameters
class MeasurementServer :
    
    def __init__(self, measurementServerName = None, measurementServerDisplay = None, measurementServerStorageFolder = None):
        
        #Retrieving parameters 
        if(measurementServerName is None):
            try:
                self.measurementServerName = rospy.get_param("measurementServerName")
            except KeyError:
                rospy.logerr("Unable to retrieve measurement server name !")
                raise NameError("MISSING PARAMETER")
        else:
            self.measurementServerName = measurementServerName
        
        if(measurementServerDisplay is None):
            try:
                self.measurementServerDisplay = rospy.get_param("measurementServerDisplay")
            except KeyError:
                rospy.logwarn("No measurement server display specified, switching to no display")
                self.measurementServerDisplay = False
        else:
            self.measurementServerDisplay = measurementServerDisplay

        if(measurementServerStorageFolder is None):
            try:
                self.measurementServerStorageFolder = rospy.get_param("measurementServerStorageFolder")
            except KeyError:
                rospy.logwarn("No measurement server storage folder specified, switching to /tmp/Measurements/")
                self.measurementServerStorageFolder = "/tmp/Measurements/"
        else:
            self.measurementServerStorageFolder = measurementServerStorageFolder 

        #Handle measurements folder creation
        try:
            os.makedirs(self.measurementServerStorageFolder)
            rospy.loginfo("Creating " + self.measurementServerStorageFolder + " ...")
        except OSError:
            rospy.logwarn(self.measurementServerStorageFolder + "already exists : its contents will be overwritten !") 
              
        self.measurementServerCounter = 0
        
        ### ROS Service Server
        self.measurementServer = rospy.Service(self.measurementServerName, Empty, self.m_measure)
        self.measurementCounterServer = rospy.Service(self.measurementServerName + "_counter", Int , self.m_updateCounter)

    def m_measure(self, _):

        for _ in range(MAX_RECOVERY_ATTEMPTS):

            if(self.measure()):
                self.measurementServerCounter += 1
                return EmptyResponse()

            else:
                if(not self.recovery()):
                    return(None)
                    
        return(None)

    def m_updateCounter(self, req):
        self.measurementServerCounter = req.data
        rospy.loginfo("Measurement server counter set to " + str(self.measurementServerCounter))
        return IntResponse()

    @abstractmethod
    def measure(self):
        return(True)

    @abstractmethod
    def recovery(self):
        return(True)
    