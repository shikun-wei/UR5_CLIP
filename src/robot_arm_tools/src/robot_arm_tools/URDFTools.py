#!/usr/bin/python3

#System packages
import subprocess
import re

#Utility packages
import numpy as np
from urchin import URDF

#ROS package
import rospkg
rospack = rospkg.RosPack()

def size(L):
    """! Computes the size of a nested list as the sum of the sizes of all contained lists

    @param L   The nested list.

    @return  The size of the nested list.
    """
    if type(L) == list or type(L) == np.ndarray:
        return(sum(size(l) for l in L))
    else:
        return(1)

def cleanXACROOutput(XACROOutput):
    """! Cleans the raw XACRO output by parsing unrecognized "package://" and "file://" tags.

    @param XACROOutput The raw XACRO output.

    @return  The cleaned XACRO output.
    """

    packages = np.unique(np.array([XACROOutput[int(m.start()):int(m.end())] for m in re.finditer('package://[a-zA-Z_]+/', XACROOutput)]))

    for package in packages:
        XACROOutput = XACROOutput.replace(package,rospack.get_path(package.split("/")[-2]) + "/")

    XACROOutput = XACROOutput.replace("file://","")
    return(XACROOutput)


def getURDF(URDFFilePath, **kwargs):
    """! Extract the URDF model contained in an URDF or a XACRO file.

    @param URDFFilePath The path to the URDF/XACRO file.
    @param **kwargs     The eventual arguments to pass to the xacro command while parsing the XACRO file.

    @return  The URDF model extracted using urchin.
    """

    #XACRO file
    if(URDFFilePath.split(".")[-1] == "xacro"):

        #Execute XACRO command and get 
        xacro = "echo $(xacro " + URDFFilePath
        for key, value in kwargs.items():
            xacro += " " + key + ":=" + str(value)
        xacro += ")" 
        xacroProcess = subprocess.run(xacro,shell=True,stdout=subprocess.PIPE)
        XACROOutput = cleanXACROOutput(str(xacroProcess.stdout)[2:-3])

        with open("/tmp/tmp.urdf", "w") as f:
            f.write(XACROOutput)

        ### Modify URDF file
        robot = URDF.load('/tmp/tmp.urdf',True)

    #URDF file
    else:
        robot = URDF.load(URDFFilePath,True)

    return(robot)

def getDistanceToBaseLink(robot, link):
    """! Computes the distance (as number of "real" joints) between a given link and the robot base link

    @param robot    The urchin robot URDF model.
    @param link     The considered link.

    @return  The distance.
    """

    linksNames = np.array([link.name for link in robot.links])
    jointsChildNames = np.array([joint.child for joint in robot.joints])
    tmpLink = link
    distance = 0
    while(tmpLink != robot.base_link):
        #Update current joint
        tmpJoint = robot.joints[np.where(jointsChildNames == tmpLink.name)[0][0]]
        #Update current link
        newLink = robot.links[np.where(linksNames == tmpJoint.parent)[0][0]]

        #Remove "fake" joints (e.g. for coarse self collisions "sc")
        if(newLink.name not in tmpLink.name):
            distance+=1
        tmpLink = newLink
    return(distance)

def getSortedJoints(robot,flangeLinkName):
    """! Stores and sorts the joints linking the robot flange to the robot base.

    @param robot            The urchin robot URDF model.
    @param flangeLinkName   The name of the robot flange link.

    @return  The robot base joints.
    @return  The robot actuated joints.
    @return  The robot flange joints.
    """

    linksNames = np.array([link.name for link in robot.links])
    
    #Find a correct flange link name to proceed
    if(flangeLinkName not in linksNames or flangeLinkName == ""):
        flangeLinkName = robot.end_links[np.argmax([getDistanceToBaseLink(robot,link) for link in robot.end_links])].name    
        print("Invalid flange link name, defaulting to : " + flangeLinkName)

    flangeLink = robot.links[np.where(linksNames == flangeLinkName)[0][0]]

    baseJoints = []
    actuatedJoints = []
    flangeJoints = []

    #Start from the flange link, go back to the base link and sort URDF joints
    tmpLink = flangeLink
    jointsChildNames = np.array([joint.child for joint in robot.joints])

    while(tmpLink != robot.base_link):
        #Update current joint
        tmpJoint = robot.joints[np.where(jointsChildNames == tmpLink.name)[0][0]]

        #Investigating not actuated intermediate joints : 
        if(tmpJoint not in robot.actuated_joints):
            #Conventionnal base joints (e.g. "base_inertia")
            if(len(actuatedJoints) != 0):
                baseJoints.append(tmpJoint)
            #Conventionnal flange joints (e.g. "flange")
            else:
                flangeJoints.append(tmpJoint)
        else:
            actuatedJoints.append(tmpJoint)

        #Update current link
        tmpLink = robot.links[np.where(linksNames == tmpJoint.parent)[0][0]]

    #Put everything in the correct order and return
    return(baseJoints[::-1],actuatedJoints[::-1],flangeJoints[::-1])
