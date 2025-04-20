#! /usr/bin/python3

import sys
from robot_arm_tools.URDFTools import cleanXACROOutput

robot_description = sys.argv[1]
tool_description = cleanXACROOutput(sys.argv[2])
robot_description_semantic = sys.argv[3]
end_effector_link = sys.argv[4]

try:
    calibrated_model = sys.argv[5]
    flange_link_name = sys.argv[6]
except:
    calibrated_model = ""
    flange_link_name = ""

import xml.etree.ElementTree as ET
robot_semantic_tree = ET.fromstring(robot_description_semantic)

if(tool_description != ""):
    robot_tree = ET.fromstring(robot_description)
    tool_tree = ET.fromstring(tool_description)

    all_links = [link for link in robot_tree.iter("link")]
    all_joints = [joint for joint in robot_tree.iter("joint")]
    links_pending = [link for link in all_links if link.attrib["name"] == flange_link_name]

    counter = 0
    max_robot_links_collisions = 2
    robot_links_names_no_collisions = []    #List of ignored collisions links

    #Multiple flanges => Impossible in our case
    if(len(links_pending) != 1):
        sys.exit("Error: Multiple flange links found")

    def handle_tip(link):
        """Function handling collisions fo all links and joints starting from a given tip link recursively"""

        #Get all link child joints
        child_joints = [joint for joint in all_joints if any(parent_link.attrib["link"].strip() == link.attrib["name"].strip() for parent_link in joint.iter("parent"))]

        for joint in child_joints:

            #Separate actuated chain starting at link : may collide !
            if(joint.attrib["type"] == "revolute" or joint.attrib["type"] == "prismatic"):
                #Remove joint so that the actuated chain is not handled again
                all_joints.remove(joint)

            #Separare passive chain starting at link  : will not collide
            else:
                #Get joint child links
                child_links = [link for link in all_links if any(child_link.attrib["link"].strip() == link.attrib["name"].strip() for child_link in joint.iter("child"))]

                #Add child links to ingored collisions if they have a collision tag
                robot_links_names_no_collisions.extend([link.attrib["name"].strip() for link in child_links if len(list(link.iter("collision"))) != 0])

                #Add child links to pending links
                for link in child_links:
                    handle_tip(link)
                #Remove joint so that the passive chain is not handled again
                all_joints.remove(joint)

    while(len(links_pending) != 0 and counter < max_robot_links_collisions):
        #Get next pending link
        link = links_pending.pop()

        #counter < max_robot_links_collisions : we want to ignore collisions for the first 2 links group/passive chain, which are the closest to the flange and tool, so we add the link to the ignored collisions list
        if(len(list(link.iter("collision"))) != 0):
            robot_links_names_no_collisions.append(link.attrib["name"])

        #Handle eventual tip links and joints
        handle_tip(link)

        #Get link parent joints
        parent_joints = [joint for joint in all_joints if any(child_link.attrib["link"].strip() == link.attrib["name"].strip() for child_link in joint.iter("child"))]
                        
        for joint in parent_joints:

            #We have reached an acutated joint, i.e. a whole passive chain/link group has been handled
            if(joint.attrib["type"].strip() == "revolute" or joint.attrib["type"].strip() == "prismatic"):
                counter+=1

            #Get joints parent links
            parent_links = [link for link in all_links if any(parent_link.attrib["link"].strip() == link.attrib["name"].strip() for parent_link in joint.iter("parent"))]
            
            #Add parent links to pending links
            links_pending.extend(parent_links)

            #Remove joint so that the passive chain is not handled again
            all_joints.remove(joint)

    #Add ignored collisions to the semantic tree
    for link1 in tool_tree.iter("link"):
        for link2 in robot_links_names_no_collisions:
            robot_semantic_tree.append(ET.Element("disable_collisions",{"link1":link1.attrib["name"],"link2":link2,"reason":"Never"}))

    #Add tip link to the semantic tree
    for chain in robot_semantic_tree.iter("chain"):
        chain.set("tip_link",end_effector_link)

#Clean output
for elem in robot_semantic_tree.iter('*'):
    if elem.attrib is not None:
        for key in elem.attrib:
            if elem.attrib[key][0:3] == "   ":
                elem.set(key, elem.attrib[key][3:])

output = ET.tostring(robot_semantic_tree, encoding="unicode")
sys.stdout.write(output)