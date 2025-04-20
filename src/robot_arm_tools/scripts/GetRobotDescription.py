#! /usr/bin/python3

import sys
import os
import re
from robot_arm_tools.URDFTools import cleanXACROOutput

robot_description = sys.argv[1]
tool_description = cleanXACROOutput(sys.argv[2])

try:
    calibrated_model = sys.argv[3]
    flange_link_name = sys.argv[4]
except:
    calibrated_model = ""
    flange_link_name = ""

import xml.etree.ElementTree as ET
robot_tree = ET.fromstring(robot_description)

if(tool_description != ""):
    tool_tree = ET.fromstring(tool_description)

    for child in tool_tree:
        robot_tree.append(child)

for elem in robot_tree.iter('*'):
    if elem.attrib is not None:
        for key in elem.attrib:
            if elem.attrib[key][0:3] == "   ":
                elem.set(key, elem.attrib[key][3:])

output = ET.tostring(robot_tree,encoding="unicode")

if(calibrated_model != ""):
    from robot_arm_calibration.ModeltoURDF import ModeltoURDF
    
    output = cleanXACROOutput(output)
    os.makedirs("/tmp/tmp_urdf", exist_ok=True)
    with open("/tmp/tmp_urdf/tmp.urdf", "w") as f:
        f.write(output)

    output = ModeltoURDF(calibrated_model, "/tmp/tmp_urdf/tmp.urdf", flange_link_name, "/tmp/tmp_urdf/tmp_output.urdf")

#Adding ROS URDF filename tag
output = re.sub('filename="(?!package://)','filename="file://',output)

sys.stdout.write(output)
