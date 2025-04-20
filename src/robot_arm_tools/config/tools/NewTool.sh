#!/bin/bash

#$1 = Path to the new tool CAD file (.stl - named after the new tool - meters)
#$2 $3 $4 $5 $6 $7 = Tool end effector frame pose regarding to the robot flange frame
#$8 = Safety distance for collisions
#$9 = Mass of the new tool
#$10 (optional) = Name of the robot

#Setup...

Help()
{
    #Display help
    echo "Script creating all configuration files for a new tool"
    echo "parameters :"
    echo "\$1 = Path to the new tool CAD file (.stl - named after the new tool - meters)"
    echo "\$2 \$3 \$4 \$5 \$6 \$7 = Tool end effector frame pose regarding to the robot flange frame"
    echo "\$8 = Safety distance for collisions"
    echo "\$9 = Mass of the new tool"
}

while getopts ":h" option; do
   case $option in
      h) # display help
         Help
         exit;;
   esac
done

robot_arm_tools_location=$(rospack find robot_arm_tools) || exit 1
echo "[NewTool] robot_arm_tools package found at $robot_arm_tools_location"

raw_tool_name=$(basename $1) || exit 1
tool_name=${raw_tool_name%".stl"}

if [ $tool_name = $raw_tool_name ]; then
    [ echo "Invalid file type : .stl only !" && exit 1 ]
fi

answer="y"

mkdir $robot_arm_tools_location/config/tools/$tool_name || { read -e -p "Tool already exists ! Overwrite (y/n) ?" answer; }

if [[ $answer != "y" && $answer != "Y" && $answer != "" ]]; then

    echo "[NewTool] Using alredy existing tool $tool_name"

else

    echo "[NewTool] Starting setup of new tool $tool_name"

    #Step 1 - Create the tool .xacro description file

    echo "[NewTool] Creating $tool_name.xacro ..."

    cp $robot_arm_tools_location/config/tools/template/template.xacro $robot_arm_tools_location/config/tools/$tool_name/$tool_name.xacro 

    sed -i "s/\$1/$tool_name/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.xacro 

    #Step 2 - Handling the .stl mesh file : creating the .json configuration file and completing the .xacro description file

    echo "[NewTool] Calling the .stl mesh handling Python script"

    python3 $robot_arm_tools_location/config/tools/STLProcessing.py $1 $2 $3 $4 $5 $6 $7 $9

    #Step 3 - Creating the tool.urdf.xacro description file

    echo "[NewTool] Creating $tool_name.urdf.xacro"

    cp $robot_arm_tools_location/config/tools/template/template.urdf.xacro $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro

    sed -i "s/\$1/$tool_name/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$2/$2/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$3/$3/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$4/$4/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$5/$5/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$6/$6/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$7/$7/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro
    sed -i "s/\$8/$8/g" $robot_arm_tools_location/config/tools/$tool_name/$tool_name.urdf.xacro

fi

#Step 4 - Moving the .stl mesh file to the new tool folder 

cp $1 $robot_arm_tools_location/config/tools/$tool_name/$tool_name.stl