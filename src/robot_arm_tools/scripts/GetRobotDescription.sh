#! /bin/bash

parameters=$(roslaunch $1 --dump-params)

robot_description=$(echo "$parameters" | awk '/robot_description:/,/<\/robot>/' | sed s/"\/robot_description: \""// | sed 's/\\n//g' | sed 's/\\"/"/g' |  sed 's/\\//g' | sed 's/>\"/>/g' | sed 's/'\''//g')
robot_description_semantic=$(echo "$parameters" | awk '/robot_description_semantic:/,/<\/robot>/' | sed s/"\/robot_description_semantic: \""// | sed 's/\\n//g' | sed 's/\\"/"/g' |  sed 's/\\//g' | sed 's/>\"/>/g' | sed 's/'\''//g') 
robot_flange_link=$(echo "$robot_description_semantic" | awk '/tip_link="/,/"/' | sed s/^.*tip_link=\"// | sed s/\".*$//)

input="$2 robot_flange_link:=$robot_flange_link"
tool_description=$(xacro $input 2> /dev/null) || tool_description=""

python3 $(rospack find robot_arm_tools)/scripts/GetRobotDescription.py "$robot_description" "$tool_description" "$3" "$robot_flange_link"