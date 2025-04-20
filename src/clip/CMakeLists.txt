cmake_minimum_required(VERSION 3.0.2)
project(clip)

## 找到 catkin 依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

## 声明 catkin 包
catkin_package()

## 让 ROS 识别 Python 脚本
catkin_install_python(PROGRAMS
  scripts/ros_image_subscriber.py
  scripts/ros_deep_learning_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
