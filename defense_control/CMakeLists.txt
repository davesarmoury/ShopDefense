cmake_minimum_required(VERSION 3.0.2)
project(defense_control)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES defense_control
#  CATKIN_DEPENDS geometry_msgs trac_ik_python trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
