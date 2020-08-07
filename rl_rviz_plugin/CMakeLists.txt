cmake_minimum_required(VERSION 2.8.3)
project(rl_rviz_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_AUTOMOC ON)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rviz
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  trajectory_msgs
  std_msgs
  geometry_msgs
  kuka_push
)


find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Boost REQUIRED system filesystem date_time thread)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)


catkin_package(
  #INCLUDE_DIRS include/rl_rviz_plugin
  #LIBRARIES rl_rviz_plugin
  CATKIN_DEPENDS
    message_generation
    roscpp
    rviz
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    trajectory_msgs
    std_msgs
    geometry_msgs
    kuka_push
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
#add_dependencies(rl_rviz_plugin )

add_library(rl_rviz_plugin  src/rl_rviz_plugin.cpp
                            include/rl_rviz_plugin/rl_rviz_plugin.h
                           )
target_link_libraries(rl_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

