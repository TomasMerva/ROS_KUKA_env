cmake_minimum_required(VERSION 2.8.3)
project(kuka_push)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  visualization_msgs
)

find_package(gazebo 9 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)




### Generate messages in the 'msg' folder
add_message_files(
  FILES
  rlplugin_msg.msg
  object_pose_vel.msg
)

### Generate services in the 'srv' folder
add_service_files(
  FILES
  rl_env.srv
  object_reset.srv
)

### Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include/kuka_push
  LIBRARIES
  CATKIN_DEPENDS
    gazebo_ros
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    rospy
    std_msgs
    trajectory_msgs
    visualization_msgs
#  DEPENDS system_lib
)

add_library(pushobject_plugin plugin/pushobject.cpp)
target_link_libraries(pushobject_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(goalgazebo_plugin plugin/goalgazebo.cpp)
target_link_libraries(goalgazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(kukapush_env_lib src/kukapush_env_lib.cpp)
add_library(goal src/marker.cpp)

add_executable(kukapush_env src/kukapush_env.cpp )
target_link_libraries(kukapush_env
                      kukapush_env_lib
                      goal
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${EIGEN3_INCLUDE_DIRS})
