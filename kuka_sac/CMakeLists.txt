cmake_minimum_required(VERSION 2.8.3)
project(kuka_sac)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  tf
  trajectory_msgs
  visualization_msgs
  kuka_push
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
    FILES
    service.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include/kuka_sac
  LIBRARIES kuka_env_interface kuka_env_movement
  CATKIN_DEPENDS
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    rospy
    tf
    trajectory_msgs
    visualization_msgs
    kuka_push
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_library(goal_env src/marker.cpp)
add_library(kuka_env_interface src/kuka_env_interface.cpp)
add_library(kuka_env_movement src/kuka_env_movement.cpp)


add_executable(kuka_env src/kuka_env.cpp )
target_link_libraries(kuka_env
                      kuka_env_interface
                      kuka_env_movement
                      goal_env
    ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
