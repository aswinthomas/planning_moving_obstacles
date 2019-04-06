cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_moving_obstacles)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  backward_ros
)

find_package(gazebo REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planning_moving_obstacles
  CATKIN_DEPENDS roscpp std_msgs gazebo_ros 
#  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(box_animator src/box_animator.cc)
target_link_libraries(box_animator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(aruco_follower_node src/aruco_follower.cc src/aruco_follower_node.cc)
target_link_libraries(aruco_follower_node ${catkin_LIBRARIES})
