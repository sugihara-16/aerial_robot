cmake_minimum_required(VERSION 3.0.2)
project(twin_hammer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  gimbalrotor
  diagnostic_msgs
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES twin_hammer_model twin_hammer_controller
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor diagnostic_msgs std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(twin_hammer_controller src/control/twin_hammer_controller.cpp)
target_link_libraries(twin_hammer_controller ${catkin_LIBRARIES})

add_library(twin_hammer_model src/model/twin_hammer_model.cpp)
target_link_libraries(twin_hammer_model ${catkin_LIBRARIES})

install(TARGETS twin_hammer_model twin_hammer_controller 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
