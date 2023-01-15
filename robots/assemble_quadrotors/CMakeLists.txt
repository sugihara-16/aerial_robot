cmake_minimum_required(VERSION 2.8.3)
project(assemble_quadrotors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  hydrus
  aerial_robot_estimation
  )

find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp aerial_robot_control aerial_robot_model aerial_robot_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########
add_executable(optimal_design_node src/optimal_design.cpp)
target_link_libraries(optimal_design_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(optimal_thrust_node src/optimal_thrust.cpp)
target_link_libraries(optimal_thrust_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(assemble_mocap_pluginlib src/sensor/mocap.cpp)
target_link_libraries(assemble_mocap_pluginlib ${catkin_LIBRARIES})
add_dependencies(assemble_mocap_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(assemble_robot_model src/model/assemble_robot_model.cpp)
target_link_libraries(assemble_robot_model ${catkin_LIBRARIES})

add_library(assemble_flight_navigation src/navigation/assemble_flight_navigation.cpp)
target_link_libraries(assemble_flight_navigation ${catkin_LIBRARIES})

add_library(assemble_controller src/control/assemble_controller.cpp)
target_link_libraries(assemble_controller ${catkin_LIBRARIES})


