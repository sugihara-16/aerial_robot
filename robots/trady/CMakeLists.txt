cmake_minimum_required(VERSION 2.8.3)
project(trady)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)

