cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_hand_description VERSION 1.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED)

catkin_package()

# Installation
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)