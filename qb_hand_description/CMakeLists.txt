cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_hand_description VERSION 2.1.2 LANGUAGES CXX)

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

install(
  DIRECTORY
    urdf/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(
  DIRECTORY
    meshes/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(
  DIRECTORY
    config/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

