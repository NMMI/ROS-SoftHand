cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_hand_control VERSION 1.0.4 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    qb_device_control
    qb_hand_hardware_interface
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    qb_device_control
    qb_hand_hardware_interface
)

# Build Settings
add_executable(qb_hand_default
  src/main.cpp
  src/qb_hand_control.cpp
)

target_link_libraries(qb_hand_default
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(qb_hand_default
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/qb_hand_control.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
set_property(
  TARGET
    qb_hand_default
    ${PROJECT_NAME}
  PROPERTY CXX_STANDARD 14
)

set_property(
  TARGET
    qb_hand_default
    ${PROJECT_NAME}
  PROPERTY CXX_STANDARD_REQUIRED ON
)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)