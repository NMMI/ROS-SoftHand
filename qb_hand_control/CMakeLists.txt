cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_hand_control VERSION 1.0.1 LANGUAGES CXX)

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
    lib${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    qb_device_control
    qb_hand_hardware_interface
)

# Build Settings
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/qb_hand_control.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Exported libraries
add_library(lib${PROJECT_NAME}
  src/qb_hand_control.cpp
)

add_dependencies(lib${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(lib${PROJECT_NAME}
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}  # avoid "liblib" name
)

# C++ Settings
set_property(
  TARGET
    ${PROJECT_NAME}
    lib${PROJECT_NAME}
  PROPERTY CXX_STANDARD 14
)

set_property(
  TARGET
    ${PROJECT_NAME}
    lib${PROJECT_NAME}
  PROPERTY CXX_STANDARD_REQUIRED ON
)