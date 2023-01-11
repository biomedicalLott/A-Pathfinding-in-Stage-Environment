cmake_minimum_required(VERSION 3.1)
project(lab4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-g") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3)

######################################
# Compile the library >
######################################

# Add includes
include_directories(include)
######################################
# < End compile the library
######################################

######################################
# ROS >
######################################

# Only run this code in the catkin environment
if(DEFINED CATKIN_DEVEL_PREFIX)
  find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    interactive_markers
    visualization_msgs
    std_msgs
    message_generation
    )

  generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    )


  # Include catkin_libraries
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS message_runtime
  )
  set(LIBS ${LIBS} ${catkin_LIBRARIES})
  include_directories(include ${catkin_INCLUDE_DIRS})


  

  # Add the nodes
  file(GLOB NODE_SRC_FILES node/*.cpp)
  foreach(_node_file ${NODE_SRC_FILES})
      get_filename_component(_node_name ${_node_file} NAME_WE)
      add_executable(${_node_name} ${_node_file})
      target_link_libraries(${_node_name} ${LIBS})

  endforeach()
endif()

######################################
# < End ROS
######################################
