cmake_minimum_required(VERSION 2.8.3)
project(ros_cellulo_practical)

### Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
### Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#add_definitions(-DQT_NO_KEYWORDS)
## Use C++11
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS   
    message_generation
    roscpp
    sensor_msgs
    std_msgs
    tf2_ros
    nav_msgs
    visualization_msgs
    geometry_msgs
    ros_cellulo_swarm
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS
    include
  #LIBRARIES ros_cellulo_practical
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
    tf2_ros
    visualization_msgs
    ros_cellulo_swarm

#  DEPENDS
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


######################
###leader selection###
######################

# Declare a cpp library
add_executable(ros_cellulo_leader
   src/ros_cellulo_interaction/ros_cellulo_leader_node.cpp
)
target_link_libraries(ros_cellulo_leader
  ${catkin_LIBRARIES}
)

#################
###interaction###
#################

# Declare a cpp library

add_executable(ros_cellulo_interaction
   src/ros_cellulo_interaction/ros_cellulo_interaction_node.cpp
)

target_link_libraries(ros_cellulo_interaction
  ${catkin_LIBRARIES}
)


#################
###aggregation###
#################

# Declare a cpp library
add_library(ros_cellulo_aggregation_core
   include/ros_cellulo_aggregation/RosCelluloAggregation.hpp
)

set_target_properties(ros_cellulo_aggregation_core PROPERTIES LINKER_LANGUAGE CXX)

add_executable(ros_cellulo_aggregation
   src/ros_cellulo_aggregation/ros_cellulo_aggregation_node.cpp
   src/ros_cellulo_aggregation/RosCelluloAggregation.cpp
)

target_link_libraries(ros_cellulo_aggregation
  ros_cellulo_aggregation_core
  ${catkin_LIBRARIES}
)

##############
###coverage###
##############

# Declare a cpp library
add_library(ros_cellulo_coverage_core
   include/ros_cellulo_coverage/RosCelluloCoverage.hpp
)

set_target_properties(ros_cellulo_coverage_core PROPERTIES LINKER_LANGUAGE CXX)

add_executable(ros_cellulo_coverage
   src/ros_cellulo_coverage/ros_cellulo_coverage_node.cpp
   src/ros_cellulo_coverage/RosCelluloCoverage.cpp
)

target_link_libraries(ros_cellulo_coverage
  ros_cellulo_coverage_core
  ${catkin_LIBRARIES}
)




