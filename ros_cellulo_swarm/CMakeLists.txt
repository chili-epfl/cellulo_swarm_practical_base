cmake_minimum_required(VERSION 2.8.3)
project(ros_cellulo_swarm)

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
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
    nav_msgs
    visualization_msgs
    geometry_msgs
    #ros_cellulo_msgs

)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES   
   cellulo_visual_effect.msg
   cellulo_touch_key.msg
   cellulo_kidnapped_msg.msg
   cellulo_vibrateOnMotion.msg
   cellulo_coord.msg
   cellulo_simpleVibrate.msg
   cellulo_pose_velocity.msg
   ros_cellulo_sensor.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    CelluloState.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
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
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy 
    roscpp
    message_runtime
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
    tf2_ros
    visualization_msgs
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

############
###type 1###
############

## Declare a cpp library
add_library(ros_cellulo_sensor_core
   include/ros_cellulo_sensor/RosCelluloSensor.hpp
)
set_target_properties(ros_cellulo_sensor_core PROPERTIES LINKER_LANGUAGE CXX)

add_executable(ros_cellulo_sensor
   src/ros_cellulo_sensor/ros_cellulo_sensor_node.cpp
   src/ros_cellulo_sensor/RosCelluloSensor.cpp
)

target_link_libraries(ros_cellulo_sensor
  ros_cellulo_sensor_core
  ${catkin_LIBRARIES}
)

 install(PROGRAMS
   src/cellulo-ros-python.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 install(PROGRAMS
   teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
