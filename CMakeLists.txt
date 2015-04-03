cmake_minimum_required(VERSION 2.8.3)
project(rail_segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rail_manipulation_msgs
  sensor_msgs
  std_srvs
  tf
  tf2
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RemoveObject.srv
)

generate_messages()

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Check for what version of YAML we have
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DYAMLCPP_GT_0_5_0)
endif()

## Specify additional locations of header files
include_directories(include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  nodes/${PROJECT_NAME}.cpp
  src/SegmentationZone.cpp
  src/Segmenter.cpp
)

## Add message build dependencies (needed for source build)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  rail_manipulation_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.yaml"
)