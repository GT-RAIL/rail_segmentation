cmake_minimum_required(VERSION 2.8.3)
project(rail_segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  SegmentedObject.msg
  SegmentedObjectList.msg
)

add_service_files(
  FILES
  Segment.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rail_segmentation src/segmentation.cpp)
add_dependencies(rail_segmentation ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rail_segmentation ${catkin_LIBRARIES})

