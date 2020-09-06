cmake_minimum_required(VERSION 3.0.2)
project(static_avoidance)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  obstacle_detector
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg FILES
   AckermannDrive.msg
   AckermannDriveStamped.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   ackermann_msgs   geometry_msgs   sensor_msgs   std_msgs   obstacle_detector
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES static_avoidance
  CATKIN_DEPENDS geometry_msgs message_generation roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(static_avoidance src/static_avoidance.cpp)
add_dependencies(static_avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_avoidance ${catkin_LIBRARIES})

