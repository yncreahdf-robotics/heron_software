cmake_minimum_required(VERSION 2.8.3)
project(heron)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES heron_package
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS})

# for motor control node
set(${PROJECT_NAME}_SOURCES
    src/nodes/MotorControl/drive
    src/nodes/MotorControl/Constants
    src/nodes/MotorControl/ErrorCodes
    src/nodes/MotorControl/RoboteqDevice)
    
add_executable(drive ${${PROJECT_NAME}_SOURCES})

target_link_libraries(drive ${catkin_LIBRARIES})



