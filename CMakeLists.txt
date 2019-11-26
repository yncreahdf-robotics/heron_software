cmake_minimum_required(VERSION 2.8.3)
project(heron)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  message_generation)

catkin_python_setup()

add_message_files(
  DIRECTORY
  msg

  FILES
  Encoders.msg
  Battery.msg
  ValueUnitFloat.msg
  ValueUnitInt.msg
  IRSensors.msg
  USIRSensors.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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
    src/nodes/MotorControl/RoboteqDevice
    src/nodes/MotorControl/RobotSpecs)
    
add_executable(drive ${${PROJECT_NAME}_SOURCES})
target_link_libraries(drive ${catkin_LIBRARIES})
add_dependencies(drive heron_generate_messages_cpp)


add_executable(odom src/nodes/MotorControl/odom.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})
add_dependencies(odom heron_generate_messages_cpp)

add_executable(tf_broadcaster src/nodes/tf/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
#add_dependencies(tf_broadcaster heron_generate_messages_cpp)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)




