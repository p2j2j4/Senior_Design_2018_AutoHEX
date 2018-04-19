
cmake_minimum_required(VERSION 2.8.3)
project(detect_avoid)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(detect_avoid src/test_flight.cpp)
target_link_libraries(detect_avoid ${catkin_LIBRARIES})
add_dependencies(dectect_avoid detect_avoid_generate_messages_cpp)

install(
	TARGETS detect_avoid
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
