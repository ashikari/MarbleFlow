cmake_minimum_required(VERSION 2.8.3)
project(ros_prac)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES ros_prac
#  CATKIN_DEPENDS roscpp rospy std_msgs actionlib sensor_msgs control_msgs trajectory_msgs cv_bridge dynamic_reconfigure intera_core_msgs intera_motion_msgs intera_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(send_ball_pos src/send_ball_pos.cpp)
target_link_libraries(send_ball_pos ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(send_ball_pos opencv)

add_executable(blob_error_pub src/blob_error_pub.cpp)
target_link_libraries(blob_error_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
add_dependencies(blob_error_pub opencv realsense2)